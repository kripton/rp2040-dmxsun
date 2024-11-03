/**
 * Copyright (c) 2021 Open Lighting Project
 *
 * SPDX-License-Identifier: GPL-3.0
 */

extern "C" {
#include <stdio.h>
#include <hardware/clocks.h>    // Needed for the onboard LED blinking patterns
#include <hardware/dma.h>       // To control the data transfer from mem to pio
#include <hardware/gpio.h>      // To "manually" control the trigger pin
#include <hardware/adc.h>       // Pico-W runtime detection

#include <pico/stdlib.h>
#include "pico/multicore.h"

#include <pico/cyw43_arch.h>    // Toggle the LED on the Pico-W

#include "pins.h"
#include "picotool_binary_information.h"

#include "stdio_usb.h"
}

#include "log.h"
#include "dmxbuffer.h"
#include "statusleds.h"
#include "boardconfig.h"
#include "webserver.h"
#include "wireless.h"
#include "localdmx.h"
#include "eth_cyw43.h"
#include "eth_w5500.h"
#include "oled_u8g2.h"

#include "dhcpdata.h"

#include "usb_EDP.h"
#include "usb_NodleU1.h"

#include "udp_artnet.h"
#include "udp_e1_31.h"
#include "udp_edp.h"

extern "C" {
#include <bsp/board.h>          // On-board-LED
#include <tusb.h>
}

/* On-board LED blinking patterns
 * The values here are clock dividers for the on-chip RTC-clock running at
 * 46875Hz. Using that value gives a "PWM" with 1s cycle time and 50% duty cycle
 *
 * DIVIDER | Cycle time | Description
 * 2000    |   ~43ms    | Init phase, board not ready
 * 65535   | ~1.4s      | Board ready, all chans in all universes are 0
 * 42188   | ~0.9s      | Board ready, one universe with at least one channel != 0
 * 21094   | ~0.45s     | Board ready, multiple universes with at least one channel != 0
 */
// TODO: Replace the clock-derived LED-blinking with a software-controlled one
//       Reason: If the board crashed, the LED currently blinks on. It should stop doing so
enum {
    BLINK_INIT                 =  2000,
    BLINK_READY_NO_DATA        = 65535,
    BLINK_READY_SINGLE_UNI     = 42188,
    BLINK_READY_MULTI_UNI      = 21094,

};
#define BLINK_LED(div) clock_gpio_init(PIN_LED_PICO, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_RTC, div);

// Super-globals (for all modules)
Log logger;
DmxBuffer dmxBuffer;
LocalDmx localDmx;
StatusLeds statusLeds;
Oled_u8g2 oled_u8g2;
BoardConfig boardConfig;
WebServer webServer;
Wireless wireless;
DhcpData dhcpdata;
Eth_cyw43 eth_cyw43;
Eth_W5500 eth_w5500;

critical_section_t bufferLock;

uint8_t usbTraffic = 0;

struct repeating_timer led_toggle_timer;

bool led_toggle_timer_callback(repeating_timer_t *rt);
void led_blinking_task(void);

void core1_tasks(void);

// Board init sequence:
// 1. Status LEDs
// 2. Detect IO boards
// 3. Read board configuration from "first" IO board
// 3b. If no IO board was detected: Read board config from last sector of on-board flash
//     (This could be the case for nRF24 mesh masters with attached PC and no local IO
//      or nRF24 mesh repeaters with no local IO)
// 4. Depending on config: USB host-interface (style of emulation)
// 5. tusb_init(), stdio_usb_init() and configure magic-baudrate-reboot
// 6. Depending on config (IP addresses): USB-Network-Web-Server
// 7. nRF24 detection and init
// 8. Depending on base boards and config: DMX PIOs and GPIO config

int main() {
    // Runtime-detect which board we are running on
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(29);
    // Select ADC input 3 (GPIO29)
    adc_select_input(3);
    // Give some time for the voltage to stabilize and the ADC to sample
    sleep_ms(5);
    uint16_t firstRead = adc_read();
    BoardConfig::boardIsPicoW = (firstRead < 1000);

    // Make the onboard-led blink like crazy during the INIT phase
    // without having to do this in software because we're busy with other stuff
    if (!BoardConfig::boardIsPicoW) {
        BLINK_LED(BLINK_INIT);
    }

    // /!\ Do NOT use LOG() until TinyUSB-stack has been initialized /!\

    // Phase 0: Overclock the board to 250MHz. According to
    //          https://www.youtube.com/watch?v=G2BuoFNLo this should be
    //          totally safe with the default 1.10V Vcore
    set_sys_clock_khz(250000, true);

    // Phase 1: Init the status LEDs
    statusLeds.init();
    statusLeds.setBrightness(20);
    statusLeds.writeLeds();

    // Phase 2: Detect and read IO boards
    boardConfig.init();
    boardConfig.readIOBoards();

    oled_u8g2.scanBusForOLED();
    if (oled_u8g2.oledAvailable) {
        oled_u8g2.init();
    }

    // Phase 2b: Init our DMX buffers
    critical_section_init(&bufferLock);
    dmxBuffer.init();

    // Phase 3: Make sure we have some configuration ready (includes Phase 3b)
    boardConfig.prepareConfig();

    // Phase 4, USB configuration happens in usb_descriptors (boardConfig is queried)
    //          However, we would need to instantiate the relevant class here
    Usb_EDP::init();
    Usb_NodleU1::init();

    // Phase 5: Enable the USB interface, the debugging console, ...
    tusb_init();
    stdio_usb_init();
    logger.init();

    // Phase 6: Fire up the integrated web server
    // This also initialises the TinyUSB<->lwIP glue. lwIP and the DHCP server
    dhcpdata.init();

    if (BoardConfig::boardIsPicoW) {
        // pico_cyw43_arch automatically and unconditionally calls lwip_init ...
        eth_cyw43.init();
    } else {
        // Since we didn't init pico_cyw43_arch, we call lwip_init now
        // TODO?: Not using an async_context as of now
        lwip_init();
    }

    init_tinyusb_netif();   // Init TinyUSB's lwip integration and the netif
    wait_for_netif_is_up(); // TinyUSB network interface
    dhcpd_init();

    webServer.init();

    // Initialize the Ethernet module
    eth_w5500.init();

    // TODO: Check init order and if W5500 and nRF24 cooperate!

    // Phase 7: Detect if there is a radio module and init it if so
    wireless.init();

    // Phase 8: Set up PIOs and GPIOs according to the IO boards
    localDmx.init();

    // Re-init the PICO-LED to a normal LED
    gpio_init(PIN_LED_PICO);
    gpio_set_dir(PIN_LED_PICO, GPIO_OUT);
    // Set up a timer to make status LED blink
    add_repeating_timer_ms(25, led_toggle_timer_callback, NULL, &led_toggle_timer);

    // Phase 9: Do all the patching between the internal DMX buffers and ports
    // Patching is read from BoardConfig and actually nothing needs to be done here

    // Phase 10: Start our ArtNet- and E1.31 receiver
    Udp_ArtNet::init();
    Udp_E1_31::init();
    Udp_EDP::init();

    // Finally, turn on the green component of the SYSTEM status LED
    statusLeds.setStaticOn(4, 0, 1, 0);
    sleep_ms(10);
    statusLeds.writeLeds();

    // SETUP COMPLETE
    LOG("SYSTEM: SETUP COMPLETE :D ADC read: %d", firstRead);

    // Run all important tasks at least once before we start AUX tasks on core1
    // so the USB device enumeration doesn't time-out
    tud_task();
    tud_task();
    tud_task();
    webServer.cyclicTask();

    // Now get core1 running ...
    LOG("SYSTEM: Starting core 1 ...");
    multicore_launch_core1(core1_tasks);

    LOG("SYSTEM: Time to party, entering main loop");

    // Enter the main loop on core0. localDmx (PIO) is interrupt driven.
    // Everything else (I assume) is polled and handled here.
    // Wireless is on core1 so waiting for ACKs won't slow down everything else
    while (true) {
        tud_task();

        if (tud_mounted()) {
            statusLeds.setStaticOn(5, 0, 1, 0);
        } else {
            statusLeds.setStaticOff(5, 0, 1, 0);
        }

        webServer.cyclicTask(); // Make sure this is on core0 since it
                                // WILL halt core1 when writing to the flash!

        if (BoardConfig::boardIsPicoW) {
            eth_cyw43.cyclicTask();
        }
//        wireless.cyclicTask();
//        statusLeds.cyclicTask();
//        led_blinking_task();
//        sleep_us(10);
    }
};

// Core1 handles wireless (which can delay quite a bit) + status LEDs
void core1_tasks() {
    while (true) {
//        tud_task();
//        webServer.cyclicTask();
        wireless.cyclicTask();
        statusLeds.cyclicTask();
        led_blinking_task();
//        sleep_us(10);
    }
};

// LED toggle task
bool led_toggle_timer_callback(repeating_timer_t *rt) {
    if (BoardConfig::boardIsPicoW) {
        cyw43_arch_gpio_put(PIN_LED_PICOW, !cyw43_arch_gpio_get(PIN_LED_PICOW));
    } else {
        gpio_put(PIN_LED_PICO, !gpio_get(PIN_LED_PICO));
    }
    return true;
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
    // The following calculations take lots of time. However, this doesn't
    // matter since the DMX updating is done via IRQ handler

    // TODO: This iteration through all universes takes quite much time.
    //       Make the DmxBuffer remember for each universe if it's all 0s
    //       and read that property here

    uint universes_none_zero = 0;
    // Check the universes for non-zero channels
    for (uint16_t j = 0; j < 16; j++) {
        for (uint16_t i = 0; i < 512; i++) {
            if (dmxBuffer.buffer[j][i]) {
                universes_none_zero++;
                break;
            }
        }
        if (universes_none_zero > 4) {
            break;
        }
    }

    if (universes_none_zero == 0) {
        led_toggle_timer.delay_us = 1200 * 1000;
        statusLeds.setStatic(7, 0, 0, 0);
    } else if (universes_none_zero == 1) {
        led_toggle_timer.delay_us = 800 * 1000;
        statusLeds.setStatic(7, 0, 1, 0);
    } else if (universes_none_zero > 4) {
        led_toggle_timer.delay_us = 300 * 1000;
        statusLeds.setStatic(7, 1, 1, 1);
    } else if (universes_none_zero > 1) {
        statusLeds.setStatic(7, 0, 0, 1);
    }
}
