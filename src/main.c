/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// Show how to reconfigure and restart a channel in a channel completion
// interrupt handler. Plus prepare a DMX-512 buffer for 16 univereses
// before triggering each DMA transfer
//
// Our DMA channel will transfer data to a PIO state machine, which is
// configured to serialise the raw bits that we push, one by one, 16 bits in
// parallel to 16 GPIOs (16 DMX universes).
//
// Once the channel has sent a predetermined amount of data (1 DMX packet), it
// will halt, and raise an interrupt flag. The processor will enter the 
// interrupt handler in response to this, where it will:
// - Toggle GP28 LOW
// - Zero the complete wave table
// - Prepare the next DMX packet to be sent in the wavetable
// - Sets GP28 HIGH (so we can trigger a scope on it)
// - Restart the DMA channel
// This repeats.

#include <stdio.h>
#include "hardware/clocks.h"    // To derive our 250000bit/s from sys_clk
#include "hardware/dma.h"       // To control the data transfer from mem to pio
#include "hardware/gpio.h"      // To "manually" control the trigger pin
#include "hardware/i2c.h"       // Detect, read and write the EEPROMs on the IO-boards
#include "hardware/irq.h"       // To control the data transfer from mem to pio
#include "tx16.pio.h"           // Header file for the PIO program
#include "ws2812.pio.h"           // Header file for the PIO program

#include "pico/stdlib.h"
#include "stdio.h"

#include "stdio_usb.h"

#include "log.h"

#include "dmahandler.h"

#include "acminterface.h"

#include "wireless.h"

#include "bsp/board.h"          // LED timing
#include <tusb.h>

/* Blink pattern
 * - 250 ms  : Sending DMX, universe 1-15 has one value != 0
 * - 1000 ms : Sending DMX, universe 0 has one value != 0
 * - 2500 ms : Sending DMX, all universes are zero
 */

enum {
    BLINK_SENDING_ZERO         = 1000,
    BLINK_SENDING_CONTENT_ONE  =  500,
    BLINK_SENDING_CONTENT_MORE =  100,
};

static uint32_t blink_interval_ms = BLINK_SENDING_ZERO;

static uint32_t debug_refresh_interval_ms = 100;

const int PIN_LEDs = 22;

int dma_chan;                          // The DMA channel we use to push data around
uint8_t dmx_values[16][512];           // 16 universes with 512 byte each
uint8_t ioboards[8][256];              // Content of the IO-board EEPROMS

void led_blinking_task(void);

static inline void put_pixel(uint32_t pixel_grb) {
    pio_sm_put_blocking(pio1, 3, pixel_grb << 8u);
}
static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return
            ((uint32_t) (r) << 8) |
            ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

int main() {
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    tusb_init();

    stdio_usb_init();

    // Create the state machine for the status LEDS
    int sm = 3;
    uint offsetsled = pio_add_program(pio1, &ws2812_program);
    ws2812_program_init(pio1, sm, offsetsled, PIN_LEDs, 800000, false);

    put_pixel(0x10101);
    put_pixel(0x10101);
    put_pixel(0x10101);

    while (!tud_cdc_connected()) {
        // Wait here until CDC connected
    }

    // Scan for io boards
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    // Pull-ups are populated on Rev 0.1 base boards ....
    gpio_pull_up(0);
    gpio_pull_up(1);
    memset(ioboards, 0x00, 8*256);
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    for (int addr = 80; addr < 88; ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        int ret;
        uint8_t src = 0;
        // Set EEPROM address to read from
        i2c_write_blocking(i2c0, addr, &src, 1, false);
        // Try to read the EEPROM data
        ret = i2c_read_blocking(i2c0, addr, ioboards[addr - 0x50], 256, false);
        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");

    // Output the content of the EEPROMS
    for (uint8_t board = 0; board < 8; board++) {
        printf("IOBOARD %d: ", board);
        for (uint16_t byte = 0; byte < 256; byte++) {
            printf("%02x", ioboards[board][byte]);
        }
        printf("\n");
    }

    // Set up our TRIGGER GPIO on GP28 and init it to LOW
    gpio_init(PIN_TRIGGER);
    gpio_set_dir(PIN_TRIGGER, GPIO_OUT);
    gpio_put(PIN_TRIGGER, 0);

    // Set up a PIO state machine to serialise our bits at 250000 bit/s
    uint offset = pio_add_program(pio0, &tx16_program);
    float div = (float)clock_get_hz(clk_sys) / 250000;
    tx16_program_init(pio0, 0, offset, div);

    // Configure a channel to write the wavetable to PIO0
    // SM0's TX FIFO, paced by the data request signal from that peripheral.
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true); // TODO: is by default. Line needed?
    channel_config_set_dreq(&c, DREQ_PIO0_TX0);

    dma_channel_configure(
        dma_chan,
        &c,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        NULL,             // Don't provide a read address yet
        WAVETABLE_LENGTH/2, // Write one complete DMX packet, then halt and interrupt
                          // It's WAVETABLE_LENGTH/2 since we transfer 32 bit per transfer
        false             // Don't start yet
    );

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(dma_chan, true);

    // Configure the processor to run dma_handler() when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // Manually call the handler once, to trigger the first transfer
    dma_handler();

    LOG("Calling wirelessInit() ..."); printLogBuffer(); clearLogBuffer();
    wirelessInit();
    LOG("Done!"); printLogBuffer(); clearLogBuffer();

    // Everything else from this point is interrupt-driven. The processor has
    // time to sit and think about its early retirement -- maybe open a bakery?
    while (true) {
        tud_task();
        led_blinking_task();
    }
};

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
    // The following calculations take lots of time. However, this doesn't
    // matter since the DMX updating is done via IRQ handler

    // Check which blinking pattern to use
    blink_interval_ms = BLINK_SENDING_ZERO;

    // Check if first universe is all zero
    for (uint16_t i = 0; i < 512; i++) {
        if (dmx_values[0][i]) {
            blink_interval_ms = BLINK_SENDING_CONTENT_ONE;
        }
    }
    // Check the other universes
    for (uint16_t j = 1; j < 16; j++) {
        for (uint16_t i = 0; i < 512; i++) {
            if (dmx_values[j][i]) {
                blink_interval_ms = BLINK_SENDING_CONTENT_MORE;
            }
        }
    }

    static uint32_t start_ms = 0;
    static bool led_state = false;

    // Blink every interval ms
    if (board_millis() - start_ms < blink_interval_ms) return; // not enough time
    start_ms += blink_interval_ms;

    board_led_write(led_state);
    led_state = 1 - led_state; // toggle


    // Output current DMX universe values periodically
    static uint32_t start_ms_debug = 0;

    if (board_millis() - start_ms_debug < debug_refresh_interval_ms) return; // not enough time
    start_ms_debug += debug_refresh_interval_ms;

    wirelessSend();

    // Debugging stuff
    /*
    LOGuni(0, dmx_values[0]);
    LOGuni(1, dmx_values[1]);
    */

    /* Logging demo:
    LOG("Hello world!");
    LOGfmt("%d * %02x = %d", 3, 5 , 3*5);
    */

    //printLogBuffer();
    clearLogBuffer();
}
