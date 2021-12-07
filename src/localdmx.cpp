#include "log.h"
#include "localdmx.h"

#include <string.h>

#include <hardware/clocks.h>    // To derive our 250000bit/s from sys_clk
#include <hardware/dma.h>       // To control the data transfer from mem to pio
#include <hardware/gpio.h>      // To "manually" control the trigger pin
#include <hardware/irq.h>       // To control the data transfer from mem to pio

#include "debug_struct.h"
extern struct DebugStruct debugStruct;

// TEMPORARY for DmxInput
#include "dmxbuffer.h"
extern DmxBuffer dmxBuffer;
// /TEMPORARY for DmxInput

#include "tx_dmx.pio.h"           // Header file for the PIO program
#include "rx_dmx.pio.h"           // Header file for the PIO program

extern LocalDmx localDmx;

extern critical_section_t bufferLock;

uint8_t LocalDmx::buffer[LOCALDMX_COUNT][512];
uint16_t LocalDmx::wavetable[WAVETABLE_LENGTH];  // 16 universes (data type) with 5648 bit each

// DMX OUT-ONLY (max 16 universes) are on PIO 1, SM 2 and use DMA IRQ 0
// STATUS LEDs (ws2812) are on PIO 1, SM 3, no DMA and no IRQ

// So, we have 7 state machines for "local output"
// - WS2812 LEDs (besides) the status LEDs work but won't be supported for now.
//   The question for them is if we need one STATE MACHINE for each string OR
//   if we can drive multiple ones from one SM
// - RDM ports needs ONE STATE MACHINE each (to be able to switch from TX to RX)
//   As such, we can AT MOST support 7 RDM ports
// - DMX IN ports will most probably also need 1 SM / port. I assume.
//   Could also be possible to sample multiple GPIOs with one SM and do the
//   decoding in software
// - DMX OUTs (consecutive ports) can be done with ONE STATE MACHINE

// FOR NOW we are NOT respecting the connected IO modules and just assume
// we have 16 outputs. This will make RDM ports FAILto send properly




// ---------------- The following is the explanation when we only have 16 OUTs
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

void LocalDmx::init() {
    // TODO: According to the BoardConfig (Which type of IO board is 
    //       attached to which slot), check for VALIDITY and configure
    //       the PIO state machines accordingly

    initRxDmx(14);

    // TODO: Make pin base and count depending on board config
    // TODO: /!\ Temporarily reduced to 8 universes so we an play around with GPIOs 14-21 (DMX IN ;) /!\ //
    initTxDmx(6, 8);
    //initTxDmx(6, 16);
}

void LocalDmx::initRxDmx(uint pin) {
    uint offset = pio_add_program(pio0, &rx_dmx_program);

    // Set this pin's GPIO function (connect PIO to the pad)
    pio_sm_set_consecutive_pindirs(pio1, 0, pin, 1, false);
    pio_gpio_init(pio0, pin);
    gpio_pull_up(pin);

    // Generate the default PIO state machine config provided by pioasm
    pio_sm_config sm_conf = rx_dmx_program_get_default_config(offset);
    sm_config_set_in_pins(&sm_conf, pin); // for WAIT, IN
    sm_config_set_jmp_pin(&sm_conf, pin); // for JMP

    // Setup the side-set pins for the PIO state machine
    // Shift to right, autopush disabled
    sm_config_set_in_shift(&sm_conf, true, false, 32);
    // Deeper FIFO as we're not doing any TX
    sm_config_set_fifo_join(&sm_conf, PIO_FIFO_JOIN_RX);

    // Setup the clock divider to run the state machine at exactly 1MHz
    uint clk_div = clock_get_hz(clk_sys) / 1000000;
    sm_config_set_clkdiv(&sm_conf, clk_div);

    // Load our configuration, jump to the start of the program and run the State Machine
    pio_sm_init(pio0, 0, offset, &sm_conf);

    dma_chan_0_0 = dma_claim_unused_channel(true);


    // 8< BEGIN done. read_async 8<

    // Reset the PIO state machine to a consistent state. Clear the buffers and registers
    pio_sm_restart(pio0, 0);

    // Start the DMX PIO program from the beginning
    //pio_sm_exec(pio0, 0, pio_encode_jmp(offset));

    //setup dma
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan_0_0);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on DREQ_PIO0_RX0 (or whichever pio and sm we are using)
    channel_config_set_dreq(&cfg, DREQ_PIO0_RX0);

    #define DMXINPUT_BUFFER_SIZE(start_channel, num_channels) ((start_channel+num_channels+1)+((4-(start_channel+num_channels+1)%4)%4))
    dma_channel_configure(
        dma_chan_0_0, 
        &cfg,
        NULL,    // dst
        //dmxBuffer.buffer[8],    // dst
        &pio0->rxf[0],  // src
        DMXINPUT_BUFFER_SIZE(0, 512)/4,  // transfer count,
        false
    );

    dma_channel_set_irq0_enabled(dma_chan_0_0, true);

    // Call the IRQ handler once
    irq_handler_dma_chan_0_0();

    pio_sm_set_enabled(pio0, 0, true);
}

void LocalDmx::initTxDmx(uint pin_base, uint pin_count) {
    // Set up our TRIGGER GPIO init it to LOW
#ifdef PIN_TRIGGER
    gpio_init(PIN_TRIGGER);
    gpio_set_dir(PIN_TRIGGER, GPIO_OUT);
    gpio_put(PIN_TRIGGER, 0);
#endif // PIN_TRIGGER

    // Set up a PIO state machine to serialise our bits at 250000 bit/s
    uint offset = pio_add_program(pio1, &tx_dmx_program);
    float div = (float)clock_get_hz(clk_sys) / 250000;
    tx_dmx_program_init(pio1, 2, offset, pin_base, pin_count, div);

    // Configure a channel to write the wavetable to PIO0
    // SM0's TX FIFO, paced by the data request signal from that peripheral.
    this->dma_chan_1_2 = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(this->dma_chan_1_2);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true); // TODO: is by default. Line needed?
    channel_config_set_dreq(&c, DREQ_PIO1_TX2);

    dma_channel_configure(
        this->dma_chan_1_2,
        &c,
        &pio1_hw->txf[2], // Write address (only need to set this once)
        NULL,             // Don't provide a read address yet
        WAVETABLE_LENGTH/2, // Write one complete DMX packet, then halt and interrupt
                          // It's WAVETABLE_LENGTH/2 since we transfer 32 bit per transfer
        false             // Don't start yet
    );

    // Tell the DMA to raise IRQ line 0 when the channel finishes a block
    dma_channel_set_irq0_enabled(this->dma_chan_1_2, true);

    // Configure the processor to run the handler when DMA IRQ 0 is asserted
    irq_set_exclusive_handler(DMA_IRQ_0, irq_handler_dma_irq0_c);
    //irq_add_shared_handler(DMA_IRQ_0, dma_handler_0_0_c, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
    irq_set_enabled(DMA_IRQ_0, true);

    // Zero the wavetable so we don't output garbage on the first run
    memset(wavetable, 0x00, WAVETABLE_LENGTH * sizeof(uint16_t));

    // Manually call the handler once to trigger the first transfer
    dma_channel_set_read_addr(dma_chan_1_2, wavetable, true);
}

bool LocalDmx::setPort(uint8_t portId, uint8_t* source, uint16_t sourceLength) {
    // TODO: Check portId for validity (existing on local IO board), configured as an OUT, ...
    if ((portId >= LOCALDMX_COUNT) || (source == nullptr) || sourceLength == 0) {
        return false;
    }
    // Shall we lock the buffer so two sources don't write at the same time?
    // TODO: Don't change the buffer while the conversion to the wavetable is running

    uint16_t length = MIN(sourceLength, 512);

    critical_section_enter_blocking(&bufferLock);
    memset(this->buffer[portId], 0x00, 512);
    memcpy(this->buffer[portId], source, length);
    critical_section_exit(&bufferLock);

    return true;
}

// Appends one bit to the wavetable for port "port" at the position
// bitoffset. The offset will be increased by 1!
void LocalDmx::wavetable_write_bit(int port, uint16_t* bitoffset, uint8_t value) {
    if (!value) {
        // Since initial value is 0, just increment the offset
        (*bitoffset)++;
        return;
    }

    wavetable[(*bitoffset)++] |= (1 << port);
};

// Appends one byte (including on start and two stop bits) to the wavetable for
// given port at the given bit offset. This offset will be increased!
void LocalDmx::wavetable_write_byte(int port, uint16_t* bitoffset, uint8_t value) {
    // Start bit is 0
    this->wavetable_write_bit(port, bitoffset, 0);
    // I assume LSB is first? At least it works :)
    this->wavetable_write_bit(port, bitoffset, (value >> 0) & 0x01);
    this->wavetable_write_bit(port, bitoffset, (value >> 1) & 0x01);
    this->wavetable_write_bit(port, bitoffset, (value >> 2) & 0x01);
    this->wavetable_write_bit(port, bitoffset, (value >> 3) & 0x01);
    this->wavetable_write_bit(port, bitoffset, (value >> 4) & 0x01);
    this->wavetable_write_bit(port, bitoffset, (value >> 5) & 0x01);
    this->wavetable_write_bit(port, bitoffset, (value >> 6) & 0x01);
    this->wavetable_write_bit(port, bitoffset, (value >> 7) & 0x01);

    // Write two stop bits
    this->wavetable_write_bit(port, bitoffset, 1);
    this->wavetable_write_bit(port, bitoffset, 1);
};

void irq_handler_dma_irq0_c() {
    localDmx.irq_handler_dma_irq0();
}

void LocalDmx::irq_handler_dma_chan_0_0() {
    debugStruct.dma_0_0_counter++;
/*
            dma_channel_set_write_addr(dma_chan_0_0, dmxBuffer.buffer[8], true);
            pio_sm_exec(pio0, 0, pio_encode_jmp(prgm_offsets[pio_get_index(instance->_pio)]));
            pio_sm_clear_fifos(instance->_pio, instance->_sm);
#ifdef ARDUINO
            instance->_last_packet_timestamp = millis();
#else
            instance->_last_packet_timestamp = to_ms_since_boot(get_absolute_time());
*/
}

// One transfer has finished, prepare the next DMX packet and restart the
// DMA transfer
void LocalDmx::irq_handler_dma_chan_1_2() {
    debugStruct.dma_1_2_counter++;

    uint8_t universe;   // Loop over the 16 universes
    uint16_t bitoffset; // Current bit offset inside current universe
    uint16_t chan;      // Current channel in universe

#ifdef PIN_TRIGGER
    // Drive the TRIGGER GPIO to LOW
    gpio_put(PIN_TRIGGER, 0);
#endif // PIN_TRIGGER

    critical_section_enter_blocking(&bufferLock);

    // Zero the wavetable. *2 because of the data type: uint16_t = 2 byte per element
    memset(wavetable, 0x00, WAVETABLE_LENGTH * sizeof(uint16_t));

    // Loop through all 16 universes
    for (universe = 0; universe < 16; universe++) {
        // Usually, DMX needs a BREAK (LOW level) of at least 96µs before
        // MARK-AFTER-BREAK (MAB, HIGH LEVEL)
        // However, since the line is already at a defined LOW level
        // and we need CPU time to prepare the wavetable (~3ms), we don't
        // generate a BREAK. We start right away with the MAB
        bitoffset = 0;

        // Write 4 bit MARK-AFTER-BREAK (16µs)
        wavetable_write_bit(universe, &bitoffset, 1);
        wavetable_write_bit(universe, &bitoffset, 1);
        wavetable_write_bit(universe, &bitoffset, 1);
        wavetable_write_bit(universe, &bitoffset, 1);

        // Write the startbyte
        wavetable_write_byte(universe, &bitoffset, 0);

        // Write the data (channel values) from the universe's buffer
        for (chan = 0; chan < 512; chan++) {
            wavetable_write_byte(universe, &bitoffset, this->buffer[universe][chan]);
        }

        // Leave the line at a defined LOW level (BREAK) until the next packet starts
        wavetable_write_bit(universe, &bitoffset, 0);
    }

    critical_section_exit(&bufferLock);

    // Give the channel a new wavetable-entry to read from, and re-trigger it
    dma_channel_set_read_addr(dma_chan_1_2, wavetable, true);

#ifdef PIN_TRIGGER
    // Drive the TRIGGER GPIO to HIGH
    gpio_put(PIN_TRIGGER, 1);
#endif // PIN_TRIGGER

}

void LocalDmx::irq_handler_dma_irq0() {
    debugStruct.irq0_counter++;
    debugStruct.dma_inte0 = dma_hw->inte0;
    debugStruct.dma_ints0 = dma_hw->ints0;
    debugStruct.dma_inte1 = dma_hw->inte1;
    debugStruct.dma_ints1 = dma_hw->ints1;

    // Check the DMA channel that triggered the IRQ

    if ((dma_hw->ints0 & (1u<<dma_chan_0_0))) {
        irq_handler_dma_chan_0_0();
        dma_hw->ints0 |= (1u<<dma_chan_0_0);
    }

    if ((dma_hw->ints0 & (1u<<dma_chan_1_2))) {
        irq_handler_dma_chan_1_2();
        dma_hw->ints0 |= (1u<<dma_chan_1_2);
    }
    // TODO: Other DMA channels!


    // TODO: I'm not sure which approach is better:
    //       Clearing each IRQ flag right after its handler has been processed
    //       OR
    //       Clearing ALL IRQ flags after all handlers have been processed

    // TODO: What happens when the IRQ handler is busy while DMX bytes are coming in?

    // All possible sources for this IRQ have been handled, reset all sources
    dma_hw->ints0 = 0x0000ffff;
};
