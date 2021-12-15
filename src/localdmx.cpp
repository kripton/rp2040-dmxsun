#include "log.h"
#include "localdmx.h"
#include "statusleds.h"

// For DMX input:
#include "dmxbuffer.h"
extern DmxBuffer dmxBuffer;

#include <string.h>

#include <hardware/clocks.h>    // To derive our 250000bit/s from sys_clk
#include <hardware/dma.h>       // To control the data transfer from mem to pio
#include <hardware/gpio.h>      // To "manually" control the trigger pin
#include <hardware/irq.h>       // To control the data transfer from mem to pio

#include "debug_struct.h"
extern struct DebugStruct debugStruct;

#include "tx_dmx.pio.h"           // Header file for the PIO program

extern LocalDmx localDmx;
extern StatusLeds statusLeds;

extern mutex_t bufferLock;

uint8_t LocalDmx::buffer[LOCALDMX_COUNT][513];
uint16_t LocalDmx::wavetable[WAVETABLE_LENGTH];  // 16 universes (data type) with 5648 bit each
uint8_t LocalDmx::inBuffer[8][513];

// DMX OUT-ONLY (max 16 universes) are on PIO 1, SM 2 and use   IRQ 1
// STATUS LEDs (ws2812) are on PIO 1, SM 3

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

    initMultiUniverseOutput();

    // Init local DMX input for testing
    memset(inBuffer, 0x00, 8*513);

    DmxInput::return_code retVal = dmxInput_0.begin(14, 0, 512, pio0);
    LOG("DmxInput.begin port 14 returned %u", retVal);
    dmxInput_0.read_async(inBuffer[0], input_updated_c);

    retVal = dmxInput_1.begin(15, 0, 512, pio0);
    LOG("DmxInput.begin port 15 returned %u", retVal);
    dmxInput_1.read_async(inBuffer[1], input_updated_c);
}

void LocalDmx::initMultiUniverseOutput() {
    // Set up our TRIGGER GPIO init it to LOW
#ifdef PIN_TRIGGER
    gpio_init(PIN_TRIGGER);
    gpio_set_dir(PIN_TRIGGER, GPIO_OUT);
    gpio_put(PIN_TRIGGER, 0);
#endif // PIN_TRIGGER

    // Set up a PIO state machine to serialise our bits at 250000 bit/s
    uint offset = pio_add_program(pio1, &tx_dmx_program);
    float div = (float)clock_get_hz(clk_sys) / 250000;
    // TODO: /!\ Temporarily reduced to 8 universes so we an play around with GPIOs 14-21 (DMX IN ;) /!\ //
    tx_dmx_program_init(pio1, 2, offset, 6, 8, div); // TODO: Make pin base and count depending on board config

    // Configure a channel to write the wavetable to PIO0
    // SM0's TX FIFO, paced by the data request signal from that peripheral.
    this->dma_chan_MultiUniOut = dma_claim_unused_channel(true);
    debugStruct.localDmx_dmaChan = this->dma_chan_MultiUniOut;
    LOG("LocalDmx is using DMA channel %u", this->dma_chan_MultiUniOut);
    dma_channel_config c = dma_channel_get_default_config(this->dma_chan_MultiUniOut);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true); // TODO: is by default. Line needed?
    channel_config_set_dreq(&c, DREQ_PIO1_TX2);

    dma_channel_configure(
        this->dma_chan_MultiUniOut,
        &c,
        &pio1_hw->txf[2], // Write address (only need to set this once)
        NULL,             // Don't provide a read address yet
        WAVETABLE_LENGTH/2, // Write one complete DMX packet, then halt and interrupt
                          // It's WAVETABLE_LENGTH/2 since we transfer 32 bit per transfer
        false             // Don't start yet
    );

    // Tell the DMA to raise IRQ line 1 when the channel finishes a block
    dma_channel_set_irq1_enabled(this->dma_chan_MultiUniOut, true);

    // Configure the processor to run dma_handler() when DMA IRQ 1 is asserted
    //irq_set_exclusive_handler(DMA_IRQ_1, irq_handler_MultiUniOut_c);
    irq_set_exclusive_handler(DMA_IRQ_1, irq_handler_MultiUniOut_c);
    irq_set_enabled(DMA_IRQ_1, true);

    // Zero the wavetable so we don't output garbage
    memset(buffer, 0x00, LOCALDMX_COUNT*513);
    memset(wavetable, 0x00, WAVETABLE_LENGTH * sizeof(uint16_t));

    // Manually call the handler once, to trigger the first transfer
    dma_channel_set_read_addr(dma_chan_MultiUniOut, wavetable, true);
}

bool LocalDmx::setPort(uint8_t portId, uint8_t* source, uint16_t sourceLength) {
    // TODO: Check portId for validity (existing on local IO board), configured as an OUT, ...
    if ((portId >= LOCALDMX_COUNT) || (source == nullptr) || sourceLength == 0) {
        return false;
    }
    // Shall we lock the buffer so two sources don't write at the same time?
    // TODO: Don't change the buffer while the conversion to the wavetable is running

    uint16_t length = MIN(sourceLength, 512);

    mutex_enter_blocking(&bufferLock);
    memset(this->buffer[portId]+1, 0x00, 512);
    memcpy(this->buffer[portId]+1, source, length);
    mutex_exit(&bufferLock);

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

void irq_handler_MultiUniOut_c() {
    localDmx.irq_handler_MultiUniOut();
}

void input_updated_c(DmxInput* instance) {
    localDmx.input_updated(instance);
}

void __isr LocalDmx::input_updated(DmxInput* instance) {
    // Inside here: Don't log, don't sleep*
    //LOG("Input on pin %u changed!", instance->pin());
    statusLeds.setBlinkOnce(7, 0, 1, 0);
    if (instance->pin() == 14) {
        dmxBuffer.setBuffer(0, inBuffer[0] + 1, 512); // TODO: Proper patch handling!
    } else if (instance->pin() == 15) {
        dmxBuffer.setBuffer(1, inBuffer[1] + 1, 512); // TODO: Proper patch handling!
    }
}

// One transfer has finished, prepare the next DMX packet and restart the
// DMA transfer
void __isr LocalDmx::irq_handler_MultiUniOut() {
    uint8_t universe;   // Loop over the 16 universes
    uint16_t bitoffset; // Current bit offset inside current universe
    uint16_t chan;      // Current channel in universe

    // Check if it was our DMA chan that triggered the IRQ
    if (!(dma_hw->ints1 & (1u<<dma_chan_MultiUniOut))) {
        // If not, do nothing
        return;
    }

#ifdef PIN_TRIGGER
    // Drive the TRIGGER GPIO to LOW
    gpio_put(PIN_TRIGGER, 0);
#endif // PIN_TRIGGER

    mutex_enter_blocking(&bufferLock);

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
        wavetable_write_byte(universe, &bitoffset, this->buffer[universe][0]);

        // Write the data (channel values) from the universe's buffer
        for (chan = 1; chan < 513; chan++) {
            wavetable_write_byte(universe, &bitoffset, this->buffer[universe][chan]);
        }

        // Leave the line at a defined LOW level (BREAK) until the next packet starts
        wavetable_write_bit(universe, &bitoffset, 0);
    }

    mutex_exit(&bufferLock);

    // Clear the interrupt request.
    dma_hw->ints1 = 1u << dma_chan_MultiUniOut;

    // Give the channel a new wavetable-entry to read from, and re-trigger it
    dma_channel_set_read_addr(dma_chan_MultiUniOut, wavetable, true);

#ifdef PIN_TRIGGER
    // Drive the TRIGGER GPIO to HIGH
    gpio_put(PIN_TRIGGER, 1);
#endif // PIN_TRIGGER
};
