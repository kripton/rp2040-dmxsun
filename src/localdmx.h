#ifndef LOCALDMX_H
#define LOCALDMX_H

#include <stdio.h>

#include "pins.h"

#ifndef LOCALDMX_COUNT
#define LOCALDMX_COUNT 16
#endif // LOCALDMX_COUNT

#define WAVETABLE_LENGTH 5648   // bits per DMX packet. Wavetable has 16*this bits in total

#ifdef __cplusplus

// Class that stores and manages ALL local DMX ports
class LocalDmx {
  public:
    static uint8_t buffer[LOCALDMX_COUNT][512];
    bool setPort(uint8_t portId, uint8_t* source, uint16_t sourceLength); // alias "copyFrom"
    void init();

    // 2 IRQ handlers, one for each DMA_IRQ line. Currently, we use DMA_IRQ0 on core 0
    void irq_handler_dma_irq0(); // The DMA handler to call if DMA_IRQ0 fired
    void irq_handler_dma_irq1(); // The DMA handler to call if DMA_IRQ1 fired

    void irq_handler_dma_chan_0_0();
    void irq_handler_dma_chan_0_1();
    void irq_handler_dma_chan_0_2();
    void irq_handler_dma_chan_0_3();
    void irq_handler_dma_chan_1_0();
    void irq_handler_dma_chan_1_1();
    void irq_handler_dma_chan_1_2();

  private:
    void initRxDmx(uint pin);
    void initTxDmx(uint pin_base, uint pin_count); // "Fallback": 16 output universes

    // TODO: Do we need more than one for multiple SMs? Or could we use
    //       ONE DMA channel for multiple SMs?
    // TODO: The RP2040 has 12 DMA channels. Are 7 available or already
    //       claimed by s.th. else?
    int dma_chan_0_0;                  // The DMA channel for PIO 0, SM0
    int dma_chan_0_1;                  // The DMA channel for PIO 0, SM1
    int dma_chan_0_2;                  // The DMA channel for PIO 0, SM2
    int dma_chan_0_3;                  // The DMA channel for PIO 0, SM3
    int dma_chan_1_0;                  // The DMA channel for PIO 1, SM0
    int dma_chan_1_1;                  // The DMA channel for PIO 1, SM1
    int dma_chan_1_2;                  // The DMA channel for PIO 1, SM2
    // PIO 1, SM3 is used for the Status LEDs

    // TODO: This assumes 16 OUTs
    static uint16_t wavetable[WAVETABLE_LENGTH];  // 16 universes (data type) with 5648 bit each

    // Helper functions for DMX output generation
    // TODO: Check if those work for RDM ports (or fewer universes than 16)
    void wavetable_write_bit(int port, uint16_t* bitoffset, uint8_t value);
    void wavetable_write_byte(int port, uint16_t* bitoffset, uint8_t value);
};

#endif // __cplusplus

// Helper methods which are called from C code
#ifdef __cplusplus
extern "C" {
#endif

    void irq_handler_dma_irq0_c(); // The IRQ handler to call if DMA_IRQ0 fired
    void irq_handler_dma_irq1_c(); // The IRQ handler to call if DMA_IRQ1 fired

#ifdef __cplusplus
}
#endif

#endif // DMXBUFFER_H
