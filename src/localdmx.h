#ifndef LOCALDMX_H
#define LOCALDMX_H

#include <stdio.h>

#include "pins.h"

#include "DmxOutput.h"
#include "DmxInput.h"

#ifndef LOCALDMX_COUNT
#define LOCALDMX_COUNT 16
#endif // LOCALDMX_COUNT

#define WAVETABLE_LENGTH 5648   // bits per DMX packet. Wavetable has 16*this bits in total

#ifdef __cplusplus

struct PortStatus {
  bool available;            // If this port is physically available
  bool isOnMultiOut;         // If this port is controlled by the multi-universe-out-only state machine
  DmxOutput* outputInstance; // The instance of DmxOutput for that port, if any
  DmxInput* inputInstance;   // The instance of DmxInput for that port, if any
  uint16_t dataPin;          // GPIO pin used to write or read DMX data
  uint16_t dirPin;           // GPIO pin used to switch RX / TX
};

// Class that stores and manages ALL local DMX ports
class LocalDmx {
  public:
    static uint8_t buffer[LOCALDMX_COUNT][513];
    static uint8_t inBuffer[8][513];
    bool setPort(uint8_t portId, uint8_t* source, uint16_t sourceLength); // alias "copyFrom"

    void init();
    void initMultiUniverseOutput();

    // The IRQ handler called if MultiUniverseOutput needs data
    void __isr irq_handler_MultiUniOut();

    // The IRQ handler called if there is new DMX data on an input
    void __isr input_updated(DmxInput* instance);

  private:
    struct PortStatus portStati[16];

    // TODO: Do we need more than one for multiple SMs? Or could we use
    //       ONE DMA channel for multiple SMs?
    // TODO: The RP2040 has 12 DMA channels. Are 7 available or already
    //       claimed by s.th. else?
    int dma_chan_MultiUniOut;                  // The DMA channel for PIO 1, SM2
    // PIO 1, SM3 is used for the Status LEDs

    // TODO: This assumes 16 OUTs
    static uint16_t wavetable[WAVETABLE_LENGTH];  // 16 universes (data type) with 5648 bit each

    // Helper functions for DMX output generation
    // TODO: Check if those work for RDM ports (or fewer universes than 16)
    void wavetable_write_bit(int port, uint16_t* bitoffset, uint8_t value);
    void wavetable_write_byte(int port, uint16_t* bitoffset, uint8_t value);

    DmxInput dmxInput_0;
    DmxInput dmxInput_1;
    DmxInput dmxInput_2;
    DmxInput dmxInput_3;
    DmxInput dmxInput_5;
    DmxInput dmxInput_6;

    DmxOutput dmxOutput_0;
    DmxOutput dmxOutput_1;
    DmxOutput dmxOutput_2;
    DmxOutput dmxOutput_3;
    DmxOutput dmxOutput_4;
    DmxOutput dmxOutput_5;

    bool incomingQueueValid[6];
    uint8_t incomingQueueData[6][513];
};

#endif // __cplusplus

// Helper methods which are called from C code
#ifdef __cplusplus
extern "C" {
#endif

    // The IRQ handler called if MultiUniverseOutput needs data
    void __isr irq_handler_MultiUniOut_c();

    // The IRQ handler called if there is new DMX data on an input
    void __isr input_updated_c(DmxInput* instance);

#ifdef __cplusplus
}
#endif

#endif // DMXBUFFER_H
