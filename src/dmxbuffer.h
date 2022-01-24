#ifndef DMXBUFFER_H
#define DMXBUFFER_H

#include <stdio.h>

#ifndef DMXBUFFER_COUNT
#define DMXBUFFER_COUNT 24
#endif // DMXBUFFER_COUNT

#ifdef __cplusplus

/**
 * The DmxBuffer class stores and manages ALL internal "main" DMX buffers.
 * Those internal buffers are the ones, that all DMX data needs to pass
 * through. Incoming data is written here using the setBuffer or setChannel
 * methods. These methods also check if the respictive buffer is patched to
 * an output and will copy the data there.
 */
class DmxBuffer {
  public:
    static uint8_t buffer[DMXBUFFER_COUNT][512];
    static uint8_t allZeroes[512]; //!< Array of 512 zero-bytes to be used with memcmp for performance
    void init();
    void zero(uint8_t bufferId);
    bool getBuffer(uint8_t bufferId, uint8_t* dest, uint16_t destLength); // alias "copyTo"
    bool setBuffer(uint8_t bufferId, uint8_t* source, uint16_t sourceLength); // alias "copyFrom"
    bool getChannel(uint8_t bufferId, uint16_t channel, uint8_t* value);
    bool setChannel(uint8_t bufferId, uint16_t channel, uint8_t value);

    bool isAllZero(uint8_t bufferId);

  private:
    void triggerPatchings(uint8_t bufferId, bool allZero = false);
    bool allZeroBuffers[DMXBUFFER_COUNT];
};

#endif // __cplusplus

// Helper methods which are called from C code
#ifdef __cplusplus
extern "C" {
#endif

// None yet :)

#ifdef __cplusplus
}
#endif

#endif // DMXBUFFER_H
