#include "pico/stdlib.h"
#include <stdarg.h>

// LOG = Simple log. One static (or prepared) text.
#define LOG(text) logs(__FILE__, __LINE__, text)

// LOGfmt = Log formatted. Like printf() but limited to 1024 byte. More work at runtime.
#define LOGfmt(text, ...) logfmt(__FILE__, __LINE__, text __VA_OPT__(,) __VA_ARGS__)

// LOGuni = Dump the content of a universe. Takes a 512 byte uint8_t*
#define LOGuni(id, universe) loguni(__FILE__, __LINE__, id, universe)

void logs(const char* file, uint32_t line, const char* text);
void logfmt(const char* file, uint32_t line, const char* text, ...);
void loguni(const char* file, uint32_t line, uint8_t id, const uint8_t* universe);
uint32_t getLogBuffer(char* buf, uint32_t sizeOfBuf);
void printLogBuffer();
void clearLogBuffer();
