#include "pico/stdlib.h"

#define LOG(text) dlog(__FILE__, __LINE__, text)

void dlog(char* file, uint32_t line, char* text);
uint32_t getLogBuffer(char* buf, uint32_t sizeOfBuf);
void clearLugBuffer();