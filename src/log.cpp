#include <vector>
#include <string>
#include <sstream>
#include <iterator>

#include <string.h>

static std::vector<std::string> logBuffer;

extern "C" {
#include "log.h"

void logs(const char* file, uint32_t line, const char* text) {
    std::string fname = std::string(file);
    auto const pos = fname.find_last_of('/');
    fname = fname.substr(pos + 1);

    logBuffer.push_back(
        std::string("{type: \"log\", file: \"" + fname +
        std::string("\", line: ") + std::to_string(line) +
        std::string(", text: \"") + std::string(text) + std::string("\"}")
    ));
}

void logfmt(const char* file, uint32_t line, const char* text, ...) {
    std::string fname = std::string(file);
    auto const pos = fname.find_last_of('/');
    fname = fname.substr(pos + 1);

    // Get the length of the complete string
    va_list optargs;
    va_start(optargs, text);
    int size_s = vsnprintf(nullptr, 0, text, optargs) + 1;
    va_end(optargs);

    // Limit the string to at most 1024 byte
    if (size_s > 1024) {
        size_s = 1024;
    }

    // Print the string into the newly allocated buffer
    char* tmpBuffer = (char*)malloc(size_s);
    va_start(optargs, text);
    vsnprintf(tmpBuffer, size_s, text, optargs);
    va_end(optargs);

    // Assemble all of it together
    logBuffer.push_back(
        std::string("{type: \"log\", file: \"" + fname +
        std::string("\", line: ") + std::to_string(line) +
        std::string(", text: \"") + std::string(tmpBuffer) + std::string("\"}")
    ));
}

void loguni(const char* file, uint32_t line, uint8_t id, const uint8_t* universe) {
    char dataText[(512*3) + 1];

    std::string fname = std::string(file);
    auto const pos = fname.find_last_of('/');
    fname = fname.substr(pos + 1);

    for (uint16_t chan = 0; chan < 512; chan++) {
        snprintf(dataText + (chan*3), 4, "%02x ", universe[chan]);
    }
    logBuffer.push_back(
        std::string("{type: \"universe\", file: \"" + fname +
        std::string("\", line: ") + std::to_string(line) +
        std::string(", id: ") + std::to_string(id) +
        std::string(", data: \"") + std::string(dataText) + std::string("\"}")
    ));
}

uint32_t getLogBuffer(char* buf, uint32_t sizeOfBuf) {
    std::string output;
    for (const auto& value: logBuffer) {
        output += value + "\n";
    }

    uint32_t size = output.length() + 1;
    if (sizeOfBuf < size) {
        size = sizeOfBuf - 2;
    }

    if (buf) {
        memset(buf, 0, sizeOfBuf);
        memcpy(buf, output.c_str(), size);
    }

    return size;
}

void printLogBuffer() {
    for (const auto& value: logBuffer) {
        printf("%s\n", value.c_str());
    }
}

void clearLogBuffer() {
    logBuffer.clear();
}

}
