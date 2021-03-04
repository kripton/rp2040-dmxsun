#include <vector>
#include <string>
#include <sstream>
#include <iterator>

#include <string.h>

static std::vector<std::string> logBuffer;

extern "C" {
#include "log.h"

void dlog(char* file, uint32_t line, char* text) {

    std::string fname = std::string(file);
    auto const pos = fname.find_last_of('/');
    fname = fname.substr(pos + 1);

    logBuffer.push_back(
        std::string("{type: \"log\", file: \"" + fname +
        std::string("\", line: ") + std::to_string(line) +
        std::string(", text: \"") + std::string(text) + std::string("\"}")
    ));
}

uint32_t getLogBuffer(char* buf, uint32_t sizeOfBuf) {
    const char* const delim = "\n";

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

void clearLugBuffer() {
    logBuffer.clear();
}

}
