#include <vector>
#include <string>
#include <sstream>
#include <iterator>

#include <string.h>

#include "RF24/RF24.h"

extern "C" {

void wirelessInit() {
    RF24 radio(7, 8); // using pin 7 for the CE pin, and pin 8 for the CSN pin

    if (!radio.begin()) {
    //Serial.println(F("radio hardware is not responding!!"));
    //while (1) {} // hold in infinite loop
    }

    // Set the PA Level low to try preventing power supply related problems
    // because these examples are likely run with nodes in close proximity to
    // each other.
    radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.
}

}