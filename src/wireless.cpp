extern "C" {
#include "log.h"
#include "wireless.h"
}

#include "RF24/RF24.h"

extern "C" {

extern uint8_t dmx_values[16][512];           // 16 universes with 512 byte each
RF24 radio(28, 5); // using pin 28 for the CE pin, and pin 5 for the CSN pin

void wirelessInit() {
    uint8_t address[][6] = {"1Node", "2Node"};

    // initialize the transceiver on the SPI bus
    bool result = radio.begin();
    while (!result) {
        printf("Radio hardware is not responding!\n");
        sleep_ms(5000);
        result = radio.begin();
    }

    printf("Result of radio.begin: %d\n", result);

    // Set the PA Level low to try preventing power supply related problems
    // because these examples are likely run with nodes in close proximity to
    // each other.
    radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

    radio.setPayloadSize(16);

    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(address[0]);     // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[1]); // using pipe 1

    radio.stopListening();  // put radio in TX mode
}

void wirelessSend() {
    radio.write(dmx_values[0], 16);
}

}