extern "C" {
#include "log.h"
#include "wireless.h"
}

#include <RF24.h>
#include <RF24Network.h>
#include <RF24Mesh.h>

extern "C" {

extern uint8_t dmx_values[16][512];           // 16 universes with 512 byte each
RF24 radio(28, 5); // using pin 28 for the CE pin, and pin 5 for the CSN pin
RF24Network network(radio);
RF24Mesh mesh(radio, network);

void wirelessInit() {
    uint8_t address[][6] = {"1Node", "2Node"};

    // initialize the transceiver on the SPI bus
    SPI spi;
    spi.begin(spi0, 2, 3, 4);
    bool result = radio.begin(&spi);
    while (!result) {
        printf("Radio hardware is not responding!\n");
        sleep_ms(5000);
        result = radio.begin(&spi);
    }

    printf("Result of radio.begin: %d\n", result);

    // Set the PA Level low to try preventing power supply related problems
    // because these examples are likely run with nodes in close proximity to
    // each other.
    radio.setPALevel(RF24_PA_LOW);  // RF24_PA_MAX is default.

    radio.setChannel(120);
    radio.setDataRate(RF24_1MBPS);

    radio.setPayloadSize(24);

    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(address[0]);     // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1, address[1]); // using pipe 1

    radio.stopListening();  // put radio in RX mode
}

void wirelessSend() {
    uint8_t incoming[32];
    if (radio.available()) {
        memset(incoming, 0, 32);
        radio.read(incoming, 32);
        printf("There was data available via RF24: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
          incoming[0],
          incoming[1],
          incoming[2],
          incoming[3],
          incoming[4],
          incoming[5],
          incoming[6],
          incoming[7],
          incoming[8],
          incoming[9]
        );
    }
    radio.stopListening(); // TX mode
    bool result = radio.write(dmx_values[0] + 43, 24);
    printf("WRITE RESULT: %d", result);
    radio.startListening();  // RX mode again
}

}