#ifndef ETH_W5500_H
#define ETH_W5500_H

#include "pins.h"

#include <lwip/netif.h>
#include <lwip/dhcp.h>

#include "../lib/W5500MacRaw/w5500.h"

#ifdef __cplusplus

class Eth_W5500 {
  public:
    void init();
    void getStatus();

    void service_traffic(void);

    uint16_t sendFrame(const uint8_t *data, uint16_t datalen);

    bool responding;       // True if the board resonded to the bus scan
    bool phyLink;
    bool phyLink100;
    bool phyLinkFD;

    uint8_t ethTxBuf[1800];  // At least one full frame at MTU = 1500

    volatile bool irqPending;

    struct dhcp* dhcp;
    struct netif netif;

  private:
    uint8_t mac[6];
    uint8_t ethRxBuf[1800];  // At least one full frame at MTU = 1500
    bool phyLinkPrevious;
    Wiznet5500 *w5500;
    bool initDone;
};

#endif // __cplusplus

// Helper getters for certain values that are required in C code
#ifdef __cplusplus
extern "C" {
#endif

// NONE yet

#ifdef __cplusplus
}
#endif

#endif // ETH_W5500_H
