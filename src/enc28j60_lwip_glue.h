#ifndef _ENC28J60_LWIP_GLUE_H_
#define _ENC28J60_LWIP_GLUE_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <hardware/gpio.h>

#include <hardware/spi.h>
#include <pico/critical_section.h>
#include <pico/stdio.h>
#include <pico/util/queue.h>

#include "lwipopts.h"
#include <lwip/igmp.h>
#include <lwip/init.h>
#include <lwip/netif.h>
#include <lwip/timeouts.h>
#include <lwip/apps/httpd.h>

#include <pico/enc28j60/enc28j60.h>
#include <pico/enc28j60/ethernetif.h>

#include "pins.h"
#include "boardconfig.h"

void enc28j60_irq(uint gpio, uint32_t events);
void enc28j60_init_highlevel();
void enc28j60_service_traffic();

#ifdef __cplusplus
 }
#endif

#endif // _ENC28J60_LWIP_GLUE_H_