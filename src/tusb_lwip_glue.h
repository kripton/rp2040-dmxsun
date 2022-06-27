#ifndef _TUSB_LWIP_GLUE_H_
#define _TUSB_LWIP_GLUE_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <tusb.h>
#include <dhserver.h>
#include <dnserver.h>
#include <lwip/igmp.h>
#include <lwip/init.h>
#include <lwip/timeouts.h>
#include <lwip/apps/httpd.h>

#include <pico/util/queue.h>

#include "boardconfig.h"

#define TUSB_RX_QUEUE_SIZE 16

void init_lwip();
void wait_for_netif_is_up();
void dhcpd_init();
void service_traffic();


#ifdef __cplusplus
 }
#endif

#endif // _TUSB_LWIP_GLUE_H_