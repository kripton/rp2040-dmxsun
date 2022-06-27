#include "enc28j60_lwip_glue.h"
#include <pico/unique_id.h>

#include "log.h"

/* Configuration */
#define RX_QUEUE_SIZE 16
#define MAC_ADDRESS { 0x62, 0x5E, 0x22, 0x07, 0xDE, 0x92 }
#define IP_ADDRESS IPADDR4_INIT_BYTES(192, 168, 1, 200)
#define NETWORK_MASK IPADDR4_INIT_BYTES(255, 255, 255, 0)
#define GATEWAY_ADDRESS IPADDR4_INIT_BYTES(192, 168, 1, 1)

queue_t rx_queue;
critical_section_t spi_cs;
struct netif netif;
struct enc28j60 enc28j60 = {
	.spi = spi0,
	.cs_pin = PIN_SPI_CS1,
	.mac_address = MAC_ADDRESS,
	.next_packet = 0,
	.critical_section = &spi_cs,
};

void enc28j60_irq(uint gpio, uint32_t events)
{
	enc28j60_isr_begin(&enc28j60);
	uint8_t flags = enc28j60_interrupt_flags(&enc28j60);

	if (flags & ENC28J60_PKTIF) {
		struct pbuf *packet = low_level_input(&netif);
		if (packet != NULL) {
			if (!queue_try_add(&rx_queue, &packet)) {
				pbuf_free(packet);
			}
		}
	}

	if (flags & ENC28J60_TXERIF) {
		LWIP_DEBUGF(NETIF_DEBUG, ("eth_irq: transmit error\n"));
	}

	if (flags & ENC28J60_RXERIF) {
		LWIP_DEBUGF(NETIF_DEBUG, ("eth_irq: receive error\n"));
	}

	enc28j60_interrupt_clear(&enc28j60, flags);
	enc28j60_isr_end(&enc28j60);
}

void enc28j60_init_highlevel()
{
	gpio_init_mask((1 << PIN_SPI_CS1));
	gpio_set_dir_out_masked((1 << PIN_SPI_CS1));

	queue_init(&rx_queue, sizeof(struct pbuf *), RX_QUEUE_SIZE);
	critical_section_init(&spi_cs);

	gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
	gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);
	gpio_set_function(PICO_DEFAULT_SPI_RX_PIN, GPIO_FUNC_SPI);

	const struct ip4_addr ipaddr = IP_ADDRESS;
	const struct ip4_addr netmask = NETWORK_MASK;
	const struct ip4_addr gw = GATEWAY_ADDRESS;
	// lwip_init(); // Already happened for the USB network interface
	netif_add(&netif, &ipaddr, &netmask, &gw, &enc28j60, ethernetif_init, netif_input);
	netif_set_up(&netif);
	netif_set_link_up(&netif);

	gpio_init(PIN_ENC_IRQ);
	gpio_pull_up(PIN_ENC_IRQ); // pull it up even more!
	gpio_set_irq_enabled_with_callback(PIN_ENC_IRQ, GPIO_IRQ_EDGE_FALL, true, enc28j60_irq);
	enc28j60_interrupts(&enc28j60, ENC28J60_PKTIE | ENC28J60_TXERIE | ENC28J60_RXERIE);
}

void enc28j60_service_traffic(void)
{
	struct pbuf* p = NULL;
	int i = 0;
	while (i <= 5) {
    queue_try_remove(&rx_queue, &p);
	if (p != NULL) {
	    if(netif.input(p, &netif) != ERR_OK) {
			LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
			pbuf_free(p);
		}
		i++;
	} else {
		break;
	}
	}

	//sys_check_timeouts();
	//best_effort_wfe_or_timeout(make_timeout_time_ms(sys_timeouts_sleeptime()));
}
