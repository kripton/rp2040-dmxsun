#include "statusleds.h"
#include "boardconfig.h"
#include "log.h"

#include "eth_w5500.h"

#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <pico/unique_id.h>
#include <pico/critical_section.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/util/queue.h>

#include "lwipopts.h"
#include <lwip/igmp.h>
#include <lwip/init.h>
#include <lwip/netif.h>
#include <lwip/timeouts.h>
#include <lwip/apps/httpd.h>

#include <lwip/def.h>
#include <lwip/etharp.h>
#include <lwip/ethip6.h>
#include <lwip/mem.h>
#include <lwip/opt.h>
#include <lwip/pbuf.h>
#include <lwip/snmp.h>
#include <lwip/stats.h>

extern StatusLeds statusLeds;
extern BoardConfig boardConfig;
extern Eth_W5500 eth_w5500;

err_t w5500_ethernetif_init(struct netif *netif);

struct w5500_info
{
    spi_inst_t *spi;
    uint8_t cs_pin;
    absolute_time_t lastCheck;
};

struct w5500_info w5500_info = {
    .spi = spi1,
    .cs_pin = PIN_SPI_CS_W,
    .lastCheck = 0,
};

void gpio_callback_w5500(void) {
    if (gpio_get_irq_event_mask(PIN_IRQ_W5500) & GPIO_IRQ_EDGE_FALL) {
        gpio_acknowledge_irq(PIN_IRQ_W5500, GPIO_IRQ_EDGE_FALL);
        //LOG("W5500 GPIO CALLBACK!");
        eth_w5500.irqPending = true;
    }
}

void Eth_W5500::init()
{
    responding = false;
    phyLink = false;
    phyLink100 = false;
    phyLinkFD = false;
    phyLinkPrevious = false;
    initDone = false;
    irqPending = false;

    //// Set up the IRQ line interrupt handlers
    gpio_init(PIN_IRQ_W5500);
    gpio_set_dir(PIN_IRQ_W5500, GPIO_IN);
    gpio_set_irq_enabled(PIN_IRQ_W5500, GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
    gpio_add_raw_irq_handler(PIN_IRQ_W5500, gpio_callback_w5500);
    // Use SPI1 at 20MHz (maximum clock rate the nRF24 on the same port supports)
    spi_init(spi1, 20 * 1000 * 1000);
    gpio_set_function(PIN_SPI_CLK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);
    spi_set_format(spi1, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Chip selects are active-low, so we initialise them to a driven-high state
    gpio_init(PIN_SPI_CS_W);
    gpio_set_dir(PIN_SPI_CS_W, GPIO_OUT);
    gpio_put(PIN_SPI_CS_W, 1);

    // Set up the W5500 with a testing mac (OpenMoko test range)
    mac[0] = 0x00;
    mac[1] = 0x1f;
    mac[2] = 0x11;
    mac[3] = 0x00;
    mac[4] = 0xDE;
    mac[5] = 0xAD;
    w5500 = new Wiznet5500(PIN_SPI_CS_W);
    LOG("Initializing with mac %02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    uint8_t wizOk = w5500->begin(mac);
    LOG("WIZOK: %d", wizOk);

    struct ip4_addr ipaddr = IPADDR4_INIT(boardConfig.activeConfig->eth_ownIp);
    struct ip4_addr netmask = IPADDR4_INIT(boardConfig.activeConfig->eth_mask);
    struct ip4_addr gw = IPADDR4_INIT(boardConfig.activeConfig->eth_gw);

    // If DHCP is configured to NOT use a fallback-IP, just use 0
    if (
        (boardConfig.activeConfig->eth_dhcpMode == EthDhcpMode::dhcpOrFail)
    ) {
        ipaddr = IPADDR4_INIT(0);
        netmask = IPADDR4_INIT(0);
        gw = IPADDR4_INIT(0);
    }

    memcpy(netif.hwaddr, mac, 6);

    netif_add(&netif, &ipaddr, &netmask, &gw, &w5500_info, w5500_ethernetif_init, netif_input);

    if (wizOk)
    {
        responding = true;
    }

    if (wizOk == 2)
    {
        netif_set_up(&netif);
        netif_set_link_up(&netif);

        if (
            (boardConfig.activeConfig->eth_dhcpMode == EthDhcpMode::dhcpOrFail) ||
            (boardConfig.activeConfig->eth_dhcpMode == EthDhcpMode::dhcpWithFallback)
        ) {
            LOG("Starting DHCP client");
            dhcp_start(&netif);
            dhcp = netif_dhcp_data(&netif);
        }
    }

    w5500_info.lastCheck = 0;
    initDone = true;
}

void Eth_W5500::getStatus()
{
    w5500->readStatus();
    phyLink = responding && w5500->phyLink;
    phyLink100 = phyLink && w5500->phyLink100;
    phyLinkFD = phyLink && w5500->phyLinkFD;

    if (phyLink && !phyLinkPrevious)
    {
        // We just got a link. Re-init the W5500
        // so that the MAC_RAW socket is opened properly
        LOG("Link UP detected, re-init the W5500!");
        w5500->end();
        uint8_t wizOk = w5500->begin(mac);
        LOG("WIZOK: %d", wizOk);

        if (wizOk == 2)
        {
            netif_set_up(&netif);
            netif_set_link_up(&netif);

            if (
               (boardConfig.activeConfig->eth_dhcpMode == EthDhcpMode::dhcpOrFail) ||
                (boardConfig.activeConfig->eth_dhcpMode == EthDhcpMode::dhcpWithFallback)
            ) {
                LOG("Starting DHCP client");
                dhcp_start(&netif);
                dhcp = netif_dhcp_data(&netif);
            }
        }
    }

    if (!phyLink && phyLinkPrevious)
    {
        LOG("Link DOWN detected (link lost!)");
        netif_set_down(&netif);
    }

    phyLinkPrevious = phyLink;
}

void Eth_W5500::service_traffic(void)
{
    struct pbuf *p, *q;
    uint16_t size, offset;
    absolute_time_t timeNow;


    if (!initDone || !responding) {
        return;
    }

    getStatus();

    timeNow = time_us_64();

    // Check if the last check for an incoming packet was more than 100ms ago
    // Happens if just no packets are coming in OR we somehow miss the
    // "falling edge" interrupt. In that case, we would STALL :-O
    if (timeNow > (w5500_info.lastCheck + (100 * 1000))) {
        //LOG("Simulating incoming IRQ. LastCheck: %llu TimeNow: %llu", w5500_info.lastCheck, timeNow);
        irqPending = true;
    }

    // Only continue if we got an IRQ
    // OR the lastCheck was more than 100ms ago (= 10 times per sec)
    if (!irqPending) {
        return;
    }

    //LOG("Setting lastCheck to %llu", timeNow);
    w5500_info.lastCheck = timeNow;
    //LOG("lastCheck is now %llu", w5500_info.lastCheck);

    // Reset the IRQ
    irqPending = false;
    w5500->setSn_IR(0xFF);

    int i = 0;
    while (i <= 3)
    {
        size = w5500->readFrame(ethRxBuf, 2000);
        if (size)
        {
            LOG("W5500: Read packet with size %d", size);

#if ETH_PAD_SIZE
            size += ETH_PAD_SIZE; /* allow room for Ethernet padding */
#endif

            /* We allocate a pbuf chain of pbufs from the pool. */
            p = pbuf_alloc(PBUF_RAW, size, PBUF_POOL);

            if (p != NULL)
            {

#if ETH_PAD_SIZE
                pbuf_remove_header(p, ETH_PAD_SIZE); /* drop the padding word */
#endif

                /* We iterate over the pbuf chain until we have read the entire
                 * packet into the pbuf. */
                offset = 0;
                for (q = p; q != NULL; q = q->next)
                {
                    /* Read enough bytes to fill this pbuf in the chain. The
                     * available data in the pbuf is given by the q->len
                     * variable.
                     * This does not necessarily have to be a memcpy, you can also preallocate
                     * pbufs for a DMA-enabled MAC and after receiving truncate it to the
                     * actually received size. In this case, ensure the tot_len member of the
                     * pbuf is the sum of the chained pbuf len members.
                     */
                    //LOG("Copying %d byte from offset %d to qbuf", q->len, offset);
                    memcpy(q->payload, ethRxBuf + offset, q->len);
                    offset += q->len;
                }

                MIB2_STATS_NETIF_ADD(netif, ifinoctets, p->tot_len);
                if (((u8_t *)p->payload)[0] & 1)
                {
                    /* broadcast or multicast packet*/
                    MIB2_STATS_NETIF_INC(netif, ifinnucastpkts);
                }
                else
                {
                    /* unicast packet*/
                    MIB2_STATS_NETIF_INC(netif, ifinucastpkts);
                }
#if ETH_PAD_SIZE
                pbuf_add_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

                LINK_STATS_INC(link.recv);
            }
            else
            {
                LINK_STATS_INC(link.memerr);
                LINK_STATS_INC(link.drop);
                MIB2_STATS_NETIF_INC(netif, ifindiscards);
            }

            // pbuf p has the packet

            /* pass all packets to ethernet_input, which decides what packets it supports */
            if (netif.input(p, &netif) != ERR_OK)
            {
                LOG("ethernetif_input: IP input error");
                LWIP_DEBUGF(NETIF_DEBUG, ("ethernetif_input: IP input error\n"));
                pbuf_free(p);
                p = NULL;
            }
        }
        else
        {
            // No packets available
            //LOG("W5500: No packets available");
            break;
        }
    }
}

uint16_t Eth_W5500::sendFrame(const uint8_t *data, uint16_t datalen)
{
    return w5500->sendFrame(data, datalen);
}

static err_t w5500_output(struct netif *netif, struct pbuf *p)
{
    const struct w5500_info *eth = (const struct w5500_info *)netif->state;
    struct pbuf *q;
    uint16_t offset;

#if ETH_PAD_SIZE
    pbuf_remove_header(p, ETH_PAD_SIZE); /* drop the padding word */
#endif

    offset = 0;
    for (q = p; q != NULL; q = q->next)
    {
        /* Send the data from the pbuf to the interface, one pbuf at a
            time. The size of the data in each pbuf is kept in the ->len
            variable. */
        memcpy(eth_w5500.ethTxBuf + offset, q->payload, q->len);
        offset += q->len;
    }

    uint16_t sent = eth_w5500.sendFrame(eth_w5500.ethTxBuf, offset);
    LOG("SEND frame requested: %d, sent: %d", offset, sent);


    MIB2_STATS_NETIF_ADD(netif, ifoutoctets, p->tot_len);
    if (((u8_t *)p->payload)[0] & 1)
    {
        /* broadcast or multicast packet*/
        MIB2_STATS_NETIF_INC(netif, ifoutnucastpkts);
    }
    else
    {
        /* unicast packet */
        MIB2_STATS_NETIF_INC(netif, ifoutucastpkts);
    }
    /* increase ifoutdiscards or ifouterrors on error */

#if ETH_PAD_SIZE
    pbuf_add_header(p, ETH_PAD_SIZE); /* reclaim the padding word */
#endif

    LINK_STATS_INC(link.xmit);

    return ERR_OK;
}

err_t w5500_ethernetif_init(struct netif *netif)
{
    LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
    /* Initialize interface hostname */
    netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

    /*
     * Initialize the snmp variables and counters inside the struct netif.
     * The last argument should be replaced with your link speed, in units
     * of bits per second.
     */
    MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

    netif->name[0] = 'e';
    netif->name[1] = '0';

	/* maximum transfer unit */
	netif->mtu = 1500;

	/* device capabilities */
	netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_IGMP;

    err_t igmp_result;
    igmp_result = igmp_start( netif );
    LOG("IGMP START e0: %u", igmp_result);

/* We directly use etharp_output() here to save a function call.
 * You can instead declare your own function an call etharp_output()
 * from it if you have to do some checks before sending (e.g. if link
 * is available...) */
#if LWIP_IPV4
    netif->output = etharp_output;
#endif /* LWIP_IPV4 */
#if LWIP_IPV6
    netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */
    netif->linkoutput = w5500_output;

    return ERR_OK;
}
