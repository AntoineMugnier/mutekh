/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr> (c) 2012

*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <device/irq.h>
#include <hexo/iospace.h>
#include <device/class/net.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <mutek/mem_alloc.h>

#include <arch/mem_checker.h>

#include <mutek/printk.h>

#include <hexo/interrupt.h>

#include <network/if.h>

#define ETH_MAX_FIFO 8

struct soclib_eth_context_s
{
  uintptr_t addr;
  struct dev_irq_ep_s irq_ep;
  uint8_t mac[6];
  uint32_t fifo_size;

  struct net_packet_s *rx_pkt[ETH_MAX_FIFO];
  uint32_t rx_ptr, rx_count;

  struct net_packet_s *tx_pkt[ETH_MAX_FIFO];
  uint32_t tx_ptr, tx_count;

  struct net_if_s *interface;
};

#define ETHERNET_TX_SIZE   0
#define ETHERNET_TX_FIFO   4
#define ETHERNET_RX_SIZE   8
#define ETHERNET_RX_FIFO   12
#define ETHERNET_STATUS    16
#define ETHERNET_CTRL      16
#define ETHERNET_FIFO_SIZE 20
#define ETHERNET_MAC_LOW   24
#define ETHERNET_MAC_HIGH  28

#define ETHERNET_RX_EMPTY       0
#define ETHERNET_RX_DONE        1
#define ETHERNET_RX_DMA_ERR     2
#define ETHERNET_RX_PHY_ERR     3

#define ETHERNET_TX_EMPTY       0
#define ETHERNET_TX_DONE        1
#define ETHERNET_TX_DMA_ERR     2
#define ETHERNET_TX_PHY_ERR     3

#define ETHERNET_ST_LINK_UP     1
#define ETHERNET_ST_TX_DONE     2
#define ETHERNET_ST_RX_DONE     4

#define ETHERNET_CTRL_RESET     1
#define ETHERNET_CTRL_TX_IRQ    2
#define ETHERNET_CTRL_RX_IRQ    4
#define ETHERNET_CTRL_LINK_IRQ  8

static DEV_NET_PREPAREPKT(soclib_eth_preparepkt)
{
  struct device_s             *dev = ndev->dev;
  struct soclib_eth_context_s *pv = dev->drv_pv;

  if (size > ETHERMTU)
    return NULL;

  struct net_header_s		*nethdr;
  uint_fast16_t			total = sizeof (struct ether_header) + size;
  uint8_t			*buff;

  if ((buff = packet->packet = mem_alloc(total, (mem_scope_context))) == NULL)
    return NULL;

  nethdr = &packet->header[0];
  nethdr->data = buff;
  nethdr->size = total;

  packet->stage = 1;
  packet->sMAC = pv->mac;
  packet->MAClen = ETH_ALEN;

  return buff + sizeof (struct ether_header);
}

static DEV_NET_SENDPKT(soclib_eth_sendpkt)
{
  struct device_s             *dev = ndev->dev;
  struct soclib_eth_context_s *pv = dev->drv_pv;
  struct ether_header         *hdr;
  struct net_header_s         *nethdr = &packet->header[0];

  if (packet->stage == 0)
    {
      /* get a pointer to the header */
      hdr = (struct ether_header*)nethdr->data;

      /* fill the header */
      memcpy(hdr->ether_shost, packet->sMAC, ETH_ALEN);
      memcpy(hdr->ether_dhost, packet->tMAC, ETH_ALEN);
      net_be16_store(hdr->ether_type, proto);
    }

  LOCK_SPIN_IRQ(&dev->lock);

  // drop packet if TX fifo full
  if (pv->tx_count < pv->fifo_size)
    {
      packet_obj_refnew(packet);
      pv->tx_pkt[(pv->tx_ptr + pv->tx_count++) % pv->fifo_size] = packet;

      /* invalidate dcache, force dcache write before dma */
      cpu_dcache_invld_buf(nethdr->data, nethdr->size);
      
      cpu_mem_write_32(pv->addr + ETHERNET_TX_SIZE, endian_le32(nethdr->size));
      cpu_mem_write_32(pv->addr + ETHERNET_TX_FIFO, endian_le32((uintptr_t)nethdr->data));
    }

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_NET_SETOPT(soclib_eth_setopt)
{
  struct device_s               *dev = ndev->dev;
  struct soclib_eth_context_s *pv = dev->drv_pv;

  switch (option)
    {
    case DEV_NET_OPT_PROMISC:
      return -ENOTSUP;

    case DEV_NET_OPT_BCAST:
      return -ENOTSUP;

    case DEV_NET_OPT_MAC:
      return -ENOTSUP;

    default:
      return -EINVAL;
    }
}

static DEV_NET_GETOPT(soclib_eth_getopt)
{
  struct device_s               *dev = ndev->dev;
  struct soclib_eth_context_s *pv = dev->drv_pv;

  switch (option)
    {
    case DEV_NET_OPT_PROMISC:
      *(bool_t*)value = 0;
      return 0;

    case DEV_NET_OPT_BCAST:
      *(const uint8_t**)value = "\xff\xff\xff\xff\xff\xff";
      *len = 6;
      return 0;

    case DEV_NET_OPT_MAC:
      *(const uint8_t**)value = pv->mac;
      *len = 6;
      return 0;

    default:
      return -EINVAL;
    }
}

static void soclib_eth_clean_fifos(struct soclib_eth_context_s *pv)
{
  cpu_mem_write_32(pv->addr + ETHERNET_CTRL, endian_le32(ETHERNET_CTRL_RESET));

  while (pv->rx_count--)
    packet_obj_refdrop(pv->rx_pkt[pv->rx_ptr++ % pv->fifo_size]);

  while (pv->tx_count--)
    packet_obj_refdrop(pv->tx_pkt[pv->tx_ptr++ % pv->fifo_size]);
}

static size_t soclib_eth_fill_rx(struct soclib_eth_context_s *pv, uint_fast8_t count)
{
  size_t done = 0;
  struct net_packet_s *packet;

  while (count--)
    {
      /* create a new packet object */
      if ((packet = packet_obj_new(NULL)) == NULL)
        break;

      if (!(packet->packet = mem_alloc(ETHERMTU, (mem_scope_sys))))
        {
          packet_obj_refdrop(packet);
          break;
        }

      pv->rx_pkt[(pv->rx_ptr + pv->rx_count++) % pv->fifo_size] = packet;

      cpu_mem_write_32(pv->addr + ETHERNET_RX_SIZE, endian_le32(ETHERMTU));
      cpu_mem_write_32(pv->addr + ETHERNET_RX_FIFO, endian_le32((uintptr_t)packet->packet));
      done++;
    }

  return done;
}

static DEV_IRQ_EP_PROCESS(soclib_eth_irq)
{
  struct device_s *dev = ep->dev;
  struct soclib_eth_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  uint32_t st;
  do {
    st = endian_le32(cpu_mem_read_32(pv->addr + ETHERNET_STATUS));

    if (st & ETHERNET_ST_TX_DONE)
      {
        uint32_t tx_st = endian_le32(cpu_mem_read_32(pv->addr + ETHERNET_TX_FIFO));

        struct net_packet_s *packet = pv->tx_pkt[pv->tx_ptr++ % pv->fifo_size];
        // printk("Packet TX done: %p\n", packet);

        pv->tx_count--;
        packet_obj_refdrop(packet);
      }

    if (st & ETHERNET_ST_RX_DONE)
      {
        uint32_t size = endian_le32(cpu_mem_read_32(pv->addr + ETHERNET_RX_SIZE));
        uint32_t rx_st = endian_le32(cpu_mem_read_32(pv->addr + ETHERNET_RX_FIFO));

        struct net_packet_s *packet = pv->rx_pkt[pv->rx_ptr++ % pv->fifo_size];
        pv->rx_count--;
        soclib_eth_fill_rx(pv, 1);

        // printk("Packet RX done: %u %P\n", size, packet->packet, size);

        if (rx_st == ETHERNET_RX_DONE && size >= 42)
          {
            uint8_t *data = packet->packet;
            soclib_mem_mark_initialized(data, size);
            /* invalidate dcache after dma */
            cpu_dcache_invld_buf(data, size);

            struct net_header_s *nethdr = &packet->header[0];
            nethdr->data = data;
            nethdr->size = size;

            /* get the good header */
            struct ether_header *hdr = (struct ether_header *)data;

            /* fill some info */
            packet->MAClen = ETH_ALEN;
            packet->sMAC = hdr->ether_shost;
            packet->tMAC = hdr->ether_dhost;
            packet->proto = net_be16_load(hdr->ether_type);

            /* prepare packet for next stage */
            nethdr[1].data = data + sizeof(struct ether_header);
            nethdr[1].size = size - sizeof(struct ether_header);

            packet->stage++;
            if_dispatch(pv->interface, packet);
          }

        packet_obj_refdrop(packet);
      }

  } while (st & (ETHERNET_ST_TX_DONE | ETHERNET_ST_RX_DONE));

  lock_release(&dev->lock);
}

/* 
 * device open operation
 */

static const struct dev_enum_ident_s	soclib_eth_ids[] =
{
  DEV_ENUM_FDTNAME_ENTRY("soclib:vci_ethernet"),
  { 0 }
};

static DEV_INIT(soclib_eth_init);
static DEV_CLEANUP(soclib_eth_cleanup);

static const struct driver_net_s	soclib_eth_net_drv =
{
  .class_		= DRIVER_CLASS_NET,
  .f_preparepkt         = soclib_eth_preparepkt,
  .f_sendpkt            = soclib_eth_sendpkt,
  .f_setopt             = soclib_eth_setopt,
  .f_getopt             = soclib_eth_getopt,
};

const struct driver_s	soclib_eth_drv =
{
  .desc                 = "SoCLib Ethernet",
  .id_table		= soclib_eth_ids,
  .f_init		= soclib_eth_init,
  .f_cleanup		= soclib_eth_cleanup,
  .classes              = { &soclib_eth_net_drv, 0 }
};

REGISTER_DRIVER(soclib_eth_drv);

static DEV_INIT(soclib_eth_init)
{
  struct soclib_eth_context_s	*pv;

  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  memset(pv, 0, sizeof(*pv));

  if (!pv)
    goto err;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  cpu_mem_write_32(pv->addr + ETHERNET_CTRL, endian_le32(ETHERNET_CTRL_RESET));

  uint_fast8_t i;
  for (i = 0; i < 6; i++)
    pv->mac[i] = cpu_mem_read_8(pv->addr + ETHERNET_MAC_LOW + i);

  pv->fifo_size = endian_le32(cpu_mem_read_32(pv->addr + ETHERNET_FIFO_SIZE));
  if (pv->fifo_size == 0)
    goto err_mem;

  device_irq_source_init(dev, &pv->irq_ep, 1,
                         &soclib_eth_irq, DEV_IRQ_SENSE_HIGH_LEVEL);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_mem;

  if (soclib_eth_fill_rx(pv, pv->fifo_size) < 1)
    goto err_irq;

  dev->status = DEVICE_DRIVER_INIT_DONE;
  dev->drv = &soclib_eth_drv;

  pv->interface = if_register(dev, IF_ETHERNET, ETHERMTU);
  if (pv->interface == NULL)
    goto err_fifo;

  /* enable irqs */
  cpu_mem_write_32(pv->addr + ETHERNET_CTRL, endian_le32(ETHERNET_CTRL_TX_IRQ + ETHERNET_CTRL_RX_IRQ));

  return 0;

 err_if:
  if_unregister(pv->interface);
 err_fifo:
  soclib_eth_clean_fifos(pv);
 err_irq:
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
 err_mem:
  mem_free(pv);
 err:
  dev->status = DEVICE_DRIVER_INIT_FAILED;
  return -1;
}

DEV_CLEANUP(soclib_eth_cleanup)
{
  struct soclib_eth_context_s	*pv = dev->drv_pv;

  /* disable irqs */
  soclib_eth_clean_fifos(pv);

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  if_unregister(pv->interface);

  mem_free(pv);
}

