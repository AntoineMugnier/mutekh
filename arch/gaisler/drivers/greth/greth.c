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
#include <hexo/interrupt.h>
#include <hexo/iospace.h>

#include <device/irq.h>
#include <device/class/net.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <network/if.h>

#define GRETH_MAX_ETHLEN  1514

#define GRETH_REG_CTRL    0x00
#define  GRETH_CTRL_TX_EN        0x00000001
#define  GRETH_CTRL_RX_EN        0x00000002
#define  GRETH_CTRL_TXIRQ_EN     0x00000004
#define  GRETH_CTRL_RXIRQ_EN     0x00000008
#define  GRETH_CTRL_FULLDUPLEX   0x00000010
#define  GRETH_CTRL_PROMISC      0x00000020
#define  GRETH_CTRL_RESET        0x00000040
#define  GRETH_CTRL_100MBPS      0x00000080
#define  GRETH_CTRL_PHYIRQ       0x00000200
#define  GRETH_CTRL_MULTICAST_EN 0x00000400
#define  GRETH_CTRL_MULTICAST_AV 0x04000000

#define GRETH_REG_STAT   0x04
#define  GRETH_STAT_RX_ERR       0x00000001
#define  GRETH_STAT_TX_ERR       0x00000002
#define  GRETH_STAT_RXIRQ_PEND   0x00000004
#define  GRETH_STAT_TXIRQ_PEND   0x00000008
#define  GRETH_STAT_RXDMA_ERR    0x00000010
#define  GRETH_STAT_TXDMA_ERR    0x00000020
#define  GRETH_STAT_SMALL_RX     0x00000040
#define  GRETH_STAT_INV_ADDR     0x00000080
#define  GRETH_STAT_PHY_ADDR     0x00000100

#define GRETH_REG_MACMSB  0x08
#define GRETH_REG_MACLSB  0x0c
#define GRETH_REG_TXPTR   0x14
#define GRETH_REG_RXPTR   0x18

#define GRETH_TXDESC_FLAGS(len, wrap) ((len) | ((wrap) << 12) | 0x2800)
#define GRETH_TXDESC_ERROR(flags)     ((flags) & 0xc000)
#define GRETH_TXDESC_ENABLED(flags)   ((flags) & 0x800)

#define GRETH_RXDESC_FLAGS(wrap)      (((wrap) << 12) | 0x2800)
#define GRETH_RXDESC_LEN(flags)       ((flags) & 0x7ff)
#define GRETH_RXDESC_ERROR(flags)     ((flags) & 0x3c000)
#define GRETH_RXDESC_MULTICAST(flags) ((flags) & 0x04000000)
#define GRETH_RXDESC_ENABLED(flags)   ((flags) & 0x800)

struct greth_desc_s
{
  uint32_t flags;
  uint32_t addr;
};

#define GRETH_DESC_COUNT   16

struct greth_context_s
{
  uintptr_t addr;
  struct dev_irq_ep_s irq_ep;
  uint8_t mac[6];

  struct greth_desc_s *tx;
  uint32_t tx_ptr, tx_count;
  struct net_packet_s *tx_pkt[GRETH_DESC_COUNT];

  struct greth_desc_s *rx;
  uint32_t rx_ptr, rx_count;
  struct net_packet_s *rx_pkt[GRETH_DESC_COUNT];

  struct net_if_s *interface;
};

static DEV_NET_PREPAREPKT(greth_preparepkt)
{
  struct device_s             *dev = accessor->dev;
  struct greth_context_s *pv = dev->drv_pv;

  if (size > GRETH_MAX_ETHLEN)
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

static DEV_NET_SENDPKT(greth_sendpkt)
{
  struct device_s             *dev = accessor->dev;
  struct greth_context_s      *pv = dev->drv_pv;
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
  if (pv->tx_count < GRETH_DESC_COUNT)
    {
      uint_fast8_t i = (pv->tx_ptr + pv->tx_count++) % GRETH_DESC_COUNT;
      struct greth_desc_s *d = pv->tx + i;

      // printk("Packet TX: %u\n", i);

      packet_obj_refnew(packet);
      pv->tx_pkt[i] = packet;

      d->addr =  endian_be32((uintptr_t)nethdr->data);
      d->flags = endian_be32(GRETH_TXDESC_FLAGS(nethdr->size, i == GRETH_DESC_COUNT - 1));

      cpu_mem_write_32(pv->addr + GRETH_REG_CTRL, endian_be32(GRETH_CTRL_RX_EN | GRETH_CTRL_RXIRQ_EN |
                                                              GRETH_CTRL_TX_EN | GRETH_CTRL_TXIRQ_EN));
    }

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_NET_SETOPT(greth_setopt)
{
  struct device_s               *dev = accessor->dev;
  struct greth_context_s *pv = dev->drv_pv;

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

static DEV_NET_GETOPT(greth_getopt)
{
  struct device_s               *dev = accessor->dev;
  struct greth_context_s *pv = dev->drv_pv;

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

static size_t greth_fill_rx(struct greth_context_s *pv, uint_fast8_t count)
{
  size_t done = 0;
  struct net_packet_s *packet;

  while (count--)
    {
      /* create a new packet object */
      if ((packet = packet_obj_new(NULL)) == NULL)
        break;

      if (!(packet->packet = mem_alloc(GRETH_MAX_ETHLEN, (mem_scope_sys))))
        {
          packet_obj_refdrop(packet);
          break;
        }

      uint_fast8_t i = (pv->rx_ptr + pv->rx_count++) % GRETH_DESC_COUNT;

      pv->rx_pkt[i] = packet;
      pv->rx[i].addr = endian_be32((uintptr_t)packet->packet);
      pv->rx[i].flags = endian_be32(GRETH_RXDESC_FLAGS(i == GRETH_DESC_COUNT - 1));
      done++;
    }

  return done;
}

static DEV_IRQ_EP_PROCESS(greth_irq)
{
  struct device_s *dev = ep->dev;
  struct greth_context_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  uint32_t st;
  do {
    st = endian_be32(cpu_mem_read_32(pv->addr + GRETH_REG_STAT));

    if (st & GRETH_STAT_TXIRQ_PEND)
      {
        uint_fast8_t i = pv->tx_ptr++ % GRETH_DESC_COUNT;
        struct net_packet_s *packet = pv->tx_pkt[i];

        // printk("Packet TX done: %u %p\n", i, packet);

        pv->tx_count--;
        packet_obj_refdrop(packet);
      }

    if (st & GRETH_STAT_RXIRQ_PEND)
      {
        uint_fast8_t i = pv->rx_ptr++ % GRETH_DESC_COUNT;
        struct greth_desc_s *d = pv->rx + i;
        struct net_packet_s *packet = pv->rx_pkt[i];

        pv->rx_count--;

        uint32_t dflags = endian_be32(d->flags);

        if (!GRETH_RXDESC_ERROR(dflags))
          {
            uint8_t *data = packet->packet;
            uint32_t size = GRETH_RXDESC_LEN(dflags);

            // printk("Packet RX done: %u %P\n", size, data, size);

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

        greth_fill_rx(pv, 1);

        packet_obj_refdrop(packet);
      }

    cpu_mem_write_32(pv->addr + GRETH_REG_STAT, endian_be32(st));

  } while (st & (GRETH_STAT_RXIRQ_PEND | GRETH_STAT_TXIRQ_PEND));

  lock_release(&dev->lock);
}

static const struct dev_enum_ident_s	greth_ids[] =
{
  DEV_ENUM_GAISLER_ENTRY(0x1, 0x01d),
  { 0 }
};

static DEV_INIT(greth_init);
static DEV_CLEANUP(greth_cleanup);

static const struct driver_net_s	greth_net_drv =
{
  .class_		= DRIVER_CLASS_NET,
  .f_preparepkt         = greth_preparepkt,
  .f_sendpkt            = greth_sendpkt,
  .f_setopt             = greth_setopt,
  .f_getopt             = greth_getopt,
};

const struct driver_s	greth_drv =
{
  .desc                 = "Gaisler Ethernet",
  .id_table		= greth_ids,
  .f_init		= greth_init,
  .f_cleanup		= greth_cleanup,
  .classes              = { &greth_net_drv, 0 }
};

REGISTER_DRIVER(greth_drv);

static DEV_INIT(greth_init)
{
  struct greth_context_s	*pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;
//  return -1;
  uintptr_t addr;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  /* reset */
  cpu_mem_write_32(addr + GRETH_REG_CTRL, endian_be32(GRETH_CTRL_RESET));
  while (cpu_mem_read_32(addr + GRETH_REG_CTRL) & endian_be32(GRETH_CTRL_RESET))
    ;

  /* allocate private driver data */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    goto err;

  dev->drv_pv = pv;
  memset(pv, 0, sizeof(*pv));
  pv->addr = addr;

  pv->tx = mem_alloc_align(sizeof(struct greth_context_s) * GRETH_DESC_COUNT, 1024, (mem_scope_sys));
  if (!pv->tx)
    goto err_mem;

  pv->rx = mem_alloc_align(sizeof(struct greth_context_s) * GRETH_DESC_COUNT, 1024, (mem_scope_sys));
  if (!pv->rx)
    goto err_mem2;

  greth_fill_rx(pv, GRETH_DESC_COUNT);

  device_irq_source_init(dev, &pv->irq_ep, 1, &greth_irq, DEV_IRQ_SENSE_RISING_EDGE);

  if (device_irq_source_link(dev, &pv->irq_ep, 1, 1))
    goto err_mem3;

  dev->status = DEVICE_DRIVER_INIT_DONE;
  dev->drv = &greth_drv;

#warning FIXME read mac
  uint_fast8_t i;
  for (i = 0; i < 6; i++)
    pv->mac[i] = i;

  pv->interface = if_register(dev, IF_ETHERNET, ETHERMTU);
  if (pv->interface == NULL)
    goto err_irq;

  cpu_mem_write_32(pv->addr + GRETH_REG_MACLSB, endian_be32(pv->mac[5] | (pv->mac[4] << 8) | (pv->mac[3] << 16) | (pv->mac[2] << 24)));
  cpu_mem_write_32(pv->addr + GRETH_REG_MACMSB, endian_be32(pv->mac[1] | (pv->mac[0] << 8)));

  cpu_mem_write_32(pv->addr + GRETH_REG_TXPTR, endian_be32((uintptr_t)pv->tx));
  cpu_mem_write_32(pv->addr + GRETH_REG_RXPTR, endian_be32((uintptr_t)pv->rx));

  /* enable irqs */
  cpu_mem_write_32(pv->addr + GRETH_REG_STAT, endian_be32(GRETH_STAT_RXIRQ_PEND | GRETH_STAT_TXIRQ_PEND));
  cpu_mem_write_32(pv->addr + GRETH_REG_CTRL, endian_be32(GRETH_CTRL_RX_EN | GRETH_CTRL_RXIRQ_EN |
                                                          GRETH_CTRL_TXIRQ_EN));

  return 0;

 err_if:
  if_unregister(pv->interface);
 err_irq:
  device_irq_source_unlink(dev, &pv->irq_ep, 1);
 err_mem3:
  mem_free(pv->rx);
 err_mem2:
  mem_free(pv->tx);
 err_mem:
  mem_free(pv);
 err:
  dev->status = DEVICE_DRIVER_INIT_FAILED;
  return -1;
}

DEV_CLEANUP(greth_cleanup)
{
  struct greth_context_s	*pv = dev->drv_pv;

  cpu_mem_write_32(pv->addr + GRETH_REG_CTRL, endian_be32(GRETH_CTRL_RESET));

  device_irq_source_unlink(dev, &pv->irq_ep, 1);

  if_unregister(pv->interface);

  mem_free(pv->rx);
  mem_free(pv->tx);
  mem_free(pv);
}

