/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#include <hexo/types.h>
#include <hexo/device.h>
#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include "net-ne2000pci.h"

#include "net-ne2000pci-private.h"
#include "ne2000.h"

CONTAINER_FUNC(static inline, ne2000_queue, DLIST, ne2000_queue, NOLOCK, list_entry);

#ifndef CONFIG_STATIC_DRIVERS

/*
 * PCI identifiers of compatible cards.
 */

static const struct devenum_ident_s	net_ne2000pci_ids[] =
  {
    { .vendor = 0x10ec, .device = 0x8029 },	/* Realtek 8029 */
    { .vendor = 0x1050, .device = 0x0940 },	/* Winbond 89C940 */
    { .vendor = 0x1050, .device = 0x5a5a },	/* Winbond 89C940F */
    { .vendor = 0x8c4a, .device = 0x1980 },	/* Winbond 89C940 (bad ROM) */
    { .vendor = 0x11f6, .device = 0x1401 },	/* Compex ReadyLink 2000 */
    { .vendor = 0x8e2e, .device = 0x3000 },	/* KTI ET32P2 */
    { .vendor = 0x4a14, .device = 0x5000 },	/* NetVin NV5000SC */
    { .vendor = 0x12c3, .device = 0x0058 },	/* HolTek HT80232 */
    { .vendor = 0x1106, .device = 0x0926 },	/* Via 86C926 */
    { .vendor = 0x10bd, .device = 0x0e34 },	/* SureCom NE34 */
    { 0 }
  };

/*
 * Driver operations vector.
 */

const struct driver_s	net_ne2000pci_drv =
{
  .id_table		= net_ne2000pci_ids,

  .f_init		= net_ne2000pci_init,
  .f_cleanup		= net_ne2000pci_cleanup,
  .f_irq		= net_ne2000pci_irq,
  .f.net = {
    .f_preparepkt	= net_ne2000pci_preparepkt,
    .f_sendpkt		= net_ne2000pci_sendpkt,
    .f_register_proto	= net_ne2000pci_register_proto,
  }
};
#endif

/*
 * update the command register.
 */

static inline void	ne2000_command(struct device_s	*dev,
				       uint_fast8_t	cmd)
{
  uint_fast16_t		addr = dev->addr[NET_NE2000_COMMAND];

  cpu_io_write_8(addr, cpu_io_read_8(addr) | cmd);
}

/*
 * prepare sending a packet.
 */

static void	ne2000_send(struct device_s	*dev)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
#if defined(CONFIG_NE2000_FRAGMENT) || defined(CONFIG_NETWORK_AUTOALIGN)
  /* XXX copy in several times */
#else
  /* XXX copy in one time */
#endif

  /* the data being written, we wait for remote DMA to be completed (IRQ) */
}

/*
 * device IRQ handling.
 */

DEV_IRQ(net_ne2000pci_irq)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  uint_fast8_t			isr;

  ne2000_command(dev, NE2000_P0);

  isr = cpu_io_read_8(dev->addr[NET_NE2000_ISR]);

  /* remote DMA completed */
  if (isr & NE2000_RDC)
    {
      uint_fast16_t	length = pv->current->header[0].size;
      /* the packet is in the device memory, we can send it */
      /* set page start */
      cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_TPSR, pv->tx_buf);

      /* set packet size */
      if (length < ETH_ZLEN)
	length = ETH_ZLEN;
      cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_TBCR0, length);
      cpu_io_write_8(dev->addr[NET_NE2000_ADDR] + NE2000_TBCR1, length >> 8);

      /* send it ! */
      ne2000_command(dev, NE2000_TXP);

      /* acknowledge interrupt */
      cpu_io_write_8(dev->addr[NET_NE2000_ISR], NE2000_RDC);
    }

  /* packet transmitted */
  if (isr & NE2000_PTX)
    {
      struct ne2000_packet_s	*wait;

      /* packet sent successfully, drop it */
      packet_obj_refdrop(pv->current);

      /* one or more packets in the wait queue ? */
      if ((wait = ne2000_queue_pop(&pv->queue)))
	{
	  pv->current = wait->packet;

	  ne2000_send(dev);

	  mem_free(wait);
	}
      else
	pv->current = NULL;

      /* acknowledge interrupt */
      cpu_io_write_8(dev->addr[NET_NE2000_ISR], NE2000_PTX);
    }

  /* transmit error */
  if (isr & NE2000_TXE)
    {
      /* XXX */

      /* acknowledge interrupt */
      cpu_io_write_8(dev->addr[NET_NE2000_ISR], NE2000_TXE);
    }

  /* packet received */
  if (isr & NE2000_PRX)
    {
      /* XXX */

      /* acknowledge interrupt */
      cpu_io_write_8(dev->addr[NET_NE2000_ISR], NE2000_PRX);
    }

  /* buffer full */
  if (isr & NE2000_OVW)
    {
      /* XXX */

      /* acknowledge interrupt */
      cpu_io_write_8(dev->addr[NET_NE2000_ISR], NE2000_OVW);
    }

  return 0;
}

/*
 * initializing the driver and the device.
 */

DEV_INIT(net_ne2000pci_init)
{
  struct net_ne2000_context_s	*pv;

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &net_ne2000pci_drv;
#endif

  printf("ne2000pci driver init on device %p\n", dev);

  /* driver private data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  lock_init(&pv->lock);

  ne2000_queue_init(&pv->queue);
  net_protos_init(&pv->protocols);

  dev->drv_pv = pv;

  /* XXX init device here */

  /* XXX register as a net device */

  return 0;
}

/*
 * cleaning the driver.
 */

DEV_CLEANUP(net_ne2000pci_cleanup)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  struct ne2000_packet_s	*wait;

  /* empty the waitqueue */
  while ((wait = ne2000_queue_pop(&pv->queue)))
    {
      packet_obj_refdrop(wait->packet);

      mem_free(wait);
    }

  /* destroy some containers */
  ne2000_queue_destroy(&pv->queue);
  net_protos_destroy(&pv->protocols);

  /* turn the device off */
  ne2000_command(dev, NE2000_STP);

  /* free private data */
  mem_free(pv);
}

/*
 * preparing a packet.
 */

DEVNET_PREPAREPKT(net_ne2000pci_preparepkt)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  struct net_header_s		*nethdr;
  uint_fast16_t			total = 0;
  uint8_t			*buff;

#ifdef CONFIG_NE2000_FRAGMENT
  total = sizeof (struct ether_header) + size + 2;
#else
  total = sizeof (struct ether_header) + size;
#endif

  buff = packet->packet = mem_alloc(total, MEM_SCOPE_CONTEXT);

  nethdr = &packet->header[0];
  nethdr->data = buff;
  nethdr->size = total;

  packet->stage = 1;

  packet->sMAC = pv->mac;
  packet->MAClen = ETH_ALEN;

#ifdef CONFIG_NE2000_FRAGMENT
  /* when we use fragmentation, the next packet will be aligned on 4
     bytes boundary */
  return buff + sizeof (struct ether_header) + 2;
#else
  return buff + sizeof (struct ether_header);
#endif
}

/*
 * sending a packet.
 */

DEVNET_SENDPKT(net_ne2000pci_sendpkt)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  struct ether_header		*hdr;
  struct net_header_s		*nethdr;

  /* get a pointer to the header */
  nethdr = &packet->header[0];
  hdr = (struct ether_header*)nethdr->data;

  /* fill the header */
  memcpy(hdr->ether_shost, packet->sMAC, packet->MAClen);
  memcpy(hdr->ether_dhost, packet->tMAC, packet->MAClen);
  net_be16_store(hdr->ether_type, proto);

  /* take lock */
  lock_spin_irq(&pv->lock);

  /* if the device is busy, queue the packet */
  if (cpu_io_read_8(dev->addr[NET_NE2000_COMMAND]) & NE2000_TXP ||
      pv->current)
    {
      struct ne2000_packet_s	*item;

      item = mem_alloc(sizeof (struct ne2000_packet_s), MEM_SCOPE_CONTEXT);
      item->packet = packet;
      ne2000_queue_push(&pv->queue, item);

      return;
    }

  /* otherwise, send the datagram immediately */
  pv->current = packet;

  /* release lock */
  lock_release_irq(&pv->lock);

  ne2000_send(dev);
}

/*
 * registering a new protocol.
 */

DEVNET_REGISTER_PROTO(net_ne2000pci_register_proto)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  va_list			va;

  va_start(va, proto);

  /* call the protocol constructor */
  if (proto->desc->initproto)
    proto->desc->initproto(dev, proto, va);

  /* insert in the protocol list */
  net_protos_push(&pv->protocols, proto);

  va_end(va);
}
