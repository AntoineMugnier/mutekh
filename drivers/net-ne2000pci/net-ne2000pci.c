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
 * device IRQ handling.
 */

DEV_IRQ(net_ne2000pci_irq)
{
  /* XXX */

  return 0;
}

/*
 * initializing the driver and the device.
 */

DEV_INIT(net_ne2000pci_init)
{
  struct net_ne2000_context_s	*pv;
  struct net_proto_s		*rarp;
  struct net_proto_s		*arp;
  struct net_proto_s		*ip;
  struct net_proto_s		*icmp;
  struct net_proto_s		*udp;

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &net_ne2000pci_drv;
#endif

  printf("ne2000pci driver init on device %p\n", dev);

  /* driver private data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  lock_init(&pv->lock);

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

  net_protos_destroy(&pv->protocols);

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
  /* XXX */
}

/*
 * registering a new protocol.
 */

DEVNET_REGISTER_PROTO(net_ne2000pci_register_proto)
{
  struct net_ne2000_context_s	*pv = dev->drv_pv;
  va_list			va;

  va_start(va, proto);

  if (proto->desc->initproto)
    proto->desc->initproto(dev, proto, va);

  net_protos_push(&pv->protocols, proto);

  va_end(va);
}
