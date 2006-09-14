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

#include "net-ns8390.h"

#include "net-ns8390-private.h"

#include "ns8390.h"

/**************************************************************/

/*
 * device operations
 */

#ifndef CONFIG_STATIC_DRIVERS
static const struct devenum_ident_s	net_ns8390_ids[] =
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

const struct driver_s	net_ns8390_drv =
{
  .id_table		= net_ns8390_ids,

  .f_init		= net_ns8390_init,
  .f_cleanup		= net_ns8390_cleanup,
  .f_irq		= net_ns8390_irq,
  .f.net = {
    .f_preparepkt	= net_ns8390_preparepkt,
    .f_sendpkt		= net_ns8390_sendpkt,
    .f_register_proto	= net_ns8390_register_proto,
  }
};
#endif

/*
 * device IRQ handler
 */

DEV_IRQ(net_ns8390_irq)
{
  struct net_ns8390_context_s	*pv = dev->drv_pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_header		aligned;
#endif
  struct ether_header		*hdr;
  struct net_proto_s		*p;
  struct net_packet_s		*packet;
  struct net_header_s		*nethdr;
  net_proto_id_t		proto;
  uint8_t			*buff;
  size_t			size;

  printf("irq\n");

  /* create and read the packet from the card */
  if (!(size = net_ns8390_read(pv, &buff)))
    return 1;

  packet = packet_obj_new(NULL);
  packet->packet = buff;

  nethdr = &packet->header[0];
  nethdr->data = buff;
  nethdr->size = size;

  /* get the good header */
  hdr = (struct ether_header*)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!NET_ALIGNED(hdr, sizeof (uint16_t)))
    {
      memcpy(&aligned, hdr, sizeof (struct ether_header));
      hdr = &aligned;
    }
#endif

  /* fill some info */
  packet->MAClen = sizeof(struct ether_addr);
  packet->sMAC = hdr->ether_shost;
  packet->tMAC = hdr->ether_dhost;

  /* prepare packet for next stage */
  nethdr[1].data = buff + sizeof(struct ether_header);
  nethdr[1].size = size - sizeof(struct ether_header);

  packet->stage++;

  /* dispatch to the matching protocol */
  proto = net_be16_load(hdr->ether_type);
  if ((p = net_protos_lookup(&pv->protocols, proto)))
    p->desc->pushpkt(dev, packet, p, &pv->protocols);
  else
    printf("NETWORK: no protocol to handle packet (id = 0x%x)\n", proto);

  packet_obj_refdrop(packet);

  return 1;
}

/*
 * register a new protocol for the device.
 */

DEVNET_REGISTER_PROTO(net_ns8390_register_proto)
{
  struct net_ns8390_context_s	*pv = dev->drv_pv;
  va_list			va;

  va_start(va, proto);

  if (proto->desc->initproto)
    proto->desc->initproto(dev, proto, va);

  net_protos_push(&pv->protocols, proto);

  va_end(va);
}

/*
 * device packet creation operation
 */

DEVNET_PREPAREPKT(net_ns8390_preparepkt)
{
  struct net_ns8390_context_s	*pv = dev->drv_pv;
  struct net_header_s		*nethdr;
  uint_fast16_t			total = 0;
  uint8_t			*buff;

  total = sizeof (struct ether_header) + size;

  buff = packet->packet = mem_alloc(total, MEM_SCOPE_CONTEXT);

  nethdr = &packet->header[0];
  nethdr->data = buff;
  nethdr->size = total;

  packet->stage = 1;

  packet->sMAC = pv->mac;
  packet->MAClen = ETH_ALEN;

  return buff + sizeof (struct ether_header);
}

/*
 * device packet sending operation
 */

DEVNET_SENDPKT(net_ns8390_sendpkt)
{
  struct net_ns8390_context_s	*pv = dev->drv_pv;
  struct ether_header		*hdr;
  struct net_header_s		*nethdr;

  /* get a pointer to the header */
  nethdr = &packet->header[0];
  hdr = (struct ether_header*)nethdr->data;

  /* fill the header */
  memcpy(hdr->ether_shost, packet->sMAC, packet->MAClen);
  memcpy(hdr->ether_dhost, packet->tMAC, packet->MAClen);
  net_be16_store(hdr->ether_type, proto);

#ifndef CONFIG_NETWORK_AUTOALIGN
  net_ns8390_write(pv, packet->packet, nethdr->size);
#else
# ifdef CONFIG_NS8390_FRAGMENT
  /* XXX */
# else
  uint_fast8_t			i;
  uint8_t			*p;

  p = packet->data;
  for (i = 0; i < packet->total_stages; i++)
    {
    }
# endif
#endif

  packet_obj_refdrop(packet);
}

/*
 * device close operation
 */

DEV_CLEANUP(net_ns8390_cleanup)
{
  struct net_ns8390_context_s	*pv = dev->drv_pv;

  net_protos_destroy(&pv->protocols);

  mem_free(pv);
}

/*
 * device open operation
 */

DEV_INIT(net_ns8390_init)
{
  struct net_ns8390_context_s	*pv;
  struct net_proto_s		*rarp;
  struct net_proto_s		*arp;
  struct net_proto_s		*ip;
  struct net_proto_s		*icmp;
  struct net_proto_s		*udp;

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &net_ns8390_drv;
#endif

  printf("ns8390 (ne2000) driver init on device %p\n", dev);

  /* driver private data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  lock_init(&pv->lock);

  net_protos_init(&pv->protocols);

  dev->drv_pv = pv;

  if (net_ns8390_probe(pv, dev->addr[NET_NS8390_ADDR]))
    {
      printf("ns8390: unable to initialize card\n");
      return -1;
    }

  /* reset the device */
  net_ns8390_reset(pv);
  /* bind to ICU */
  DEV_ICU_BIND(icudev, dev);

  /* initialize protocols */
  ip = net_alloc_proto(&ip_protocol);
  arp = net_alloc_proto(&arp_protocol);
  rarp = net_alloc_proto(&rarp_protocol);
  icmp = net_alloc_proto(&icmp_protocol);
  udp = net_alloc_proto(&udp_protocol);

  /* register protocols into the driver */
  dev_net_register_proto(dev, ip, arp);
  dev_net_register_proto(dev, arp, ip);
  dev_net_register_proto(dev, rarp, ip);
  dev_net_register_proto(dev, icmp, ip);
  dev_net_register_proto(dev, udp, ip);

  /* a RARP request is used to assign us an IP */
  rarp_request(dev, rarp, NULL);

  while (0)
    net_ns8390_irq(dev);	/* XXX remove me ! */

  asm ("sti");

  return 0;
}

