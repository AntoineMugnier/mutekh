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
const struct driver_s	net_ns8390_drv =
{
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
  if (!ALIGNED(hdr, sizeof (uint16_t)))
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

  dummy_push(dev, packet, NULL, NULL); /* XXX remove me ! */

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

  buff = packet->packet = mem_alloc(total, MEM_SCOPE_THREAD);

  nethdr = &packet->header[0];
  nethdr->data = buff;
  nethdr->size = total;
  nethdr[1].data = buff + sizeof (struct ether_header);
  nethdr[1].size = size;

  packet->stage = 1;

  packet->sMAC = pv->mac;
  packet->MAClen = ETH_ALEN;
}

/*
 * device packet sending operation
 */

DEVNET_SENDPKT(net_ns8390_sendpkt)
{
  struct net_ns8390_context_s	*pv = dev->drv_pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct ether_header		aligned;
#endif
  struct ether_header		*hdr;
  struct net_header_s		*nethdr;

  /* get a pointer to the header */
  nethdr = &packet->header[0];
  hdr = (struct ether_header*)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!ALIGNED(hdr, sizeof (uint16_t)))
    {
      hdr = &aligned;
      memset(hdr, 0, sizeof (struct ether_header));
    }
#endif

  /* fill the header */
  memcpy(hdr->ether_shost, packet->sMAC, packet->MAClen);
  memcpy(hdr->ether_dhost, packet->tMAC, packet->MAClen);
  net_be16_store(hdr->ether_type, proto);

#ifdef CONFIG_NETWORK_AUTOALIGN
  memcpy(nethdr->data, hdr, sizeof (struct ether_header));
#endif

  dummy_push(dev, packet, NULL, NULL); /* XXX remove me ! */

  net_ns8390_write(pv, packet->packet, nethdr->size);

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

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &net_ns8390_drv;
#endif

  /* driver private data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  lock_init(&pv->lock);

  net_protos_init(&pv->protocols);

  dev->drv_pv = pv;

  if (net_ns8390_probe(pv, dev->addr[NET_NS8390_ADDR]))
    {
      printf("No NE2000 device found\n");
      return -1;
    }

  net_ns8390_reset(pv);

  return 0;
}

