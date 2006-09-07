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
 * device open operation
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
  uint8_t			*buff;
  size_t			size;
  net_proto_id_t		proto;
  struct net_header_s		*nethdr;
  struct net_header_s		*next_nethdr;

  /* create and read the packet from the card */
  if (!(size = net_ns8390_read(pv, &buff)))
    return 1;

  packet = packet_create();
  packet->packet = buff;

  nethdr = &packet->header[0];
  nethdr->data = packet->packet;
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
  next_nethdr = &packet->header[1];
  next_nethdr->data = packet->packet + sizeof(struct ether_header);
  next_nethdr->size = nethdr->size - sizeof(struct ether_header);
  packet->stage++;

  /* dispatch to the matching protocol */
  proto = net_be16_load(hdr->ether_type);
  if ((p = net_protos_lookup(&pv->protocols, proto)))
    p->desc->pushpkt(dev, packet, p);
  else
    printf("NETWORK: no protocol to handle packet (id = 0x%x)\n", proto);

  return 1;
}

/*
 * register a new protocol for the device.
 */

DEVNET_REGISTER_PROTO(net_ns8390_register_proto)
{
  struct net_ns8390_context_s	*pv = dev->drv_pv;
  struct net_proto_s		*proto;

  proto = mem_alloc(sizeof (struct net_proto_s) + desc->pv_size,
		    MEM_SCOPE_THREAD);

  proto->desc = desc;
  proto->id = desc->id;
  proto->pv = (void*)((uint8_t*)proto + sizeof (struct net_proto_s));

  net_protos_push(&pv->protocols, proto);

  return proto;
}

/*
 * device packet creation operation
 */

DEVNET_PREPAREPKT(net_ns8390_preparepkt)
{
  struct net_ns8390_context_s	*pv = dev->drv_pv;
  struct net_header_s		*nethdr;
  struct net_header_s		*next_nethdr;
  uint_fast16_t			total = 0;

  total = sizeof (struct ether_header) + size;

  packet->packet = mem_alloc(total, MEM_SCOPE_THREAD);

  nethdr = &packet->header[0];
  nethdr->data = packet->packet;
  nethdr->size = total;
  next_nethdr = &packet->header[1];
  next_nethdr->data = packet->packet + sizeof (struct ether_header);
  next_nethdr->size = size;

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

  dummy_push(dev, packet, NULL);
  net_ns8390_write(pv, packet->packet, nethdr->size);
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

