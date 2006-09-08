/*
 * IP protocol
 *
 */

#include <netinet/ip.h>
#include <netinet/packet.h>
#include <netinet/protos.h>
#include <hexo/device.h>

#include <stdio.h>

/*
 * Structures for declaring the protocol's properties & interface.
 */

static const struct ip_interface_s	ip_interface =
{
  .send = ip_send
};

const struct net_proto_desc_s	ip_protocol =
  {
    .name = "IP",
    .id = ETHERTYPE_IP,
    .pushpkt = ip_pushpkt,
    .preparepkt = ip_preparepkt,
    .initproto = ip_init,
    .f.ip = &ip_interface,
    .pv_size = sizeof (struct net_pv_ip_s),
  };

NET_INITPROTO(ip_init)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s*)proto->pv;

  memset(pv->addr, 0, 4);
}

NET_PUSHPKT(ip_pushpkt)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s*)protocol->pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct iphdr		aligned;
#endif
  struct iphdr		*hdr;
  struct net_header_s	*nethdr;
  uint_fast16_t		hdr_len;
  net_proto_id_t	proto;
  struct net_proto_s	*p;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct iphdr*)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!ALIGNED(hdr, sizeof (uint16_t)))
    {
      memcpy(&aligned, hdr, sizeof (struct iphdr));
      hdr = &aligned;
    }
#endif

  /* update packet info */
  packet->sIP = (uint8_t*)&hdr->saddr;
  packet->tIP = (uint8_t*)&hdr->daddr;

  /* XXX check IP version, checksum */

  /* XXX fragments, etc... */

  /* is the packet really for me ? */
  if (memcmp(&hdr->daddr, pv->addr, 4))
    return;

  /* next stage */
  if (!nethdr[1].data)
    {
      hdr_len = hdr->ihl * 4;
      nethdr[1].data = nethdr->data + hdr_len;
      nethdr[1].size = net_be16_load(hdr->tot_len) - hdr_len;
    }

  /* update ARP cache */
  arp_update_table(dev, net_protos_lookup(protocols, ETHERTYPE_ARP),
		   packet->sIP, packet->sMAC);

  packet->stage++;

  /* dispatch to the matching protocol */
  proto = hdr->protocol;
  if ((p = net_protos_lookup(protocols, proto)))
    p->desc->pushpkt(dev, packet, p, protocols);
  else
    printf("NETWORK: no protocol to handle packet (id = 0x%x)\n", proto);
}

NET_PREPAREPKT(ip_preparepkt)
{
  struct net_header_s	*nethdr;

  dev_net_preparepkt(dev, packet, 20 + size); /* XXX */

  nethdr = &packet->header[packet->stage];
  nethdr[1].data = nethdr->data + 20; /* XXX */
  nethdr[1].size = size;

  packet->stage++;
}

static uint_fast16_t	ip_checksum(uint8_t		*data,
				    size_t		size)
{
  uint_fast32_t		checksum = 0;
  uint16_t		*d = (uint16_t*)data;

  while(size > 1)
    {
      checksum = checksum + *d++;
      size = size - 2;
    }

  if (size)
    checksum = checksum + *(uint8_t*)d;

  while (checksum >> 16)
    checksum = (checksum & 0xffff) + (checksum >> 16);

  return ~checksum;
}

NET_IP_SEND(ip_send)
{
  struct net_pv_ip_s	*pv = (struct net_pv_ip_s*)ip->pv;
#ifdef CONFIG_NETWORK_AUTOALIGN
  struct iphdr		aligned;
#endif
  struct iphdr		*hdr;
  struct net_header_s	*nethdr;

  /* get the header */
  nethdr = &packet->header[packet->stage];
  hdr = (struct iphdr*)nethdr->data;

  /* align the packet on 16 bits if necessary */
#ifdef CONFIG_NETWORK_AUTOALIGN
  if (!ALIGNED(hdr, sizeof (uint16_t)))
    {
      hdr = &aligned;
      memset(hdr, 0, sizeof (struct iphdr));
    }
#endif

  /* fill IP header */
  hdr->version = 4;
  hdr->ihl = 5;
  hdr->tos = 0;
  net_be16_store(hdr->tot_len, nethdr->size);
  net_be16_store(hdr->id, 0);	/* XXX */
  hdr->fragment = 0;		/* XXX */
  hdr->flags = (0 << 1);	/* XXX */
  hdr->ttl = 64;
  hdr->protocol = proto->id;
  memcpy(&hdr->saddr, pv->addr, 4);
  memcpy(&hdr->daddr, packet->tIP, 4);
  /* checksum */
  hdr->check = ip_checksum(hdr, hdr->ihl * 4); /* XXX align */

#ifdef CONFIG_NETWORK_AUTOALIGN
  memcpy(nethdr->data, hdr, sizeof (struct iphdr));
  hdr = (struct iphdr*)nethdr->data;
#endif

  packet->sIP = (uint8_t*)&hdr->saddr;
  packet->tIP = (uint8_t*)&hdr->daddr;
  packet->tMAC = arp_get_mac(dev, pv->arp, packet->tIP);

  packet->stage--;
  /* send the packet to the driver */
  dev_net_sendpkt(dev, packet, ETHERTYPE_IP);
}

