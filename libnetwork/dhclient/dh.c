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

/*
 * DHCPv4 client
 *
 */

#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <netinet/ip.h>
#include <net/if_arp.h>
#include <linux/if.h>
#include <net/ethernet.h>
#include <netpacket/packet.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

struct		ether_arp
{
  struct arphdr	ea_hdr;		/* fixed-size header */
  uint8_t	arp_sha[ETH_ALEN];	/* sender hardware address */
  uint32_t	arp_spa;		/* sender protocol address */
  uint8_t	arp_tha[ETH_ALEN];	/* target hardware address */
  uint32_t	arp_tpa;		/* target protocol address */
} __attribute__ ((packed));

typedef int error_t;
typedef int bool_t;
typedef int socket_t;

int main()
{
  return dhcp_client("eth1");
}

/*
 * Ports.
 */

#define BOOTP_SERVER_PORT	67
#define BOOTP_CLIENT_PORT	68

/*
 * BOOTP operations.
 */

#define BOOTREQUEST	1
#define BOOTREPLY	2

/*
 * DHCP options identifiers.
 */

#define DHCP_PAD	0
#define DHCP_NETMASK	1
#define DHCP_ROUTER	3
#define DHCP_HOSTNAME	12
#define DHCP_REQIP	50
#define DHCP_MSG	53
#define DHCP_SERVER	54
#define DHCP_REQLIST	55
#define DHCP_ID		61
#define DHCP_END	255

/*
 * DHCP message types.
 */

#define DHCPDISCOVER	1
#define DHCPOFFER	2
#define DHCPREQUEST	3
#define DHCPDECLINE	4
#define DHCPACK		5
#define DHCPNACK	6
#define DHCPRELEASE	7
#define DHCPINFORM	8

/*
 * Misc.
 */

#define DHCP_TIMEOUT	10 /* 10 seconds */

/*
 * DHCP options.
 */

struct		dhcp_opt_s
{
  uint8_t	code;
  uint8_t	len;
  uint8_t	data[1];
} __attribute__((packed));

/*
 * DHCP header.
 */

struct			dhcphdr
{
  uint8_t		op;
  uint8_t		htype;
  uint8_t		hlen;
  uint8_t		hops;
  uint32_t		xid;
  uint16_t		secs;
  uint16_t		flags;
  uint32_t		ciaddr;
  uint32_t		yiaddr;
  uint32_t		siaddr;
  uint32_t		giaddr;
  uint8_t		chaddr[16];
  uint8_t		sname[64];
  uint8_t		file[128];
  uint8_t		magic[4];
} __attribute__((packed));


struct net_if_s
{
  int index;
  int mtu;
  char mac[6];
};

/*
 * This function broadcasts an ARP request to ensure an IP address is
 * not already assigned.
 */

static bool_t		dhcp_ip_is_free(struct net_if_s	*interface,
					uint_fast32_t	ip)
{
  socket_t		sock;
  struct sockaddr_ll	addr_sll;
  struct ether_arp	arp;
  bool_t		one = 1;
  struct timeval	tv;

  /* create a PF_PACKET socket */
  if ((sock = socket(PF_PACKET, SOCK_DGRAM, htons(ETH_P_ARP))) == NULL)
    return 0;

  if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &one, sizeof (bool_t)) < 0)
    goto leave;

  tv.tv_usec = 0;
  tv.tv_sec = 2;

  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof (struct timeval)) < 0)
    goto leave;

  addr_sll.sll_family = AF_PACKET;
  addr_sll.sll_protocol = htons(ETH_P_ARP);
  addr_sll.sll_ifindex = interface->index;

  /* bind it to the interface */
  if (bind(sock, (struct sockaddr *)&addr_sll, sizeof (struct sockaddr_ll)) < 0)
    goto leave;

  /* build an ARP request */
  arp.ea_hdr.ar_hrd = htons(ARPHRD_ETHER);
  arp.ea_hdr.ar_pro = htons(ETHERTYPE_IP);
  arp.ea_hdr.ar_hln = ETH_ALEN;
  arp.ea_hdr.ar_pln = 4;
  arp.ea_hdr.ar_op = htons(ARPOP_REQUEST);
  memcpy(arp.arp_sha, interface->mac, ETH_ALEN);
  arp.arp_spa = 0;
  arp.arp_tpa = ip;

  /* send the request */
  addr_sll.sll_halen = ETH_ALEN;
  memcpy(addr_sll.sll_addr, "\xff\xff\xff\xff\xff\xff", ETH_ALEN);

  if (sendto(sock, &arp, sizeof (struct ether_arp), 0, &addr_sll, sizeof (struct sockaddr_ll)) !=
      sizeof (struct ether_arp))
    goto leave;

  /* XXX timeout */

  /* wait reply */
  while (recv(sock, &arp, sizeof (struct ether_arp), 0) > 0)
    {
      if (arp.ea_hdr.ar_hrd == htons(ARPHRD_ETHER) && arp.ea_hdr.ar_pro == htons(ETHERTYPE_IP) &&
	  arp.ea_hdr.ar_hln == ETH_ALEN && arp.ea_hdr.ar_pln == 4 &&
	  arp.ea_hdr.ar_op == htons(ARPOP_REPLY))
	{
	  /* positive reply, the address is in use */
	  if (arp.arp_spa == ip)
	    goto leave;
	}
    }

  return 1;

 leave:

  shutdown(sock, SHUT_RDWR);

  return 0;
}

/*
 * This function reads a raw packet to detect if it is a DHCP reply
 * destinated to us.
 */

static struct dhcphdr	*dhcp_is_for_me(struct net_if_s	*interface,
					uint8_t		*packet,
					socket_t	sock)
{
  struct iphdr		*ip;
  struct udphdr		*udp;
  struct dhcphdr	*dhcp;

  /* get IP header */
  ip = (struct iphdr *)packet;
  if (ip->protocol != IPPROTO_UDP || ip->tot_len < sizeof (struct dhcphdr) + sizeof (struct udphdr))
    return NULL;

  /* get UDP header */
  udp = (struct udphdr *)((uint8_t*)packet + ip->ihl * 4);

  if (ntohs(udp->dest) != BOOTP_CLIENT_PORT || ntohs(udp->len) < sizeof (struct dhcphdr))
    return NULL;

  /* get DHCP header */
  dhcp = (struct dhcphdr *)(udp + 1);

  if (dhcp->op == BOOTREPLY && dhcp->xid == (uintptr_t)sock &&
      !memcmp(dhcp->magic, "\x63\x82\x53\x63", 4))
    {
      if (!memcmp(dhcp->chaddr, interface->mac, ETH_ALEN))
	return dhcp;
    }

  return NULL;
}

/*
 * This function browses the DHCP options to find a given value.
 */

static struct dhcp_opt_s	*dhcp_get_opt(struct dhcphdr	*dhcp,
					      uint8_t		opt)
{
  struct dhcp_opt_s		*p = (struct dhcp_opt_s *)(dhcp + 1);

  while (p->code != DHCP_END && p->code != opt)
    {
      p = (struct dhcp_opt_s *)((uint8_t *)p + 2 + p->len);
    }

  return p->code == DHCP_END ? NULL : p;
}

/*
 * This function creates the sockets used for sending and receiving
 * packets.
 */

static error_t		dhcp_init(struct net_if_s	*interface,
				  socket_t		*sock,
				  socket_t		*sock_packet)
{
  struct sockaddr_in	addr;
  struct sockaddr_ll	addr_sll;
  bool_t		one = 1;
  struct timeval	tv;

  /* create an UDP socket */
  if ((*sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) == NULL)
    return -1;

  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(BOOTP_CLIENT_PORT);

  /* bind it to the client port */
  if (bind(*sock, (struct sockaddr *)&addr, sizeof (struct sockaddr_in)) < 0)
    goto leave;

  if (setsockopt(*sock, SOL_SOCKET, SO_BROADCAST, &one, sizeof (bool_t)) < 0)
    goto leave;

  /* create a PF_PACKET socket */
  if ((*sock_packet = socket(PF_PACKET, SOCK_DGRAM, htons(ETH_P_IP))) == NULL)
    goto leave;

  addr_sll.sll_family = AF_PACKET;
  addr_sll.sll_protocol = htons(ETH_P_IP);
  addr_sll.sll_ifindex = interface->index;

  /* bind it to the interface */
  if (bind(*sock_packet, (struct sockaddr *)&addr_sll, sizeof (struct sockaddr_ll)) < 0)
    goto leave;

  tv.tv_usec = 0;
  tv.tv_sec = DHCP_TIMEOUT;

  if (setsockopt(*sock_packet, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof (struct timeval)) < 0)
    goto leave;

  return 0;

 leave:
  return -1;
}

#define FAKE_MAC	"\x00\xe0\x00\x00\x00\x01"

/*
 * This function emits a DHCP discover message.
 */

static error_t		dhcp_packet(struct net_if_s	*interface,
				    uint8_t		type,
				    uint_fast32_t	ip,
				    uint_fast32_t	serv,
				    socket_t		sock)
{
  struct sockaddr_in	broadcast;
  struct dhcphdr	*packet = NULL;
  struct dhcp_opt_s	*opt;
  uint8_t		*raw;
  size_t		packet_len;
  char			*str;

  /* init and send a DHCPDISCOVER */
  broadcast.sin_family = AF_INET;
  broadcast.sin_addr.s_addr = serv;
  broadcast.sin_port = htons(BOOTP_SERVER_PORT);

  if ((packet = malloc(interface->mtu)) == NULL)
    return -1;

  memset(packet, 0, sizeof (struct dhcphdr));

  /* setup DHCP header */
  packet->op = BOOTREQUEST;
  packet->htype = ARPHRD_ETHER;
  packet->hlen = ETH_ALEN;
  packet->xid = (uintptr_t)sock;
  memcpy(packet->magic, "\x63\x82\x53\x63", 4);

  memcpy(packet->chaddr, interface->mac, ETH_ALEN);

  /* fill DHCP options */
  opt = raw = (void *)(packet + 1);
  opt->code = DHCP_MSG;
  opt->len = 1;
  opt->data[0] = type;
  raw += 3;

  opt = (void *)raw;
  opt->code = DHCP_ID;
  opt->len = 1 + ETH_ALEN;
  opt->data[0] = ARPHRD_ETHER;
  memcpy(&opt->data[1], interface->mac, ETH_ALEN);
  raw += (3 + ETH_ALEN);

  if (type != DHCPDISCOVER)
    {
      uint32_t	*addr;

      /* if not a discovery packet, include requested ip and destination server */
      packet->siaddr = serv;
      packet->yiaddr = ip;

      opt = (void *)raw;
      opt->code = DHCP_REQIP;
      opt->len = 4;
      addr = (uint32_t *)opt->data;
      *addr = ip;
      raw += 6;

      opt = (void *)raw;
      opt->code = DHCP_SERVER;
      opt->len = 4;
      addr = (uint32_t *)opt->data;
      *addr = serv;
      raw += 6;
    }

  opt = (void *)raw;
  opt->code = DHCP_REQLIST;
  opt->len = 3;
  opt->data[0] = DHCP_NETMASK;
  opt->data[1] = DHCP_HOSTNAME;
  opt->data[2] = DHCP_ROUTER;
  raw += 5;

  opt = (void *)raw;
  opt->code = DHCP_END;

  packet_len = (uint8_t *)opt - (uint8_t *)packet;

  switch (type)
    {
      case DHCPDISCOVER:
	str = "DHCPDISCOVER";
	break;
      case DHCPREQUEST:
	str = "DHCPREQUEST";
	break;
      case DHCPDECLINE:
	str = "DHCPDECLINE";
	break;
      default:
	str = NULL;
	break;
    }

  if (str != NULL)
    printf("dhclient: sending %s...\n", str);

  if (sendto(sock, packet, packet_len, 0, (struct sockaddr *)&broadcast,
	     sizeof (struct sockaddr_in)) < 0)
    {
      free(packet);
      return -1;
    }

  free(packet);
  return 0;
}

/*
 * This function waits for DHCP offers and reply to them.
 */

static error_t		dhcp_request(struct net_if_s	*interface,
				     socket_t		sock,
				     socket_t		sock_packet)
{
  ssize_t		sz;
  uint8_t		*packet = NULL;
  struct dhcphdr	*dhcp;
  struct dhcp_opt_s	*opt;
  bool_t		requested = 0;

  /* XXX timeout */

  if ((packet = malloc(interface->mtu)) == NULL)
    return -1;

  /* receive DHCPOFFER & acks */
  while ((sz = recv(sock_packet, packet, interface->mtu, 0)) >= 0)
    {
      if ((dhcp = dhcp_is_for_me(interface, packet, sock)) != NULL)
	{
	  if ((opt = dhcp_get_opt(dhcp, DHCP_MSG)) == NULL)
	    continue;

	  switch (opt->data[0])
	    {
	      case DHCPOFFER:
		printf("dhclient: received DHCPOFFER from %u.%u.%u.%u\n",
		       (dhcp->siaddr >> 0) & 0xFF, (dhcp->siaddr >> 8) & 0xFF,
		       (dhcp->siaddr >> 16) & 0xFF, (dhcp->siaddr >> 24) & 0xFF);

		printf("dhclient: offered %u.%u.%u.%u ... ",
		       (dhcp->yiaddr >> 0) & 0xFF, (dhcp->yiaddr >> 8) & 0xFF,
		       (dhcp->yiaddr >> 16) & 0xFF, (dhcp->yiaddr >> 24) & 0xFF);

		/* if the IP is free, take the offer */
		if (!requested && dhcp_ip_is_free(interface, dhcp->yiaddr))
		  {
		    printf("is free. Accepting.\n");

		    /* send DHCPREQUEST */
		    requested = !dhcp_packet(interface, DHCPREQUEST, dhcp->yiaddr, dhcp->siaddr, sock);
		  }
		else /* otherwise, decline the offer */
		  {
		    printf("is not free. Declining.\n");

		    /* send DHCPDECLINE */
		    dhcp_packet(interface, DHCPDECLINE, dhcp->yiaddr, dhcp->siaddr, sock);
		  }
		break;
	      case DHCPACK:
		printf("dhclient: end of negociation.\n");

		/* ifconfig & route add XXX */

		/* we've got an address :-)) */
		return 0;
	      case DHCPNACK:

		break;
	      default:
		break;
	    }
	}
    }

  return -1;
}

/*
 * DHCP client entry function. Request an address for the given
 * interface.
 */

error_t			dhcp_client(const char	*ifname)
{
  socket_t		sock;
  socket_t		sock_packet;
  struct net_if_s	*interface;

#if 0
  if ((interface = if_get_by_name(ifname)) == NULL)
    return -1;
#else
  struct ifreq ifr;

  interface = malloc(sizeof (struct net_if_s));
  interface->mtu = 1500;
  memcpy(interface->mac, FAKE_MAC, 6);
  interface->index = 0;

  sock = socket(AF_INET, SOCK_DGRAM, 0);
  ifr.ifr_ifindex = 0;
  strcpy (ifr.ifr_name, ifname);
  assert(ioctl (sock, SIOGIFINDEX, &ifr) >= 0);
  interface->index = ifr.ifr_ifindex;
#endif

  /* ifconfig 0.0.0.0 XXX */

  /* create sockets */
  if (dhcp_init(interface, &sock, &sock_packet))
    return -1;

  /* discover DHCP servers */
  if (dhcp_packet(interface, DHCPDISCOVER, 0, INADDR_BROADCAST, sock))
    goto leave;

  /* answer DHCP offers */
  if (dhcp_request(interface, sock, sock_packet))
    goto leave;

  shutdown(sock, SHUT_RDWR);
  shutdown(sock_packet, SHUT_RDWR);

  return 0;

 leave:
  printf("dhclient: error, leaving\n");

  shutdown(sock, SHUT_RDWR);
  shutdown(sock_packet, SHUT_RDWR);

  return -1;
}
