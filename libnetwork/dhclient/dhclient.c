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

#include <hexo/error.h>
#include <hexo/endian.h>

#include <netinet/if.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <netinet/dhcp.h>
#include <netinet/arp.h>
#include <netinet/udp.h>
#include <netinet/route.h>
#include <netinet/socket.h>
#include <netinet/packet.h>

#include <stdlib.h>
#include <timer.h>

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
  timer_delay_t		t;

  /* create a PF_PACKET socket */
  if ((sock = socket(PF_PACKET, SOCK_DGRAM, htons(ETH_P_ARP))) == NULL)
    return 0;

  if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &one, sizeof (bool_t)) < 0)
    goto leave;

  tv.tv_usec = 0;
  tv.tv_sec = ARP_REQUEST_TIMEOUT / 1000;

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
  endian_32_na_store(&arp.arp_spa, 0);
  arp.arp_tpa = ip;

  /* send the request */
  addr_sll.sll_halen = ETH_ALEN;
  memcpy(addr_sll.sll_addr, "\xff\xff\xff\xff\xff\xff", ETH_ALEN);

  if (sendto(sock, &arp, sizeof (struct ether_arp), 0, (struct sockaddr *)&addr_sll, sizeof (struct sockaddr_ll)) < 0)
    goto leave;

  t = timer_get_tick(&timer_ms);

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

      if (timer_get_tick(&timer_ms) - t > ARP_REQUEST_TIMEOUT)
	break;
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
 * This function browses the DHCP options to find a given value. XXX check overflow
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
      /* if not a discovery packet, include requested ip and destination server */
      packet->siaddr = serv;
      packet->yiaddr = ip;

      opt = (void *)raw;
      opt->code = DHCP_REQIP;
      opt->len = 4;
      endian_32_na_store(opt->data, ip);
      raw += 6;

      opt = (void *)raw;
      opt->code = DHCP_SERVER;
      opt->len = 4;
      endian_32_na_store(opt->data, serv);
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
				     socket_t		sock_packet,
				     struct dhcp_lease_s *lease)
{
  ssize_t		sz;
  uint8_t		*packet = NULL;
  struct dhcphdr	*dhcp;
  struct dhcp_opt_s	*opt;
  bool_t		requested = 0;
  timer_delay_t		t;

  if ((packet = malloc(interface->mtu)) == NULL)
    return -1;

  t = timer_get_tick(&timer_ms);

  /* receive DHCPOFFER & acks */
  while ((sz = recv(sock_packet, packet, interface->mtu, 0)) >= 0)
    {
      if (timer_get_tick(&timer_ms) - t > DHCP_TIMEOUT * 1000)
	break;

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
		{
		  struct net_route_s	*route;
		  struct net_addr_s	addr;
		  struct net_addr_s	mask;

		  printf("dhclient: end of negociation.\n");

		  lease->serv = dhcp->siaddr;
		  lease->ip = dhcp->yiaddr;

		  /* compute lease time */
		  opt = dhcp_get_opt(dhcp, DHCP_LEASE);
		  if (opt != NULL)
		    lease->delay = (endian_be32_na_load(opt->data) / 2) * 1000;
		  else
		    lease->delay = DHCP_DFL_LEASE;

		  /* configure IP */
		  IPV4_ADDR_SET(addr, 0);
		  if_config(interface->index, IF_DEL, &addr, &addr);
		  IPV4_ADDR_SET(addr, dhcp->yiaddr);

		  /* if netmask is present, use it, otherwise guess it */
		  opt = dhcp_get_opt(dhcp, DHCP_NETMASK);
		  if (opt != NULL)
		    IPV4_ADDR_SET(mask, endian_32_na_load(opt->data));
		  else
		    {
		      if (IN_CLASSA(dhcp->yiaddr))
			IPV4_ADDR_SET(mask, IN_CLASSA_NET);
		      else if (IN_CLASSB(dhcp->yiaddr))
			IPV4_ADDR_SET(mask, IN_CLASSB_NET);
		      else if (IN_CLASSC(dhcp->yiaddr))
			IPV4_ADDR_SET(mask, IN_CLASSC_NET);
		    }
		  printf("dhclient:\n  attributed %u.%u.%u.%u netmask %u.%u.%u.%u\n",
			 (addr.addr.ipv4 >> 0) & 0xff, (addr.addr.ipv4 >> 8) & 0xff,
			 (addr.addr.ipv4 >> 16) & 0xff, (addr.addr.ipv4 >> 24) & 0xff,
			 (mask.addr.ipv4 >> 0) & 0xff, (mask.addr.ipv4 >> 8) & 0xff,
			 (mask.addr.ipv4 >> 16) & 0xff, (mask.addr.ipv4 >> 24) & 0xff);
		  if_config(interface->index, IF_SET, &addr, &mask);

		  printf("  lease time: %u seconds\n", lease->delay / 1000);
		  if ((opt = dhcp_get_opt(dhcp, DHCP_HOSTNAME)) != NULL)
		    {
		      char	name[opt->len + 1];

		      memcpy(name, opt->data, opt->len);
		      name[opt->len] = 0;
		      printf("  hostname: %s\n", name);
		    }

		  if ((opt = dhcp_get_opt(dhcp, DHCP_ROUTER)) != NULL)
		    {
		      struct net_route_s	*def;
		      uint32_t			gateway;

		      gateway = endian_32_na_load(opt->data);

		      printf("  gateway: %u.%u.%u.%u\n",
			     (gateway >> 0) & 0xff, (gateway >> 8) & 0xff,
			     (gateway >> 16) & 0xff, (gateway >> 24) & 0xff);

		      /* configure default route */
		      if ((def = mem_alloc(sizeof (struct net_route_s *), MEM_SCOPE_NETWORK)) != NULL)
			{
			  def->interface = interface;
			  IPV4_ADDR_SET(def->target, 0x0);
			  IPV4_ADDR_SET(def->mask, 0x0);
			  def->is_routed = 1;
			  IPV4_ADDR_SET(def->router, gateway);
			  route_add(def);
			}
		    }

		  /* configure default route */
		  if ((route = mem_alloc(sizeof (struct net_route_s *), MEM_SCOPE_NETWORK)) != NULL)
		    {
		      route->interface = interface;
		      IPV4_ADDR_SET(route->target, addr.addr.ipv4 & mask.addr.ipv4);
		      memcpy(&route->mask, &mask, sizeof (struct net_addr_s));
		      route->is_routed = 0;
		      route_add(route);
		    }

		  /* we've got an address :-)) */
		  free(packet);
		  return 0;
		}
		break;
	      case DHCPNACK:
		/* error during attribution */
		printf("dhclient: error.\n");
		requested = 0;
		break;
	      default:
		break;
	    }
	}
    }

  free(packet);

  return -1;
}

/*
 * DHCP renew thread.
 */

static TIMER_CALLBACK(dhcp_renew)
{
  struct dhcp_lease_s	*lease = (struct dhcp_lease_s *)pv;
  socket_t		sock = NULL;
  socket_t		sock_packet = NULL;
  uint_fast8_t		i;

  /* create sockets */
  if (dhcp_init(lease->interface, &sock, &sock_packet))
    goto leave;

  /* try to renew (12 times is about 2 min) */
  for (i = 0; i < 12; i++)
    {
      /* send a DHCPREQUEST */
      dhcp_packet(lease->interface, DHCPREQUEST, lease->ip, lease->serv, sock);

      /* wait answer */
      if (!dhcp_request(lease->interface, sock, sock_packet, lease))
	break;
    }

  /* critical error, panic */
  if (i == 12)
    {
      /* best solution is to restart from the beginning... */

      /* discover DHCP servers */
      if (dhcp_packet(lease->interface, DHCPDISCOVER, 0, INADDR_BROADCAST, sock))
	goto leave;

      /* answer DHCP offers */
      if (dhcp_request(lease->interface, sock, sock_packet, lease))
	goto leave;
    }

 leave:

  if (sock != NULL)
    shutdown(sock, SHUT_RDWR);
  if (sock_packet != NULL)
    shutdown(sock_packet, SHUT_RDWR);

  /* start the timer again */
  timer_add_event(&timer_ms, timer);
}

/*
 * DHCP client entry function. Request an address for the given
 * interface.
 */

error_t			dhcp_client(const char	*ifname)
{
  struct timer_event_s	*timer;
  struct dhcp_lease_s	*lease;
  socket_t		sock = NULL;
  socket_t		sock_packet = NULL;
  struct net_if_s	*interface;
  struct net_addr_s	null;
  struct net_route_s	route;

  if ((interface = if_get_by_name(ifname)) == NULL)
    return -1;

  if ((lease = malloc(sizeof (struct dhcp_lease_s))) == NULL)
    return -1;

  lease->interface = interface;

  /* ifconfig 0.0.0.0 */
  IPV4_ADDR_SET(null, 0);
  if_config(interface->index, IF_SET, &null, &null);
  route.interface = if_get_by_name(ifname);
  IPV4_ADDR_SET(route.target, 0x0);
  IPV4_ADDR_SET(route.mask, 0x0);
  route.is_routed = 0;
  route_add(&route);

  /* create sockets */
  if (dhcp_init(interface, &sock, &sock_packet))
    goto leave;

  /* discover DHCP servers */
  if (dhcp_packet(interface, DHCPDISCOVER, 0, INADDR_BROADCAST, sock))
    goto leave;

  /* answer DHCP offers */
  if (dhcp_request(interface, sock, sock_packet, lease))
    goto leave;

  /* start DHCP renew timer */
  if ((timer = malloc(sizeof (struct timer_event_s))) != NULL)
    {
      timer->callback = dhcp_renew;
      timer->delay = lease->delay;
      timer->pv = lease;
      timer_add_event(&timer_ms, timer);
    }

  route_del(&route);

  shutdown(sock, SHUT_RDWR);
  shutdown(sock_packet, SHUT_RDWR);

  return 0;

 leave:
  printf("dhclient: error, leaving\n");

  free(lease);

  route_del(&route);

  if (sock != NULL)
    shutdown(sock, SHUT_RDWR);
  if (sock_packet != NULL)
    shutdown(sock_packet, SHUT_RDWR);

  return -1;
}
