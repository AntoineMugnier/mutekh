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

#include <netinet/if.h>
#include <netinet/in.h>
#include <netinet/dhcp.h>
#include <netinet/arp.h>
#include <netinet/socket.h>

#include <stdlib.h>

static bool_t		dhcp_ip_is_free(uint_fast32_t	ip)
{
  return 1;
}

#define FAKE_MAC	"\x00\xe0\x00\x00\x00\x01"

error_t			dhcp_client(const char	*ifname)
{
  socket_t		sock;
  struct sockaddr_in	broadcast;
  struct sockaddr_in	addr;
  struct dhcphdr	*packet = NULL;
  struct dhcp_opt_s	*opt;
  uint8_t		*raw;
  size_t		packet_len;
  struct net_if_s	*interface;

#if 0
  if ((interface = if_get_by_name(ifname)) == NULL)
    return -1;
#endif

  /* create an UDP socket */
  if ((sock = socket(PF_INET, SOCK_DGRAM, 0)) == NULL)
    return -1;

  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(BOOTP_CLIENT_PORT);

  /* bind it to the client port on the good interface */
  if (bind(sock, (struct sockaddr *)&addr, sizeof (struct sockaddr_in)) < 0)
    return -1;

  if (setsockopt(sock, SOL_SOCKET, SO_BINDTODEVICE, ifname, strlen(ifname)) < 0)
    goto leave;

  /* init and send a DHCPDISCOVER */
  broadcast.sin_family = AF_INET;
  broadcast.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  broadcast.sin_port = htons(BOOTP_SERVER_PORT);

  if ((packet = malloc(1000 /* XXX */)) == NULL)
    goto leave;

  memset(packet, 0, sizeof (struct dhcphdr));

  packet->op = BOOTREQUEST;
  packet->htype = ARPHRD_ETHER;
  packet->hlen = ETH_ALEN;
  packet->xid = (uintptr_t)sock;

#if 0
  memcpy(packet->chaddr, interface->mac, ETH_ALEN);
#else
  memcpy(packet->chaddr, FAKE_MAC, ETH_ALEN);
#endif

  opt = raw = (void *)(packet + 1);
  opt->code = DHCP_MSG;
  opt->len = 1;
  opt->data[0] = DHCPDISCOVER;

  opt = (void *)(raw += 3);
  opt->code = DHCP_ID;
  opt->len = 1 + ETH_ALEN;
  opt->data[0] = ARPHRD_ETHER;
#if 0
  memcpy(&opt->data[1], interface->mac, ETH_ALEN);
#else
  memcpy(&opt->data[1], FAKE_MAC, ETH_ALEN);
#endif

  opt = (void *)(raw += (3 + ETH_ALEN));
  opt->code = DHCP_REQLIST;
  opt->len = 3;
  opt->data[0] = DHCP_NETMASK;
  opt->data[1] = DHCP_HOSTNAME;
  opt->data[2] = DHCP_ROUTER;

  opt = (void *)(raw += 6);
  opt->code = DHCP_END;

  packet_len = (uint8_t *)opt - (uint8_t *)packet;

  printf("dhclient: sending DHCPDISCOVER...\n");

  if (sendto(sock, packet, packet_len, 0, (struct sockaddr *)&broadcast,
	     sizeof (struct sockaddr_in)) < 0)
    goto leave;



  return 0;

 leave:
  printf("dhclient: error, leaving\n");

  if (packet != NULL)
    free(packet);

  shutdown(sock, SHUT_RDWR);

  return -1;
}
