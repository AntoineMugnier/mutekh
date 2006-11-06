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
 * Ping function
 */

#include <hexo/error.h>

#include <netinet/if.h>
#include <netinet/in.h>
#include <netinet/icmp.h>
#include <netinet/socket.h>

#include <timer.h>

#include <netinet/ping.h>

static void	usleep(uint_fast32_t	usec) /* XXX remove me */
{
  timer_delay_t	t;

  t = timer_get_tick(&timer_ms) + usec / 1000;
  while (timer_get_tick(&timer_ms) < t)
    ;
}

error_t			ping(struct net_addr_s	*host,
			     uint_fast32_t	count,
			     size_t		size,
			     struct ping_s	*stat)
{
  struct sockaddr_in	addr;
  uint8_t		*buf;
  struct icmphdr	*hdr;
  size_t		i;
  socket_t		sock;
  uint_fast32_t		id;
  uint_fast32_t		check;

  /* reset the statistics */
  memset(stat, 0, sizeof (struct ping_s));

  /* allocate and create the message */
  if ((buf = mem_alloc(size + sizeof (struct icmphdr), MEM_SCOPE_SYS)) == NULL)
    return -1;

  for (i = 0; i < size; i++)
    buf[sizeof (struct icmphdr) + i] = 32 + i % 96;

  /* fill icmp header */
  hdr = (struct icmphdr *)buf;
  id = (uint32_t)hdr;
  id = id + (id >> 16);

  hdr->type = 8;
  hdr->code = 0;
  hdr->un.echo.id = id;
  hdr->un.echo.sequence = 0;
  hdr->checksum = 0;
  check = hdr->checksum = ~packet_checksum(buf, size + sizeof (struct icmphdr));

  /* open a SOCK_RAW socket and connect it */
  if ((sock = socket(PF_INET, SOCK_RAW, htons(IPPROTO_ICMP))) == NULL)
    {
      mem_free(buf);
      return -1;
    }

  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(IPV4_ADDR_GET(*host));

  if (connect(sock, (struct sockaddr *)&addr, sizeof (struct sockaddr_in)) < 0)
    {
      mem_free(buf);
      return -1;
    }

  printf("PING %u.%u.%u.%u %u bytes of data.\n",
	 addr.sin_addr.s_addr & 0xff,
	 (addr.sin_addr.s_addr >> 8) & 0xff,
	 (addr.sin_addr.s_addr >> 16) & 0xff,
	 (addr.sin_addr.s_addr >> 24) & 0xff,
	 size);

  /* send successive ping */
  for (i = 0; i < count; i++)
    {
      ssize_t	sent;

      if (i)
	usleep(PING_INTERVAL * 1000);

      sent = send(sock, buf, size + sizeof (struct icmphdr), 0);

      if (sent != size + sizeof (struct icmphdr))
	printf("Error (sent = %d)\n", sent);

      /* increment sequence number */
      hdr->un.echo.sequence++;
      check = hdr->checksum - 1;
      hdr->checksum = check + (check >> 16);
    }

  /* wait for echo */

  mem_free(buf);

  return 0;
}
