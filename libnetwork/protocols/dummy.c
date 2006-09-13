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
 * this is a dummy packet handler.
 *
 * it only dumps packets.
 */

#include <netinet/dummy.h>
#include <netinet/packet.h>
#include <netinet/protos.h>

#include <stdio.h>

/*
 * Structures for declaring the protocol's properties & interface.
 */

const struct net_proto_desc_s	dummy_protocol =
  {
    .name = "dummy",
    .id = 0,
    .pushpkt = dummy_push,
    .preparepkt = NULL,
    .f.other = NULL,
    .pv_size = 0
  };

/*
 * This function hexdump a packet.
 */

static void		dummy_hexdump(uint8_t	*data,
				      size_t	sz)
{
  uint_fast32_t		i, j;
  uint8_t		c;

  printf("Data len = %u\n", sz);
  for (i = 0; i < sz; i += 16)
    {
      printf("%7x ", i);

      for (j = 0; j < 16 && i + j < sz; j++)
	{
	  printf("%02x ", data[i + j]);
	}

      for (; j < 16; j++)
	printf("   ");

      for (j = 0; j < 16 && i + j < sz; j++)
	{
	  c = data[i + j];

	  if (c >= 32 && c < 127)
	    putchar(c);
	  else
	    putchar('.');
	}
      putchar('\n');
    }
}

/*
 * Dump a packet.
 */

NET_PUSHPKT(dummy_push)
{
  return;

  printf("Source MAC: %2x:%2x:%2x:%2x:%2x:%2x\n",
	 packet->sMAC[0], packet->sMAC[1], packet->sMAC[2],
	 packet->sMAC[3], packet->sMAC[4], packet->sMAC[5]);

  printf("Target MAC: %2x:%2x:%2x:%2x:%2x:%2x\n",
	 packet->tMAC[0], packet->tMAC[1], packet->tMAC[2],
	 packet->tMAC[3], packet->tMAC[4], packet->tMAC[5]);

  printf("Packet dump:\n");

  dummy_hexdump(packet->header[packet->stage].data,
		packet->header[packet->stage].size);
}

