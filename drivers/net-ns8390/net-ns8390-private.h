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

#ifndef NET_NS8390_PRIVATE_H_
#define NET_NS8390_PRIVATE_H_

#include <hexo/types.h>
#include <hexo/lock.h>
#include <netinet/ether.h>
#include <netinet/protos.h>

/*
 * private data of a ns8390 network device
 */

struct			net_ns8390_context_s
{
  lock_t		lock;

  uint_fast16_t		base;
  uint_fast16_t		asic;
  uint_fast16_t		rx_start;
  uint_fast16_t		tx_start;
  uint_fast16_t		mem;
  uint_fast8_t		mode_16bits;

  uint8_t		mac[ETH_ALEN];

  net_protos_root_t	protocols;
};

#endif

