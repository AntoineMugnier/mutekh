/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#ifndef NETWORK_PING_H
#define NETWORK_PING_H

/**
   @file
   @module{Network library}
   @short Ping client
 */

# ifndef CONFIG_NETWORK_PING
#  warning PING support is not enabled in configuration file
# endif

# include <hexo/types.h>
# include <network/packet.h>

# define PING_INTERVAL	200	// ms
# define PING_TIMEOUT	10000	// ms

struct		ping_s
{
  uint_fast32_t	total;
  uint_fast32_t	lost;
  uint_fast32_t	error;
};

error_t		ping(struct net_addr_s	*host,
		     uint_fast32_t	count,
		     size_t		size,
		     struct ping_s	*stat);

#endif
