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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

/**
   @file
   @module{Network stack library}
   @short Network address structure
 */

#ifndef NET_ADDR_H
#define NET_ADDR_H

#include <hexo/types.h>

/**
   @this is a network address.  It contains all addresses that can be
   set by any layer in the networking stack.  Its fields are enabled
   depending on compiled network layers and libraries.
 */
struct net_addr_s
{
#if defined(CONFIG_BLE)
  uint8_t reliable;
  uint8_t llid;
  uint16_t att;
  uint16_t cid;
#endif
#if defined(CONFIG_NET_TCP) || defined(CONFIG_NET_UDP)
  uint16_t port;
#endif
#if defined(CONFIG_NET_IPV4)
  uint8_t ipv4[4];
#endif
#if defined(CONFIG_NET_IPV6)
  uint8_t ipv6[16];
#endif
#if defined(CONFIG_NET_ETHERNET)
  uint8_t mac[6];
#endif
  
};

#endif
