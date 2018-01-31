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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#ifndef BLE_ADDRESS_H_
#define BLE_ADDRESS_H_

/**
   @file
   @module {Libraries::Bluetooth Low Energy}
   @short Bluetooth device address declaration

   @ref {ble_addr_s} {Device addresses} are defined as a 48-bit
   address + 1 bit type (Random / Public).  There are various tools
   provided to generate and resolve random addresses.

   @list
     @item @ref ble_stack_context_address_resolvable_generate and
           @ref ble_stack_context_address_non_resolvable_generate
           can generate addresses,
     @item @ref ble_addr_random_type_set can override address type,
     @item @ref #BLE_ADDR_FMT and @ref #BLE_ADDR_ARG can help
           printing addresses with printf-like functions,
     @item @ref ble_addr_cmp can compare addresses,
     @item @ref ble_addr_net_parse and @ref ble_addr_net_set can
           convert from/to @ref {net_addr_s} {libnetwork addresses}.
   @end list
 */

#include <hexo/types.h>
#include <string.h>

struct net_addr_s;

/** @internal */
extern const char *const ble_addr_rand_name[4];

/**
   @this defines BLE address types.
 */
enum ble_addr_type_e
{
  /** Public IEEE-allocated address */
  BLE_ADDR_PUBLIC,
  /** Random address */
  BLE_ADDR_RANDOM,
};

/**
   @this defines BLE random address types.
 */
enum ble_addr_random_type_e
{
  BLE_ADDR_RANDOM_NON_RESOLVABLE,
  BLE_ADDR_RANDOM_RESOLVABLE,
  /** @internal */
  BLE_ADDR_RANDOM_INVALID,
  BLE_ADDR_RANDOM_STATIC,
};

/**
   @this defines a BLE address.
 */
struct ble_addr_s
{
  /** 48-bit address looking like a MAC address */
  uint8_t addr[6];
  /** BLE address type */
  enum ble_addr_type_e type:1;
};

/**
   @this can be used to print a BLE address
   @see #BLE_ADDR_ARG
 */
#define BLE_ADDR_FMT "[%s %02x:%02x:%02x:%02x:%02x:%02x]"

/**
   @this can be used to print a BLE address
   @see #BLE_ADDR_FMT
 */
#define BLE_ADDR_ARG(x)                                                 \
  (x)->type == BLE_ADDR_RANDOM ? ble_addr_rand_name[(x)->addr[5] >> 6]  \
                               : "Public",                              \
  (x)->addr[5], (x)->addr[4], (x)->addr[3],                             \
  (x)->addr[2], (x)->addr[1], (x)->addr[0]

/**
   @this retrieves random address type from a random address.
 */
ALWAYS_INLINE
enum ble_addr_random_type_e ble_addr_random_type(const struct ble_addr_s *a)
{
  if (a->type != BLE_ADDR_RANDOM)
    return BLE_ADDR_RANDOM_INVALID;
  return a->addr[5] >> 6;
}

/**
   @this sets address as random with a given random type.
 */
ALWAYS_INLINE
void ble_addr_random_type_set(struct ble_addr_s *a, enum ble_addr_random_type_e t)
{
  a->type = BLE_ADDR_RANDOM;
  a->addr[5] &= 0x3f;
  a->addr[5] |= t << 6;
}

/**
   @this compares two BLE addresses and returns a difference between
   them, if any.  This can be used to sort addresses.
 */
ALWAYS_INLINE
int_fast8_t ble_addr_cmp(const struct ble_addr_s *a,
                         const struct ble_addr_s *b)
{
  int8_t diff = (uint8_t)a->type - (uint8_t)b->type;

  if (diff)
    return diff;

  return memcmp(a->addr, b->addr, 6);
}

/**
   @this fills in BLE address from libnetwork's address type.
 */
void ble_addr_net_parse(struct ble_addr_s *addr, const struct net_addr_s *naddr);

/**
   @this fills in libnetwork's address from BLE address type.
 */
void ble_addr_net_set(const struct ble_addr_s *addr, struct net_addr_s *naddr);

#endif
