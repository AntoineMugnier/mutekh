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

#include <hexo/types.h>
#include <string.h>

extern const char *const ble_addr_rand_name[4];

enum ble_addr_type_e
{
  BLE_ADDR_PUBLIC,
  BLE_ADDR_RANDOM,
};

enum ble_addr_random_type_e
{
  BLE_ADDR_RANDOM_NON_RESOLVABLE,
  BLE_ADDR_RANDOM_RESOLVABLE,
  BLE_ADDR_RANDOM_INVALID,
  BLE_ADDR_RANDOM_STATIC,
};

struct ble_addr_s
{
  uint8_t addr[6];
  enum ble_addr_type_e type:1;
};

#define BLE_ADDR_FMT "[%s %02x:%02x:%02x:%02x:%02x:%02x]"
#define BLE_ADDR_ARG(x)                                                 \
  (x)->type == BLE_ADDR_RANDOM ? ble_addr_rand_name[(x)->addr[5] >> 6]  \
                               : "Public",                              \
  (x)->addr[5], (x)->addr[4], (x)->addr[3],                             \
  (x)->addr[2], (x)->addr[1], (x)->addr[0]

ALWAYS_INLINE
enum ble_addr_random_type_e ble_addr_random_type(const struct ble_addr_s *a)
{
  if (a->type != BLE_ADDR_RANDOM)
    return BLE_ADDR_RANDOM_INVALID;
  return a->addr[5] >> 6;
}

ALWAYS_INLINE
void ble_addr_random_type_set(struct ble_addr_s *a, enum ble_addr_random_type_e t)
{
  a->type = BLE_ADDR_RANDOM;
  a->addr[5] &= 0x3f;
  a->addr[5] |= t << 6;
}


ALWAYS_INLINE
uint8_t ble_addr_cmp(const struct ble_addr_s *a, const struct ble_addr_s *b)
{
  int8_t diff = (uint8_t)a->type - (uint8_t)b->type;

  if (diff)
    return diff;

  return memcmp(a->addr, b->addr, 6);
}

#endif
