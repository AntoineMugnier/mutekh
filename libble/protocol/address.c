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

#include <ble/protocol/address.h>
#include <net/addr.h>

const char *const ble_addr_rand_name[4] = {
    "Non resolvable", "Resolvable", "Invalid", "Static"
};

void ble_addr_net_parse(struct ble_addr_s *addr, const struct net_addr_s *naddr)
{
  memrevcpy(addr->addr, naddr->mac, 6);
  addr->type = naddr->random_addr ? BLE_ADDR_RANDOM : BLE_ADDR_PUBLIC;
}

void ble_addr_net_set(const struct ble_addr_s *addr, struct net_addr_s *naddr)
{
  memrevcpy(naddr->mac, addr->addr, 6);
  naddr->random_addr = addr->type == BLE_ADDR_RANDOM;
}
