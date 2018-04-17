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

#ifndef BLE_SNIFFER_ADVERTISE_H_
#define BLE_SNIFFER_ADVERTISE_H_

struct buffer;
struct ble_sniffer;
struct ble_address;

struct ble_advertise_handler
{
    void (*adv)(void *pvdata, uint8_t channel, struct buffer_s *packet);
    void (*done)(void *pvdata);
};

void ble_advertise_oberve(
    void *pvdata,
    const struct ble_advertise_handler *handler,
    struct ble_sniffer *radio);

#endif
