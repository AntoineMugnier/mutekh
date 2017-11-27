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

#ifndef BLE_PROTOCOL_COMPANY_H
#define BLE_PROTOCOL_COMPANY_H

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Bluetooth Company identifiers
 */

#include <hexo/types.h>

/**
   Company name lookup function. @ref ble_company.
 */
const char *ble_company_name(uint16_t cid);

/**
   Bluetooth company ID sparse list.
 */
enum ble_company {
  BLE_COMPANY_TI = 0x000d,
  BLE_COMPANY_BROADCOM = 0x000f,
  BLE_COMPANY_QUALCOMM = 0x001d,
  BLE_COMPANY_ATHEROS = 0x0045,
  BLE_COMPANY_APPLE = 0x004c,
  BLE_COMPANY_NORDIC = 0x0059,
  BLE_COMPANY_SAMSUNG = 0x0075,
};

#endif
