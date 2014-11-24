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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#include <ble/protocol/company.h>

const char *ble_company_name(uint16_t cid)
{
  switch (cid) {
  case BLE_COMPANY_TI: return "Ti";
  case BLE_COMPANY_BROADCOM: return "Bcm";
  case BLE_COMPANY_ATHEROS: return "Ath";
  case BLE_COMPANY_APPLE: return "Apple";
  case BLE_COMPANY_NORDIC: return "Nordic";
  case BLE_COMPANY_SAMSUNG: return "Samsung";
  }

  return "Unknown";
}
