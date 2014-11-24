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

#include <hexo/endian.h>

#include <string.h>

#include <ble/uuid.h>

const struct ble_uuid_s bluetooth_base_uuid = BLE_UUID_SHORT(0);

bool_t ble_uuid_is_bluetooth_based(const struct ble_uuid_s *uuid)
{
  return !memcmp(bluetooth_base_uuid.value + 4, uuid->value + 4, 12);
}

bool_t ble_uuid_is_uuid16(const struct ble_uuid_s *uuid)
{
  return ble_uuid_is_bluetooth_based(uuid)
    && !endian_be16_na_load(uuid->value);
}

uint16_t ble_uuid_uuid16_get(const struct ble_uuid_s *uuid)
{
  return endian_be16_na_load(uuid->value + 2);
}

bool_t ble_uuid_is_uuid32(const struct ble_uuid_s *uuid)
{
  return ble_uuid_is_bluetooth_based(uuid);
}

uint32_t ble_uuid_uuid32_get(const struct ble_uuid_s *uuid)
{
  return endian_be32_na_load(uuid->value);
}

void ble_uuid_bluetooth_based(struct ble_uuid_s *uuid, uint32_t short_uuid)
{
  memcpy(uuid->value + 4, bluetooth_base_uuid.value + 4, 12);
  endian_be32_na_store(uuid->value, short_uuid);
}

uint8_t ble_uuid_cmp(const struct ble_uuid_s *a, const struct ble_uuid_s *b)
{
  return memcmp(a, b, 16);
}
