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

const struct ble_uuid_s bluetooth_base_uuid = BLE_UUID_BT_BASED(0);

bool_t ble_uuid_is_bluetooth_based(const struct ble_uuid_s *uuid)
{
  return !memcmp(bluetooth_base_uuid.value, uuid->value, 12);
}

bool_t ble_uuid_is_uuid32(const struct ble_uuid_s *uuid)
{
  return ble_uuid_is_bluetooth_based(uuid);
}

uint32_t ble_uuid_uuid32_get(const struct ble_uuid_s *uuid)
{
  return endian_le32_na_load(uuid->value + 12);
}

bool_t ble_uuid_is_uuid16(const struct ble_uuid_s *uuid)
{
  return ble_uuid_is_bluetooth_based(uuid)
    && ble_uuid_uuid32_get(uuid) < 0x10000;
}

uint16_t ble_uuid_uuid16_get(const struct ble_uuid_s *uuid)
{
  return endian_le16_na_load(uuid->value + 12);
}

void ble_uuid_bluetooth_based(struct ble_uuid_s *uuid, uint32_t short_uuid)
{
  memcpy(uuid->value, bluetooth_base_uuid.value, 12);
  endian_le32_na_store(uuid->value + 12, short_uuid);
}

int8_t ble_uuid_cmp(const struct ble_uuid_s *a, const struct ble_uuid_s *b)
{
  return memcmp(a, b, 16);
}
