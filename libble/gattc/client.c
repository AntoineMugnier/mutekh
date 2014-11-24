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

#include <ble/gattc/client.h>
#include <ble/gattc/registry.h>
#include <ble/gattc/service.h>

error_t ble_gattc_init(struct ble_gattc_s *client)
{
  ble_gattc_registry_list_init(&client->registry_list);
}

void ble_gattc_deinit(struct ble_gattc_s *client)
{
  ble_gattc_registry_list_destroy(&client->registry_list);
}

size_t ble_gattc_sollicitation_16_get(struct ble_gattc_s *client,
                                      void *srvlist_, size_t size)
{
  uintptr_t srvlist = (uintptr_t)srvlist_;
  size_t off = 0;

  GCT_FOREACH(ble_gattc_registry_list, &client->registry_list, reg,
              if (!(reg->desc->flags & BLE_GATTC_SERVICE_SOLLICIT))
                GCT_FOREACH_CONTINUE;

              if (!ble_uuid_is_uuid16(reg->desc->type))
                GCT_FOREACH_CONTINUE;

              if (off + 2 <= size)
                endian_le16_na_store((void*)(srvlist + off),
                                     ble_uuid_uuid16_get(reg->desc->type));

              off += 2;
              );

  return off;
}

size_t ble_gattc_sollicitation_128_get(struct ble_gattc_s *client,
                                       void *srvlist_, size_t size)
{
  uintptr_t srvlist = (uintptr_t)srvlist_;
  size_t off = 0;

  GCT_FOREACH(ble_gattc_registry_list, &client->registry_list, reg,
              if (!(reg->desc->flags & BLE_GATTC_SERVICE_SOLLICIT))
                GCT_FOREACH_CONTINUE;

              if (ble_uuid_is_uuid16(reg->desc->type))
                GCT_FOREACH_CONTINUE;

              if (off + 16 <= size)
                memcpy((void*)(srvlist + off), (const void *)reg->desc->type, 16);
              off += 16;
              );

  return off;
}
