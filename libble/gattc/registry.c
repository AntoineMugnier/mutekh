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

error_t ble_gattc_char_read(struct ble_gattc_registry_s *reg,
                            uint8_t srv_instance, uint8_t char_index,
                            uint8_t char_instance)
{
}

error_t ble_gattc_char_subscribe(struct ble_gattc_registry_s *reg,
                                 uint8_t srv_instance, uint8_t char_index,
                                 uint8_t char_instance, bool_t acked)
{
}

error_t ble_gattc_char_write(struct ble_gattc_registry_s *reg,
                             uint8_t srv_instance, uint8_t char_index,
                             uint8_t char_instance,
                             const void *data, size_t size,
                             enum ble_gattc_subscription_mode_e mode)
{
}

error_t ble_gattc_register(struct ble_gattc_registry_s *reg,
                           struct ble_gattc_s *client,
                           const struct ble_gattc_service_s *srv)
{
  memset(reg, 0, sizeof(*reg));

  reg->desc = srv;
  ble_gattc_registry_list_pushback(&client->registry_list, reg);

  return 0;
}

error_t ble_gattc_unregister(struct ble_gattc_registry_s *reg)
{
  ble_gattc_registry_list_remove(&client->registry_list, reg);

  return 0;
}
