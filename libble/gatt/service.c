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

#include <ble/gatt/service.h>
#include <ble/gatt/db.h>
#include <ble/gatt/client.h>

#include <ble/protocol/gatt.h>
#include <ble/protocol/att.h>
#include <ble/uuid.h>

#include <stdlib.h>

enum ble_att_error_e ble_gatt_db_char_write(struct ble_gatt_client_s *client,
                                            struct ble_gatt_db_service_s *service,
                                            uint8_t charid,
                                            const void *data, size_t size)
{
  enum ble_att_error_e err;

  if (service->service->characteristic_count <= charid)
    return BLE_ATT_ERR_INVALID_HANDLE;

  const struct ble_gatt_characteristic_s *chr = &service->service->characteristic[charid];

  if (!(chr->permissions & (BLE_GATT_PERM_OTHER_WRITE | BLE_GATT_PERM_ENC_WRITE | BLE_GATT_PERM_AUTH_WRITE)))
    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;

  uint16_t perms = BLE_GATT_PERM_OTHER_WRITE;
  if (client->encrypted)
    perms |= BLE_GATT_PERM_ENC_WRITE;
  if (client->authenticated)
    perms |= BLE_GATT_PERM_AUTH_WRITE;

  if (!(perms & chr->permissions))
    return BLE_ATT_ERR_INSUF_AUTHENTICATION;

  switch (chr->mode) {
  case BLE_GATT_CHARACTERISTIC_CONSTANT:
    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;

  case BLE_GATT_CHARACTERISTIC_PLAIN:
    if (chr->data.plain.size != size)
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    memcpy(chr->data.plain.data, data, size);
    if (chr->data.plain.on_changed)
      chr->data.plain.on_changed(client, service, charid);
    ble_gatt_db_char_changed(service, charid, 1, data, size);
    return 0;

  case BLE_GATT_CHARACTERISTIC_DYNAMIC:
    err = chr->data.dynamic.on_write(client, service, charid, data, size);
    if (err == 0)
      ble_gatt_db_char_changed(service, charid, 1, data, size);
    return err;

#if defined(CONFIG_BLE_ATT_LONG_WRITE)
  case BLE_GATT_CHARACTERISTIC_DYNAMIC_PREPARED:
# error implement me
#endif

  case BLE_GATT_CHARACTERISTIC_RAW:
#warning TODO
    return BLE_ATT_ERR_REQUEST_NOT_SUPPORTED;
  }

  return BLE_ATT_ERR_UNLIKELY_ERROR;
}

enum ble_att_error_e ble_gatt_db_char_read(struct ble_gatt_client_s *client,
                                           struct ble_gatt_db_service_s *service,
                                           uint8_t charid, size_t offset,
                                           void *data, size_t *size)
{
  if (service->service->characteristic_count <= charid)
    return BLE_ATT_ERR_INVALID_HANDLE;

  ssize_t transferred;
  const struct ble_gatt_characteristic_s *chr = &service->service->characteristic[charid];

  if (!(chr->permissions & (BLE_GATT_PERM_OTHER_READ | BLE_GATT_PERM_ENC_READ | BLE_GATT_PERM_AUTH_READ)))
    return BLE_ATT_ERR_READ_NOT_PERMITTED;

  uint16_t perms = BLE_GATT_PERM_OTHER_READ;
  if (client->encrypted)
    perms |= BLE_GATT_PERM_ENC_READ;
  if (client->authenticated)
    perms |= BLE_GATT_PERM_AUTH_READ;

  if (!(perms & chr->permissions))
    return BLE_ATT_ERR_INSUF_AUTHENTICATION;

  switch (chr->mode) {
  case BLE_GATT_CHARACTERISTIC_CONSTANT:
    transferred = __MIN(__MAX((ssize_t)(chr->data.constant.size - offset), 0), *size);
    memcpy(data, chr->data.constant.data + offset, transferred);
    *size = transferred;
    return 0;

  case BLE_GATT_CHARACTERISTIC_PLAIN:
    transferred = __MIN(__MAX((ssize_t)(chr->data.plain.size - offset), 0), *size);
    memcpy(data, chr->data.plain.data + offset, transferred);
    *size = transferred;
    return 0;

  case BLE_GATT_CHARACTERISTIC_DYNAMIC:
    return chr->data.dynamic.on_read(client, service, charid, offset, data, size);

#if defined(CONFIG_BLE_GATT_DYNAMIC)
  case BLE_GATT_CHARACTERISTIC_DYNAMIC_PREPARED:
# error implement me
#endif

  case BLE_GATT_CHARACTERISTIC_RAW:
#warning TODO
    return BLE_ATT_ERR_REQUEST_NOT_SUPPORTED;
  }

  return BLE_ATT_ERR_UNLIKELY_ERROR;
}
