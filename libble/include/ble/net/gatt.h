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

#ifndef BLE_GATT_H_
#define BLE_GATT_H_

/**
   @file
   @module{BLE library}
   @short GATT protocol layer

   @section {Description}

   This implements the Attribute protocol for LE links backed by a
   GATT database.

   @end section
*/

#include <hexo/types.h>
#include <hexo/decls.h>
#include <net/scheduler.h>

#include <ble/gatt/client.h>
#include <ble/protocol/att.h>
#include <ble/peer.h>

/**
 BLE GATT protocol layer.  This actually implements an ATT layer
 backed to a GATT Db.
 */
struct ble_gatt_s
{
  struct net_layer_s layer;
  struct ble_gatt_client_s client;
  struct ble_peer_s *peer;
  const struct ble_gatt_handler_s *handler;
  struct net_task_s *delayed_client_update;

  uint16_t server_mtu;
};

STRUCT_COMPOSE(ble_gatt_s, layer);
STRUCT_COMPOSE(ble_gatt_s, client);

struct ble_gatt_service_desc_s
{
  /** Service type */
  struct ble_uuid_s type;

  /** Service handle */
  uint16_t handle;

  /** Service last handle, inclusive */
  uint16_t group_end_handle;
};

struct ble_gatt_find_services_s
{
  struct net_task_s task;

  /**
     Range to search chars in, inclusive.  On call return, @tt start
     is updated to be next attribute handle unscanned to date.
  */
  uint16_t start, end;

  /**
     Error returned by server.
     0 if overflow of @tt service_count.
   */
  enum ble_att_error_e error;

  /**
     True if overflow of @tt service_count.
   */
  bool_t more;

  /**
     Array of returned services, caller allocated.
   */
  struct ble_gatt_service_desc_s *service;
  /**
     Size of @tt services array.
   */
  uint16_t service_max_count;
  /**
     Actual returned size of @tt service array.
   */
  uint16_t service_count;
};

struct ble_gatt_char_desc_s
{
  /** Characteristic type */
  struct ble_uuid_s type;

  /** Characteristic handle */
  uint16_t handle;
};

struct ble_gatt_find_chars_s
{
  struct net_task_s task;

  /**
     Range to search chars in, inclusive.  On call return, @tt start
     is updated to be next attribute handle unscanned to date.
  */
  uint16_t start, end;

  /**
     Error returned by server.
     0 if overflow of @tt service_count.
   */
  enum ble_att_error_e error;

  /**
     True if overflow of @tt service_count.
   */
  bool_t more;

  /**
     Array of returned chars, caller allocated.
   */
  struct ble_gatt_char_desc_s *chars;
  /**
     Size of @tt chars array.
   */
  uint16_t chars_max_count;
  /**
     Actual returned size of @tt chars array.
   */
  uint16_t chars_count;
};

struct ble_gatt_params_s
{
  struct ble_peer_s *peer;
  struct ble_gatt_db_s *db;
};

#endif
