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

#ifndef BLE_ATT_H_
#define BLE_ATT_H_

/**
   @file
   @module{BLE library}
   @short Attribute protocol network layer

   @section {Description}

   This implements the Attribute protocol for LE links.  This layer
   actually is a demo for typical Att handling.  Usual ATT
   implementation using a GATT DB backend is in @ref {gatt.h}.

   @end section
*/

#include <hexo/types.h>
#include <hexo/decls.h>
#include <net/task.h>
#include <ble/uuid.h>
#include <ble/protocol/att.h>

struct net_layer_s;
struct net_scheduler_s;

#define BLE_ATT_SERVER 1
#define BLE_ATT_CLIENT 0

enum ble_att_task_id_e
{
  BLE_ATT_REQUEST = 0xa1100000,
};

struct ble_att_transaction_s
{
  struct net_task_s task;

  /* Att generic info */
  enum ble_att_opcode_e command;
  uint8_t error;
  uint16_t error_handle;
};

STRUCT_COMPOSE(ble_att_transaction_s, task);

struct ble_att_information_s
{
  uint16_t handle;
  struct ble_uuid_s type;
};

/**
   A Find Information request sequence.

   @tt start, @tt end, @tt information and @tt information_max_count
   need to be set on query.

   @tt information_count must be reset before query.

   On response, @tt start is updated with next attribute to query
   from, and @tt information_count is set to count of pairs found.
 */
struct ble_att_find_information_task_s
{
  struct ble_att_transaction_s base;

  /* Query */
  uint16_t start, end;
  struct ble_att_information_s *information;
  uint16_t information_max_count;

  /* Response */
  uint16_t information_count;
};

STRUCT_COMPOSE(ble_att_find_information_task_s, base);

struct ble_att_handle_information_s
{
  uint16_t found;
  uint16_t end_group;
};

/**
   A Find By Type Value request sequence.

   @tt start, @tt end, @tt type, @tt value, @tt value_size, @tt
   information and @tt information_max_count need to be set on query.

   @tt information_count must be reset before query.

   On response, @tt start is updated with next attribute to query
   from, and @tt information_count is set to count of pairs found.
 */
struct ble_att_find_by_type_value_task_s
{
  struct ble_att_transaction_s base;

  /* Query */
  uint16_t start, end;
  struct ble_uuid_s type;
  uint8_t *value;
  uint8_t value_size;

  /* Return value */
  struct ble_att_handle_information_s *information;
  uint16_t information_max_count;

  /* Response */
  uint16_t information_count;
};

STRUCT_COMPOSE(ble_att_find_by_type_value_task_s, base);

struct ble_att_handle_value_s {
  uint16_t handle;
  uint8_t value[0];
};

/**
   A Read By Type request sequence.

   @tt start, @tt end, @tt type, @tt value_handle, @tt
   handle_value_size_max need to be set on query.

   @tt handle_value_size must be reset before query.

   On response, @tt start is updated with next attribute to query
   from, @tt handle_value_size is set to the total size of the @t
   handle_value array, and @tt handle_value_stride is set to offset
   from an element to the other.

   At most @tt handle_value_stride - 2 bytes of read data is filled in
   each @tt handle_value element.

   @tt handle_value_stride is invalid if less than 2, makes no sense
   if equal to 2.
 */
struct ble_att_read_by_type_task_s
{
  struct ble_att_transaction_s base;

  /* Query */
  uint16_t start, end;
  struct ble_uuid_s type;
  struct ble_att_handle_value_s *handle_value;
  uint16_t handle_value_size_max;

  /* Response */
  uint16_t handle_value_size;
  uint8_t handle_value_stride;
};

STRUCT_COMPOSE(ble_att_read_by_type_task_s, base);

/**
   A Read request sequence.

   @tt handle, @tt value, @tt value_size and @tt value_size_max need
   to be set on query.

   @tt value_size will be the offset for Read Blob requests.  Read
   data will be written to @tt{value + value_size}, up to @tt{value +
   value_size_max}

   On response, @tt value_size is set to actual value size.

   On request, responder can replace @tt value, @tt value_size and @tt
   value_size_max to point to constant data.  Actually returned data
   is @tt{data[value_size:value_size_max]}.
 */
struct ble_att_read_task_s
{
  struct ble_att_transaction_s base;

  /* Query */
  uint16_t handle;
  void *value;
  uint16_t value_size_max;

  /* Response */
  uint16_t value_size;
  uint16_t offset;
};

STRUCT_COMPOSE(ble_att_read_task_s, base);

/**
   A Read Multiple request sequence.

   @tt handle, @tt handle_count, @tt buffer, @tt buffer_size_max need
   to be set on query.

   @tt buffer_size must be reset before query.

   @tt buffer_size gets set in response.  It is up to the requester to
   know how to parse response buffer.
 */
struct ble_att_read_multiple_task_s
{
  struct ble_att_transaction_s base;

  /* Query */
  uint16_t *handle;
  void *buffer;
  uint16_t handle_count;
  uint16_t buffer_size_max;

  /* Response */
  uint16_t buffer_size;
};

STRUCT_COMPOSE(ble_att_read_multiple_task_s, base);

struct ble_att_data_s
{
  uint16_t handle, end_group;
  uint8_t value[0];
};

/**
   A Read By Group Type request sequence.

   @tt start, @tt end, @tt type, @tt attribute_data, and @tt
   attribute_data_size_max need to be set on query.

   @tt attribute_data_size must be reset before query.

   @tt start, @tt attribute_data_size and @tt attribute_data_stride
   gets set in response.
 */
struct ble_att_read_by_group_type_task_s
{
  struct ble_att_transaction_s base;

  /* Query */
  uint16_t start, end;
  struct ble_uuid_s type;
  struct ble_att_data_s *attribute_data;
  uint16_t attribute_data_size_max;

  /* Response */
  uint16_t attribute_data_stride;
  uint16_t attribute_data_size;
};

STRUCT_COMPOSE(ble_att_read_by_group_type_task_s, base);

/**
   A Write request.

   @tt handle, @tt value, @tt value_size need to be set on query.

   Task type may be:
   @list
   @item @tt BLE_ATT_WRITE_TASK,
   @item @tt BLE_ATT_WRITE_NO_RSP_TASK,
   @item @tt BLE_ATT_WRITE_PREPARED_TASK,
   @item @tt BLE_ATT_NOTIFICATION_TASK,
   @item @tt BLE_ATT_INDICATION_TASK.
   @end list
*/
struct ble_att_write_task_s
{
  struct ble_att_transaction_s base;

  /* Query */
  uint16_t handle;
  uint16_t value_size;
  void *value;
};

STRUCT_COMPOSE(ble_att_write_task_s, base);

#endif
