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
   @module {Bluetooth Low Energy library}
   @short Network layer definition for Attribute Protocol (ATT)

   This header defines the Network Layer API for Attribute Protocol
   (ATT).

   This layer handles protocol encoding specificities and exposes a
   request-based asynchronous API.

   There may be one layer registered as @ref {#BLE_ATT_SERVER}
   {server}, one as @ref {#BLE_ATT_CLIENT} {client}.  Unacknowledged
   ATT operations are forwarded as simple packets with relevant
   attribute id set in address.

   When attribute layer receives a data packet from upper layers, it
   is converted either to a notification packet (if attribute id is
   set in source address) or a write without response packet (if
   attribute id is set in destination address). They are unambiguously
   forwarded to children layers:
   @list
   @item an attribute notification / indication packet is forwarded to
   registered client layer,
   @item an attribute write without response packet is forwarded to
   registered server layer.
   @item a packet received from registered client layer is forwarded
   as write without response,
   @item a packet received from registered server layer is forwarded
   as attribute notification / indication,
   @end list

   Other layers not parented to ATT layer directly may act as client
   and request any operation to peer's server through this layer.  ATT
   layer will respect sequential aspect of the Attribute protocol.

   There is a generic implementation of this layer in the library that
   can be created through @ref ble_att_create.
*/

#include <hexo/types.h>
#include <hexo/decls.h>
#include <net/task.h>
#include <ble/uuid.h>
#include <ble/protocol/att.h>

struct net_layer_s;
struct net_scheduler_s;

/** Layer type to pass for binding a child layer as server.  Server
    layer must respond for all queries, and is the only layer able to
    generate attribute value notifications / indications. */
#define BLE_ATT_SERVER 1
/** Layer type to pass for binding a child layer as client.  Client
 layer explicitly bound to att layer is unique and receives attribute
 value notifications / indications.  Other client layers need not be
 bound and may only use synchronous requests. */
#define BLE_ATT_CLIENT 0

/** ID for task holding all ATT-based operations, to be used in @ref
    {ble_att_transaction_s::task::query::opcode}. */
#define BLE_ATT_REQUEST 0xa1100000

struct ble_att_information_s
{
  uint16_t handle;
  struct ble_uuid_s type;
};

struct ble_att_handle_information_s
{
  uint16_t found;
  uint16_t end_group;
};

struct ble_att_handle_value_s
{
  uint16_t handle;
  uint8_t value[0];
};

struct ble_att_data_s
{
  uint16_t handle, end_group;
  uint8_t value[0];
};

/**
   @this inherits @ref net_task_s and holds information contained in a
   request / response transaction at the Attribute Protocol level.
 */
struct ble_att_transaction_s
{
  struct net_task_s task;

  /* Att generic info */
  enum ble_att_opcode_e command;
  uint8_t error;
  uint16_t error_handle;

  /** Buffer reference associated with this transaction, if any */
  struct buffer_s *packet;

  union {
    /**
       A Find Information request sequence.

       @tt start, @tt end, @tt information and @tt information_max_count
       need to be set on query.

       @tt information_count must be reset before query.

       On response, @tt start is updated with next attribute to query
       from, and @tt information_count is set to count of pairs found.
    */
    struct {
      /* Query */
      uint16_t start, end;
      struct ble_att_information_s *information;
      uint16_t information_max_count;

      /* Response */
      uint16_t information_count;
    } find_information;

    /**
       A Find By Type Value request sequence.

       @tt start, @tt end, @tt type, @tt value, @tt value_size, @tt
       information and @tt information_max_count need to be set on query.

       @tt information_count must be reset before query.

       On response, @tt start is updated with next attribute to query
       from, and @tt information_count is set to count of pairs found.
    */
    struct {
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
    } find_by_type_value;

    /**
       A Read By Type request sequence.

       @tt start, @tt end, @tt type, @tt value_handle, @tt
       handle_value_size_max need to be set on query.

       @tt handle_value_size must be reset before query.

       On response, @tt start is updated with next attribute to query
       from, @tt handle_value_size is set to the total size of the @tt
       handle_value array, and @tt handle_value_stride is set to offset
       from an element to the other.

       At most @tt handle_value_stride - 2 bytes of read data is filled in
       each @tt handle_value element.

       @tt handle_value_stride is invalid if less than 2, makes no sense
       if equal to 2.
    */
    struct {
      /* Query */
      uint16_t start, end;
      struct ble_uuid_s type;
      struct ble_att_handle_value_s *handle_value;
      uint16_t handle_value_size_max;

      /* Response */
      uint16_t handle_value_size;
      uint8_t handle_value_stride;
    } read_by_type;

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
    struct {
      /* Query */
      uint16_t handle;
      void *value;
      uint16_t value_size_max;

      /* Response */
      uint16_t value_size;
      uint16_t offset;
    } read;

    /**
       A Read Multiple request sequence.

       @tt handle, @tt handle_count, @tt buffer, @tt buffer_size_max need
       to be set on query.

       @tt buffer_size must be reset before query.

       @tt buffer_size gets set in response.  It is up to the requester to
       know how to parse response buffer.
    */
    struct {
      /* Query */
      uint16_t *handle;
      void *buffer;
      uint16_t handle_count;
      uint16_t buffer_size_max;

      /* Response */
      uint16_t buffer_size;
    } read_multiple;

    /**
       A Read By Group Type request sequence.

       @tt start, @tt end, @tt type, @tt attribute_data, and @tt
       attribute_data_size_max need to be set on query.

       @tt attribute_data_size must be reset before query.

       @tt start, @tt attribute_data_size and @tt attribute_data_stride
       gets set in response.
    */
    struct {
      /* Query */
      uint16_t start, end;
      struct ble_uuid_s type;
      struct ble_att_data_s *attribute_data;
      uint16_t attribute_data_size_max;

      /* Response */
      uint16_t attribute_data_stride;
      uint16_t attribute_data_size;
    } read_by_group_type;


    /**
       A Write request.

       @tt handle, @tt value, @tt value_size need to be set on query.

       Task type may be:
       @list
       @item @tt BLE_ATT_WRITE_TASK,
       @item @tt BLE_ATT_WRITE_PREPARED_TASK,
       @item @tt BLE_ATT_INDICATION_TASK.
       @end list
    */
    struct {
      /* Query */
      uint16_t handle;
      uint16_t value_size;
      void *value;

      bool_t authenticated;
      bool_t encrypted;
    } write;
  };
};

STRUCT_COMPOSE(ble_att_transaction_s, task);

#endif
