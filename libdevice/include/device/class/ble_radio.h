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

/**
   @file
   @module{Devices support library}
   @short Bluetooth Low Energy Radio driver class

   @section {Description}

   Bluetooth Low Energy Radio devices may be used to access radio for
   all BLE operations:
   @list
   @item RSSI Sampling,
   @item advertising, responding to scans, receiving connection requests,
   @item observing (passively listening for advertises),
   @item scanning,
   @item initiating a connection,
   @item handling master-side of a connection event,
   @item handling slave-side of a connection event,
   @item sniffing a connection event.
   @end list

   Each of these operations must be done through a request.  In
   particular, for connection-oriented operations, a single request
   handles a single connection event, not a whole connection.

   A request on BLE_Radio class is bounded in time.  It has a limit
   that may not be crossed in past and in future, and a maximum run
   time once started.

   In case of conflict, requests may not be honored.  Their precedence
   is computed in terms of importance.  @see dev_ble_radio_rq_type_e
   declaration order.

   Request holds a pointer to a @ref buffer_pool_s, this allows the
   device to allocate packets even if they eventually get dropped for
   whatever reason.

   Depending on request types, either single packets or packet queues
   are passed.  Packet queues and packet pointers must all be
   initialized on request.  Request handling by device never resets
   queues or pointers.

   For requests where an address field has a particular meaning in
   packets (scanning, connection), matching is performed on address
   contained in passed packets.

   Request timing is backed on a timer device in an
   implementation-defined way.  Most probably, device will also
   implement @tt timer class.

   Device also informs user about its own address (static or public,
   if any) and its packet size capabilities (prefix and MTU).

   Cryptography is not performed in the BLE_Radio class.  Packets must
   be handed to the radio device already ciphered.

   A request can be cancelled.  If request is actually being served
   when caller asks for cancellation, cancellation may fail.  Kroutine
   will be called normally in such cases.

   @end section
*/

#ifndef DEVICE_BLE_RADIO_H
#define DEVICE_BLE_RADIO_H

#include <hexo/types.h>
#include <device/request.h>
#include <device/class/timer.h>

#include <mutek/buffer_pool.h>

#include <ble/protocol/address.h>

struct device_ble_radio_s;
struct ble_addr_s;

enum dev_ble_radio_rq_type_e
{
    DEVICE_BLE_RADIO_RSSI_SAMPLE,
#if defined(CONFIG_BLE_PERIPHERAL)
    DEVICE_BLE_RADIO_ADV,
#endif
#if defined(CONFIG_BLE_CENTRAL)
    DEVICE_BLE_RADIO_ADV_LISTEN,
    DEVICE_BLE_RADIO_ADV_SCAN,
    DEVICE_BLE_RADIO_CONNECT,
    DEVICE_BLE_RADIO_DATA_MASTER,
#endif
#if defined(CONFIG_BLE_PERIPHERAL)
    DEVICE_BLE_RADIO_DATA_SLAVE,
#endif
#if defined(CONFIG_BLE_SNIFFER)
    DEVICE_BLE_RADIO_SNIFF,
#endif
    DEVICE_BLE_RADIO_COMMAND_COUNT,
    DEVICE_BLE_RADIO_COMMAND_MAX = DEVICE_BLE_RADIO_COMMAND_COUNT - 1,
};

enum dev_ble_radio_rq_status_e
{
    DEVICE_BLE_RADIO_IFS_TIMEOUT,
    DEVICE_BLE_RADIO_IN_PAST,
    DEVICE_BLE_RADIO_WINDOW_DONE,
    DEVICE_BLE_RADIO_HANDLER_DONE,
    DEVICE_BLE_RADIO_DEADLINE_MISSED,
};

struct dev_ble_radio_rq_s
{
  /* Inheritance */
  struct dev_request_s base;
  enum dev_ble_radio_rq_status_e status;

  /* Timing constraints */
  dev_timer_value_t not_before; /* 0 = ASAP */
  dev_timer_value_t not_after; /* 0 = Dont care */
  dev_timer_value_t max_duration; /* 0 = Dont care */

  /* Where to pick packet from */
  struct buffer_pool_s *packet_pool;

  /* Request parameters */
  enum dev_ble_radio_rq_type_e type;
  union {
    struct {
      uint8_t channel;
      int16_t rssi; // in .125dB steps
    } rssi_sample;

#if defined(CONFIG_BLE_PERIPHERAL)
    struct {
      // Packet to broadcast
      struct buffer_s *adv_packet;
      // Optional, may be left empty if no scan is possible
      struct buffer_s *scan_rsp_packet;
      // May be filled by connection request received back (if relevant)
      struct buffer_s *conn_packet;

      // Conn packet timestamp
      dev_timer_value_t anchor;
    } adv;
#endif

#if defined(CONFIG_BLE_CENTRAL)
    struct {
      // Reset on request
      buffer_queue_root_t received;

      uint8_t channel;
    } adv_listen;

    struct {
      // Must be filled with a scan packet and a target address,
      // target address will be extracted for filtering
      struct buffer_s *scan_packet;

      // May be filled by successful scan, must be empty on request
      struct buffer_s *adv_packet;
      struct buffer_s *scan_rsp_packet;

      uint8_t channel;

      bool_t rssi_sample;
      int16_t rssi; // in .125dB steps
    } adv_scan;

    struct {
      // Must be filled with a connection request packet (including
      // address), will be sent in response to a matching advertising
      // packet
      struct buffer_s *conn_packet;

      // Conn packet timestamp
      dev_timer_value_t anchor;

      uint8_t channel;
    } connect;
#endif

#if defined(CONFIG_BLE_CENTRAL) || defined(CONFIG_BLE_PERIPHERAL)
    struct {
      // Never reset.
      buffer_queue_root_t rx_queue;
      // Never reset.  Acknowledged packet are popped.
      buffer_queue_root_t tx_queue;
      // Never reset, in/out
      bool_t nesn, sn;
      // Whether peer has more data
      bool_t md;

      // Whether radio is allowed to close event if it has no packet
      // in tx queue.
      bool_t latency_permitted;

      bool_t rssi_sample;
      int16_t rssi; // in .125dB steps

      uint32_t access;
      uint32_t crc_init;
      uint8_t channel; // Already remapped

      // First packet timestamp
      dev_timer_value_t anchor;

      uint8_t packet_count;
      uint8_t tx_count;
      uint8_t tx_acked_count;
      uint8_t tx_empty_count;
      uint8_t rx_ok_count;
      uint8_t rx_invalid_count;
      uint8_t rx_repeated_count;
      uint8_t rx_empty_count;
    } data;
#endif

#if defined(CONFIG_BLE_SNIFFER)
    struct {
      // Never reset.
      buffer_queue_root_t rx_queue;

      uint32_t access;
      uint32_t crc_init;
      uint8_t channel;

      // First packet timestamp
      dev_timer_value_t anchor;

      bool_t rssi_sample;
      int16_t rssi; // in .125dB steps
    } sniff;
#endif
  };
};

struct ble_radio_info_s
{
  struct ble_addr_s address;
  uint16_t prefix_size;
  uint16_t mtu;
};

STRUCT_INHERIT(dev_ble_radio_rq_s, dev_request_s, base);

/* expand the dev_ble_radio_pqueue_insert function usable with generic
   device request priority queue. */
GCT_CONTAINER_KEY_TYPES(dev_request_pqueue, CUSTOM, SCALAR,
                        dev_ble_radio_rq_s_cast(dev_request_pqueue_item)->not_before, dev_ble_radio_pqueue);

GCT_CONTAINER_KEY_FCNS(dev_request_pqueue, ASC, inline, dev_ble_radio_pqueue, dev_ble_radio_pqueue,
                       remove, insert, next);


/**
   @this enqueues a request to the BLE radio.  Request may be refused
   if it happens in past. In such case, request is not enqueued and no
   kroutine will be called.
 */
#define DEVICE_BLE_RADIO_REQUEST(n)                                        \
  error_t (n)(struct device_ble_radio_s *accessor,                         \
              struct dev_ble_radio_rq_s *rq)

/** @This defines the prototype of the request function. */
typedef DEVICE_BLE_RADIO_REQUEST(device_ble_radio_request_t);

/**
   @this tries to cancel a request.

   If request is cancelled, its kroutine wont be called and @this
   returns 0.

   If request cannot be cancelled, request will proceed and call
   kroutine normally.
 */
#define DEVICE_BLE_RADIO_CANCEL(n)                                      \
  error_t (n)(struct device_ble_radio_s *accessor,                         \
              struct dev_ble_radio_rq_s *rq)

/** @This defines the prototype of the cancel function. */
typedef DEVICE_BLE_RADIO_CANCEL(device_ble_radio_cancel_t);

/**
   @this retrieves BLE radio information containing local preferred
   address and local data buffer prefix size.
 */
#define DEVICE_BLE_RADIO_GET_INFO(n)                                 \
  error_t (n)(struct device_ble_radio_s *accessor,                      \
              struct ble_radio_info_s *info)

/** @This defines the prototype of the get_info function. */
typedef DEVICE_BLE_RADIO_GET_INFO(device_ble_radio_get_info_t);

/** Driver types. */
DRIVER_CLASS_TYPES(ble_radio,
                   device_ble_radio_request_t *f_request;
                   device_ble_radio_cancel_t *f_cancel;
                   device_ble_radio_get_info_t *f_get_info;
                  );

#define DRIVER_BLE_RADIO_METHODS(prefix)                                \
  (&(const struct driver_ble_radio_s){                                  \
    .class_ = DRIVER_CLASS_BLE_RADIO,                                   \
    .f_request = prefix ## _request,                                    \
    .f_cancel = prefix ## _cancel,                                      \
    .f_get_info = prefix ## _get_info,                                  \
  })

#endif
