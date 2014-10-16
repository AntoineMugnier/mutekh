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

#ifndef NRF51_BLE_RADIO_HANDLER_H_
#define NRF51_BLE_RADIO_HANDLER_H_

#include <hexo/types.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/class/icu.h>
#include <device/class/clock.h>
#include <device/class/timer.h>
#include <device/class/ble_radio.h>

#include <arch/nrf51/radio.h>

struct buffer_s;

enum adv_state {
  ADV_IND,
  ADV_SCAN_REQ,
  ADV_SCAN_RSP,
  ADV_DONE,
};

enum conn_state {
  CONN_ADV,
  CONN_SEND,
  CONN_SENT,
  CONN_DONE,
};

enum transfer_mode {
  MODE_TX,
  MODE_RX,
  MODE_RSSI,
};

struct ble_radio_params
{
  uint32_t access;
  uint32_t crc_init;
  uint8_t channel;
  enum transfer_mode mode;
  bool_t rx_rssi;
  bool_t ifs;
};

struct ble_radio
{
  struct dev_irq_ep_s irq_source[NRF51_BLE_RADIO_IRQ_COUNT];

#if defined(CONFIG_DEVICE_CLOCK)
  struct dev_clock_sink_ep_s clock_sink[NRF51_BLE_RADIO_CLK_COUNT];

  bool_t accurate_clock_requested;
  bool_t accurate_clock_running;
#endif

  dev_request_pqueue_root_t queue;

  struct dev_freq_accuracy_s sleep_acc;

  struct dev_ble_radio_rq_s *current;
  struct dev_ble_radio_rq_s *first;
  struct dev_ble_radio_rq_s *next;
  const struct ble_radio_request_handler *handler;

  struct ble_radio_params radio_current, radio_next;
  struct buffer_s *transmitting;

  dev_timer_value_t start;
  dev_timer_value_t end;

  dev_timer_value_t base;
  dev_timer_value_t last_now;
  dev_timer_value_t address_timestamp;

  int16_t last_rx_rssi;
  bool_t callbacking;
  uint32_t use_count;

  union {
    struct {
      uint8_t channel;
      enum adv_state state;
    } adv;

    struct {
      enum conn_state state;
    } conn;

    struct {
      bool_t done;
      bool_t last_rx_ok;
    } data;
  } state;
};

struct ble_radio_request_handler
{
  /**
    Setup current request context. May be NULL.

     @param radio BLE Radio context
   */
  void (*startup)(struct ble_radio *radio);

  /**
    Setup current request context. May be NULL.

     @param radio BLE Radio context
   */
  void (*cleanup)(struct ble_radio *radio);

  /**
     Retrieves radio parameters as estimated now. May be called more
     than once between two packets.  When called twice between two
     packet and radio parameters are changed in-between, T_IFS may not
     be enforced between current and subsequent packet.

     @param radio BLE Radio context
     @param params Radio parameters to setup for next packet
     @returns whether request is to be continued, else it is
              callbacked as done.
  */
  bool_t (*radio_params)(const struct ble_radio *radio,
                         struct ble_radio_params *params);

  /**
     Retrieve packet to send on next tx.

     May be NULL for requests not transmitting packets.

     @param radio BLE Radio context
     @returns a packet reference, must not be NULL
   */
  struct buffer_s *(*payload_get)(const struct ble_radio *radio);

  /**
     After (T_IFS + preamble time), we may decide whether we had an
     IFS timeout or a packet got received.  This is called to indicate
     whether a packet is transferred or a timeout occurred.

     If a packet is being transferred, @this should assume packet
     radio is currently transmitting is the right one.  It should
     update internal request handler state in an optimistic way in
     order to be able to reconfigure the radio.  A call to @tt
     radio_params issued after this call should return radio
     parameters of next expected transfer, even if this would change
     after analyzing payload.

     This is also called on TX with @tt rx_timeout set to @tt false.

     May be NULL.

     @param radio BLE Radio context
   */
  void (*ifs_event)(struct ble_radio *radio,
                    bool_t rx_timeout);

  /**
     After payload is received and CRC has been checked, it is handed
     over to the request.  At that time, radio is already
     reconfiguring for next packet's radio parameters.  Nevertheless,
     internal state may be updated and radio parameters changed, in
     case they will be aborted and reconfigured, in such case, T_IFS
     may not be enforced between transfers.

     May be NULL for requests not receiving packets.

     @param radio BLE Radio context
     @param packet Received packet, callee must take reference if wanted.
   */
  void (*payload_received)(struct ble_radio *radio,
                           bool_t crc_valid,
                           struct buffer_s *packet);
};

struct buffer_s *nrf51_ble_radio_packet_alloc(const struct ble_radio *pv);

extern const struct ble_radio_request_handler
request_handler[DEVICE_BLE_RADIO_COMMAND_MAX + 1];

#endif
