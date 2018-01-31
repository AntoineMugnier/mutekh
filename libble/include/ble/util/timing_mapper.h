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

#ifndef BLE_UTIL_TIMING_H_
#define BLE_UTIL_TIMING_H_

/**
   @file
   @module {Libraries::Bluetooth Low Energy}
   @short BLE connection timing mapper utility

   @this contains all declarations for the @ref {ble_timing_mapper_s}
   {Timing Mapper utility} object.
*/

#include <hexo/error.h>

#include <device/class/timer.h>

#include <ble/protocol/data.h>
#include <ble/protocol/advertise.h>

/**
   This utility object tracks timing of a running data connection.  It
   does not take any action about optimal timing policy, but simply
   keeps track of what time frame a given connection event takes place
   in.

   This can be used either by master of slave side of a connection.

   When used from the slave side, this utility takes window widening
   in consideration and updates timing reference to match master's.
*/
struct ble_timing_mapper_s
{
  struct device_timer_s timer;

  struct ble_conn_timing_param_s current, pending;

  dev_timer_value_t drops_at;
  dev_timer_value_t last_anchor;

  dev_timer_delay_t imprecision_tk;
  dev_timer_delay_t timeout_tk;
  dev_timer_delay_t pending_timeout_tk;
  dev_timer_delay_t min_conn_event_tk;

  uint16_t last_event;
  uint16_t update_instant;
  uint16_t pending_win_offset;
  uint16_t pending_win_size;

  uint8_t master_sca;
  bool_t update_pending : 1;
};

/**
   @this initiailzes a @ref {ble_timing_mapper_s} {Timing Mapper} with
   a timing reference and a connection request parameter set for the
   slave side of a connection.
 */
error_t ble_timing_mapper_slave_init(struct ble_timing_mapper_s *tm,
                                     struct device_timer_s *timer,
                                     const struct ble_adv_connect_s *connect,
                                     dev_timer_value_t reference);

/**
   @this initiailzes a @ref {ble_timing_mapper_s} {Timing Mapper} with
   a timing reference and a connection request parameter set for the
   master side of a connection.
 */
error_t ble_timing_mapper_master_init(struct ble_timing_mapper_s *tm,
                                      struct device_timer_s *timer,
                                      const struct ble_adv_connect_s *connect,
                                      dev_timer_value_t reference);

/**
   @this releases all data associated to a @ref {ble_timing_mapper_s}
   {Timing Mapper}.
 */
void ble_timing_mapper_cleanup(struct ble_timing_mapper_s *tm);

/**
   @this sets a connection event number @tt event as last successful
   at time @tt anchor.  This updates the internal timing reference and
   resets window widening.
 */
void ble_timing_mapper_event_set(struct ble_timing_mapper_s *tm,
                                 uint16_t event,
                                 dev_timer_value_t anchor);

/**
   @this retrieves the connection event window for the master side of
   a connection.

   @param begin Time the first packet must be sent on 
   @param max_duration Maximum time the connection event may last
 */
void ble_timing_mapper_window_master_get(struct ble_timing_mapper_s *tm,
                                         uint16_t event,
                                         dev_timer_value_t *begin,
                                         dev_timer_value_t *end,
                                         dev_timer_delay_t *max_duration);

/**
   @this retrieves the connection event window for the slave side of a
   connection.  After @tt end, slave may stop listening, no packet
   should be received after this deadline.  @tt max_duration is the
   time the connection event may last at most between the first packet
   and the last packet.

   @param begin Time the first packet may be received at, or later
   @param end Time the first packet may be received at, or before
   @param max_duration Maximum time the connection event may last
 */
void ble_timing_mapper_window_slave_get(struct ble_timing_mapper_s *tm,
                                        uint16_t event,
                                        dev_timer_value_t *begin,
                                        dev_timer_value_t *end,
                                        dev_timer_delay_t *max_duration);

/**
   @this pushes a connection parameters update as pending until a
   given instant.  If instant passed already, this function may return
   an error.
 */
error_t ble_timing_mapper_update_push(struct ble_timing_mapper_s *tm,
                                      const struct ble_conn_params_update *update);

#endif
