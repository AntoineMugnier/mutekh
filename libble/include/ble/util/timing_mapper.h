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
   @module{BLE library}
   @short Bluetooth LE timing mapper

   @section {Description}

   This utility tracks timing mapping of a running data connection.

   @end section
*/

#include <hexo/error.h>

#include <device/class/timer.h>

#include <ble/protocol/data.h>
#include <ble/protocol/advertise.h>

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

error_t ble_timing_mapper_init(struct ble_timing_mapper_s *tm,
                               struct device_timer_s *timer,
                               const struct ble_adv_connect_s *connect,
                               dev_timer_value_t reference);

void ble_timing_mapper_cleanup(struct ble_timing_mapper_s *tm);

void ble_timing_mapper_event_set(struct ble_timing_mapper_s *tm,
                                 uint16_t event,
                                 dev_timer_value_t anchor);

void ble_timing_mapper_window_get(struct ble_timing_mapper_s *tm,
                                  uint16_t event,
                                  dev_timer_value_t *begin,
                                  dev_timer_value_t *end,
                                  dev_timer_delay_t *max_duration);

void ble_timing_mapper_window_slave_get(struct ble_timing_mapper_s *tm,
                                        uint16_t event,
                                        dev_timer_value_t *begin,
                                        dev_timer_value_t *end,
                                        dev_timer_delay_t *max_duration);

error_t ble_timing_mapper_update_push(struct ble_timing_mapper_s *tm,
                                      const struct ble_conn_params_update *update);

#endif
