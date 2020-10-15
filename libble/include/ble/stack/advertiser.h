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

#ifndef BLE_STACK_ADVERTISER_H_
#define BLE_STACK_ADVERTISER_H_

/**
   @file
   @module {Bluetooth Low Energy library}
   @short Advertiser utility
*/

#include <hexo/types.h>

#include <gct_platform.h>
#include <gct/container_clist.h>

#include <mutek/buffer_pool.h>
#include <device/class/timer.h>

#include <ble/protocol/address.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/radio.h>
#include <ble/protocol/gap.h>

#include <ble/gatt/db.h>
#include <ble/stack/advertiser.h>
#include <ble/stack/peripheral.h>

#include <ble/security_db.h>

#include <ble/stack/runtime.h>

struct ble_advertiser_s;

enum ble_advertiser_state_e
{
  BLE_ADVERTISER_IDLE,
  BLE_ADVERTISER_CONNECTABLE,
};

struct ble_advertiser_handler_s
{
  void (*state_changed)(struct ble_advertiser_s *adv, bool_t advertising);
  void (*connection_request)(struct ble_advertiser_s *adv,
                             const struct ble_adv_connect_s *params,
                             dev_timer_value_t anchor);
};

struct ble_advertiser_s
{
  const struct ble_advertiser_handler_s *handler;
    struct runtime_s *runtime;

    struct device_ble_radio_s ble;
    struct device_timer_s rtc;
    struct device_crypto_s crypto;
    struct dev_ble_radio_rq_s adv;
};

error_t ble_advertiser_init(struct ble_advertiser_s *peri);
void ble_advertiser_addr_set(struct ble_advertiser_s *peri,
                             const struct ble_addr_s *addr);
void ble_advertiser_start(struct ble_advertiser_s *peri);
void ble_advertiser_stop(struct ble_advertiser_s *peri);

#endif
