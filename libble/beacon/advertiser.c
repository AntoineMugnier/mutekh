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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#include <hexo/endian.h>
#include <mutek/mem_alloc.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/ble_radio.h>
#include <device/class/timer.h>

#include <mutek/buffer_pool.h>
#include <ble/protocol/address.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/company.h>
#include <ble/protocol/gap.h>

#include <ble/beacon/advertiser.h>

#include <stdlib.h>

#define ADV_BEFORE_RENEW (1000 / 50 * 60)

struct ble_beacon_advertiser_s
{
  struct buffer_pool_s packet_pool;
  struct device_ble_radio_s ble;
  struct device_timer_s rtc;
  struct dev_ble_radio_rq_s adv;
  struct ble_addr_s addr;
  uint16_t adv_before_renew;
  struct ble_beacon_config_s config;
  bool_t running;
};

static void beacon_destroy(struct ble_beacon_advertiser_s *beacon);

__attribute__((__packed__))
struct beacon_data_s
{
  uint16_t vendor;
  uint8_t type;
  uint8_t len;
  struct ble_uuid_s uuid;
  uint16_t major, minor;
  uint8_t rssi;
};

static void adv_send(struct ble_beacon_advertiser_s *beacon);

static KROUTINE_EXEC(adv_done)
{
  struct ble_beacon_advertiser_s *beacon
    = KROUTINE_CONTAINER(kr, *beacon, adv.base.kr);

  if (beacon->adv.adv.conn_packet)
    buffer_refdec(beacon->adv.adv.conn_packet);
  beacon->adv.adv.conn_packet = NULL;

  adv_send(beacon);
}

static void rand_addr(struct ble_addr_s *addr)
{
  for (uint8_t i = 0; i < 6; ++i)
    addr->addr[i] = random();

  addr->type = BLE_ADDR_RANDOM;
  addr->addr[5] &= 0x3f;
  addr->addr[5] |= BLE_ADDR_RANDOM_NON_RESOLVABLE << 6;
}

static dev_timer_value_t random_future_time(
    struct ble_beacon_advertiser_s *beacon)
{
    dev_timer_value_t now;

    DEVICE_OP(&beacon->rtc, get_value, &now, 0);

    return now + 32768 + (random() & 0x1ff) - 256;
}

static void adv_init(struct ble_beacon_advertiser_s *beacon)
{
  struct buffer_s *adv;

  if (!beacon->adv.adv.adv_packet)
    beacon->adv.adv.adv_packet = buffer_pool_alloc(&beacon->packet_pool);

  adv = beacon->adv.adv.adv_packet;
  beacon->adv.adv.scan_rsp_packet = NULL;

  ble_adv_ind_set(adv, &beacon->addr);

  static const uint8_t appearance = 0x05;
  ble_adv_data_append(adv, BLE_GAP_FLAGS, &appearance, 1);

  struct beacon_data_s data;
  data.vendor = endian_le16(BLE_COMPANY_APPLE);
  data.type = 2;
  data.len = 16 + 2 + 2 + 1;
  data.uuid = beacon->config.group_uuid;
  data.major = endian_le16(beacon->config.major);
  data.minor = endian_le16(beacon->config.minor);
  data.rssi = beacon->config.one_meter_rssi;

  ble_adv_data_append(adv, BLE_GAP_MANUFACTURER_SPECIFIC_DATA,
                      &data, sizeof(data));

  beacon->adv_before_renew = ADV_BEFORE_RENEW;
}

static void adv_send(struct ble_beacon_advertiser_s *beacon)
{
  if (!beacon->running)
    return beacon_destroy(beacon);

  kroutine_init(&beacon->adv.base.kr, adv_done, KROUTINE_INTERRUPTIBLE);

  if (beacon->adv_before_renew == 0) {
    rand_addr(&beacon->addr);
    adv_init(beacon);
  }

  beacon->adv_before_renew--;

  dev_timer_value_t begin = random_future_time(beacon);
  beacon->adv.not_before = begin;
  beacon->adv.not_after = begin + 100;
  beacon->adv.max_duration = 0;
  beacon->adv.type = DEVICE_BLE_RADIO_ADV;

  beacon->adv.packet_pool = &beacon->packet_pool;
  beacon->adv.adv.conn_packet = NULL;

  DEVICE_OP(&beacon->ble, request, &beacon->adv);
}

static void beacon_destroy(struct ble_beacon_advertiser_s *beacon)
{
  buffer_pool_cleanup(&beacon->packet_pool);
  device_put_accessor(&beacon->rtc);
  device_put_accessor(&beacon->ble);
  mem_free(beacon);
}

void ble_beacon_destroy(struct ble_beacon_advertiser_s *beacon)
{
  beacon->running = 0;
}

static SLAB_GROW(packet_pool_grow)
{
  return 10;
}

error_t ble_beacon_create(const char *ble_dev,
                          const struct ble_beacon_config_s *config,
                          struct ble_beacon_advertiser_s **ret)
{
  struct ble_beacon_advertiser_s *beacon;
  error_t err;

  beacon = mem_alloc(sizeof(*beacon), mem_scope_sys);
  if (!beacon)
    return -ENOMEM;

  memset(beacon, 0, sizeof(*beacon));

  err = device_get_accessor_by_path(&beacon->ble, NULL,
                                    ble_dev, DRIVER_CLASS_BLE_RADIO);
  if (err)
    goto err_free;

  err = device_get_accessor_by_path(&beacon->rtc, NULL,
                                    ble_dev, DRIVER_CLASS_TIMER);
  if (err)
    goto err_release;

  device_start(&beacon->ble);
  buffer_pool_init(&beacon->packet_pool, CONFIG_BLE_PACKET_SIZE,
                   packet_pool_grow, mem_scope_sys);

  beacon->running = 1;
  beacon->config = *config;

  adv_send(beacon);

  *ret = beacon;

  return 0;

 err_release:
  device_put_accessor(&beacon->ble);
 err_free:
  mem_free(beacon);

  return err;
}
