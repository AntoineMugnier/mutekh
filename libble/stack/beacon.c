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

#define LOGK_MODULE_ID "beac"

#include <ble/stack/context.h>
#include <ble/stack/beacon.h>
#include <ble/net/adv.h>
#include <ble/protocol/company.h>
#include <ble/protocol/gap.h>

#include <ble/net/layer_id.h>

#include <mutek/printk.h>

__attribute__((__packed__))
struct beacon_data_s
{
  uint16_t vendor;
  uint8_t type;
  uint8_t len;
  struct ble_uuid_s group;
  uint16_t major, minor;
  uint8_t rssi;
};

struct ble_beacon_adv_config_s
{
  struct ble_advertiser_param_s adv_params;
  uint8_t ad[30];
};

#define BEACON_AD_SIZE (sizeof(struct beacon_data_s) + 2)

static
void ble_beacon_config_apply(struct ble_beacon_adv_config_s *adv_config,
                             const struct ble_beacon_config_s *config)
{
  adv_config->adv_params.phy = config->phy;
  adv_config->adv_params.local_addr = config->local_addr;
  adv_config->adv_params.interval_ms = config->interval_ms;
  adv_config->adv_params.delay_max_ms = config->interval_ms / 10;
  adv_config->adv_params.connectable = 0;
  adv_config->adv_params.ad = adv_config->ad;
  adv_config->adv_params.ad_len = 30;

  adv_config->ad[0] = 2;
  adv_config->ad[1] = BLE_GAP_FLAGS;
  adv_config->ad[2] = BLE_GAP_FLAGS_BREDR_NOT_SUPPORTED | BLE_GAP_FLAGS_GENERAL_ADV;

  adv_config->ad[3] = 26;
  adv_config->ad[4] = BLE_GAP_MANUFACTURER_SPECIFIC_DATA;
  endian_le16_na_store(adv_config->ad + 5, BLE_COMPANY_APPLE);
  adv_config->ad[7] = 2;
  adv_config->ad[8] = 21;
  memrevcpy(adv_config->ad + 9, &config->group_uuid, 16);
  endian_be16_na_store(adv_config->ad + 25, config->major);
  endian_be16_na_store(adv_config->ad + 27, config->minor);
  adv_config->ad[29] = (int8_t)config->one_meter_rssi;
}

error_t ble_beacon_create(
    struct ble_stack_context_s *context,
    const struct ble_beacon_config_s *config,
    struct net_layer_s **beaconer)
{
  struct ble_beacon_adv_config_s params;

  ble_beacon_config_apply(&params, config);

  return DEVICE_OP(&context->ble, layer_create,
                  &context->scheduler,
                  BLE_NET_LAYER_ADV,
                  &params.adv_params,
                  NULL, NULL,
                  beaconer);
}

error_t ble_beacon_update(
    struct net_layer_s *beaconer,
    const struct ble_beacon_config_s *config)
{
  struct ble_beacon_adv_config_s params;

  ble_beacon_config_apply(&params, config);

  logk("Beacon config updated: "BLE_UUID_FMT" %d:%d, %d dbm @1m, interval: %d ms",
         BLE_UUID_ARG(&config->group_uuid),
         config->major, config->minor, config->one_meter_rssi,
         config->interval_ms);
  
  return ble_advertiser_params_update(beaconer, &params.adv_params);
}
