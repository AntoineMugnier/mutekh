#include <hexo/types.h>

#include <gct_platform.h>
#include <gct/container_clist.h>

#include <mutek/buffer_pool.h>
#include <device/class/ble_radio.h>
#include <device/class/timer.h>

#include <ble/protocol/address.h>
#include <ble/protocol/advertise.h>
#include <ble/protocol/gap.h>

#include <ble/gatt/db.h>
#include <ble/stack/advertiser.h>
#include <ble/stack/peripheral.h>

#include <ble/security_db.h>

#include "runtime.h"

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
