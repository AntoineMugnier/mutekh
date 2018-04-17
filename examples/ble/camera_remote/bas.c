#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/gatt/hid.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/service.h>

#include "bas.h"

#define dprintk(...) do{}while(0)
//#define dprintk printk

#include <mutek/printk.h>

#define ADC_MASK (1 << 0)

static void batt_schedule_next(struct batt_s *batt)
{
  assert(!batt->adc_busy);

  DEVICE_OP(&batt->timer, get_value, &batt->waiter.deadline, 0);
  batt->waiter.deadline += batt->interval;

  DEVICE_OP(&batt->timer, request, &batt->waiter);
}

static KROUTINE_EXEC(batt_sample)
{
  struct batt_s *batt = KROUTINE_CONTAINER(kr, *batt, waiter.rq.kr);

  dprintk("%s %d\n", __FUNCTION__, batt->adc_busy);

  if (batt->adc_busy)
    return;

  batt->adc_busy = 1;

  batt->adc_req.data = &batt->adc_group;
  batt->adc_req.attribute = VALIO_ADC_VALUE;
  batt->adc_req.type = DEVICE_VALIO_READ;
  batt->adc_group.mask = ADC_MASK;

  DEVICE_OP(&batt->adc, request, &batt->adc_req);
}

static KROUTINE_EXEC(batt_adc_done)
{
  struct batt_s *batt = KROUTINE_CONTAINER(kr, *batt, adc_req.base.kr);
  bool_t changed = 0;

  dprintk("%s %d %d\n", __FUNCTION__, batt->adc_busy, batt->adc_req.error);

  uint8_t percent = __MIN(__MAX(batt->adc_value - 569, 0) / 2, 100);

  dprintk("Raw batt reading: %d, percent %d\n", batt->adc_value, percent);

  changed = batt->last_value != percent;

  if (changed)
    ble_gattdb_char_changed(&batt->dbs, 0, 0, &batt->last_value, 1);

  batt->last_value = percent;
  batt->adc_busy = 0;

  batt_schedule_next(batt);
}

static
uint8_t batt_level_subscribed(struct ble_gattdb_registry_s *reg,
                              uint8_t charid,
                              bool_t subscribed)
{
  struct batt_s *batt = batt_s_from_dbs(reg);
  
  printk("Batt level subscribed: %d\n", subscribed);

  batt->running = subscribed;
    
  return 0;
}

static
uint8_t batt_level_read(struct ble_gattdb_client_s *client,
                        struct ble_gattdb_registry_s *reg, uint8_t charid,
                        uint16_t offset,
                        void *data, size_t *size)
{
  struct batt_s *batt = batt_s_from_dbs(reg);

  if (*size < 1)
    return -ENOMEM;

  if (offset)
    return -EINVAL;

  *(uint8_t *)data = batt->last_value;
  *size = 1;

  return 0;
}

BLE_GATTDB_SERVICE_DECL(batt_service,
  BLE_GATTDB_SERVICE_PRIMARY,
  BLE_UUID_BT_BASED_P(BLE_GATT_SERVICE_BATTERY),
  NULL,
  BLE_GATTDB_CHAR(BLE_UUID_BT_BASED_P(BLE_GATT_CHAR_BATTERY_LEVEL),
                  BLE_GATTDB_NOTIFIABLE | BLE_GATTDB_PERM_OTHER_READ,
                  BLE_GATTDB_CHAR_DATA_DYNAMIC(batt_level_read,
                                               NULL,
                                               batt_level_subscribed),
                  BLE_GATTDB_DESCRIPTORS(BLE_HID_INPUT_REPORT(2))
                  ),
                        );

error_t bas_register(struct batt_s *batt,
                     struct ble_gattdb_s *db)
{
  memset(batt, 0, sizeof(*batt));

  kroutine_init_sched_switch(&batt->waiter.rq.kr, batt_sample);
  kroutine_init_sched_switch(&batt->adc_req.base.kr, batt_adc_done);

  if (device_get_accessor_by_path(&batt->timer.base, NULL, "/rtc1", DRIVER_CLASS_TIMER))
    return -ENOENT;

  if (device_get_accessor_by_path(&batt->adc.base, NULL, "/adc", DRIVER_CLASS_VALIO))
    return -ENOENT;

  batt->adc_busy = 0;
  batt->last_value = 0xff;

  device_start(&batt->timer.base);

  dev_timer_init_sec(&batt->timer, &batt->interval, NULL, 30, 1);

  kroutine_exec(&batt->waiter.rq.kr);

  return ble_gattdb_service_register(&batt->dbs, db, &batt_service);
}
