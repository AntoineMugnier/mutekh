#ifndef BAS_H_
#define BAS_H_

#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <device/class/timer.h>
#include <device/class/valio.h>
#include <device/valio/adc.h>

struct batt_s
{
  struct ble_gattdb_registry_s dbs;
  struct device_timer_s timer;
  struct device_valio_s adc;
  dev_timer_delay_t interval;
  struct dev_timer_rq_s waiter;
  struct dev_valio_rq_s adc_req;
  struct valio_adc_group_s adc_group;
  int16_t adc_value;
  uint8_t last_value;
  bool_t adc_busy, waiting, running;
};

STRUCT_COMPOSE(batt_s, dbs);

extern const struct ble_gattdb_service_s batt_service;

error_t bas_register(struct batt_s *dbs,
                             struct ble_gattdb_s *db);

void bas_battery_level_set(struct batt_s *dbs, uint8_t level);

#endif
