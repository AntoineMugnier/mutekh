#ifndef LED_H_
#define LED_H_

#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <device/class/valio.h>
#include <device/valio/led.h>

struct led_s
{
  struct ble_gattdb_registry_s dbs;
  struct device_valio_s led;
  struct dev_valio_rq_s led_rq;
  struct valio_led_luminosity_s color;
};

STRUCT_COMPOSE(led_s, dbs);
STRUCT_COMPOSE(led_s, led_rq);

error_t led_service_register(struct led_s *led,
                             struct ble_gattdb_s *db);

#endif
