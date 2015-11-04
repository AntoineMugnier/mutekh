#ifndef LED_H_
#define LED_H_

#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <device/class/pwm.h>

struct led_s
{
    struct ble_gattdb_registry_s dbs;
    struct device_pwm_s pwm;

    struct dev_pwm_config_s cfg[3];
};

STRUCT_COMPOSE(led_s, dbs);

error_t led_service_register(struct led_s *led,
                             struct ble_gattdb_s *db);

#endif
