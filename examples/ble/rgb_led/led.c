#include <ble/gattdb/db.h>
#include <ble/gattdb/service.h>
#include <ble/gatt/characteristic.h>
#include <ble/gatt/service.h>
#include <mutek/printk.h>
#include "led.h"

/*
  Rant to Cypress for their RGB Led example:

  You are a Bluetooth SIG member, you know about Type UUID allocation.
  You should use custom 128-bit UUIDs, not hijack some short ones for
  demo purposes.  People do exactly as you do in demo code.  If they
  see you hijack 16-bit UUIDs, they'll do the same in actual products.

  Now, I'm tempted to reuse your Type UUIDs just not to redevelop an
  iPhone client app to test my stack, thus perpetuating bad practices.

  Kids, dont do this at home, use custom 128-bit UUIDs.
 */
#define CYPRESS_LED_SERVICE 0xcbbb
#define CYPRESS_LED_COLOR_CHAR 0xcbb1

static uint8_t led_color[4] = {0,0,255,255};

static
uint8_t on_led_color_write(struct ble_gattdb_client_s *client,
                           struct ble_gattdb_registry_s *service,
                           uint8_t charid)
{
  struct led_s *led = led_s_from_dbs(service);

  for (uint8_t i = 0; i < 3; ++i) {
    led->cfg[i].duty.num = led_color[i];
    led->cfg[i].duty.denom = ((256 - led_color[3]) << 4) - (1 << 4) + 0x100;
  }

  struct dev_request_status_s status;
  struct dev_pwm_rq_s rq = {
    .cfg = led->cfg,
    .chan_mask = 0x7,
    .error = 0,
    .mask = DEV_PWM_MASK_DUTY,
  };

  dev_request_sched_init(&rq.base, &status);
  DEVICE_OP(&led->pwm, config, &rq);
  dev_request_sched_wait(&status);

  return 0;
}

BLE_GATTDB_SERVICE_DECL(led_service,
                        BLE_GATTDB_SERVICE_PRIMARY | BLE_GATTDB_SERVICE_ADVERTISED,
                        BLE_UUID_BT_BASED_P(CYPRESS_LED_SERVICE),
                        NULL,
                        BLE_GATTDB_CHAR(BLE_UUID_BT_BASED_P(CYPRESS_LED_COLOR_CHAR),
                                        BLE_GATTDB_PERM_OTHER_WRITE | BLE_GATTDB_PERM_OTHER_READ,
                                        BLE_GATTDB_CHAR_DATA_PLAIN(led_color, sizeof(led_color),
                                                                   NULL, on_led_color_write)),
                        );

error_t led_service_register(struct led_s *led,
                             struct ble_gattdb_s *db)
{
  error_t err;

  err = device_get_accessor_by_path(&led->pwm, NULL, "leds", DRIVER_CLASS_PWM);
  if (err)
    return err;

  for (uint8_t i = 0; i < 3; ++i) {
    led->cfg[i].freq.num = 100;
    led->cfg[i].freq.denom = 1;
    led->cfg[i].duty.num = 0;
    led->cfg[i].duty.denom = 1;
    led->cfg[i].pol = DEV_PWM_POL_LOW;
  }

  struct dev_request_status_s status;
  struct dev_pwm_rq_s rq = {
    .cfg = led->cfg,
    .chan_mask = 0x7,
    .error = 0,
    .mask = DEV_PWM_MASK_FREQ | DEV_PWM_MASK_DUTY | DEV_PWM_MASK_POL,
  };

  dev_request_sched_init(&rq.base, &status);
  DEVICE_OP(&led->pwm, config, &rq);
  dev_request_sched_wait(&status);

  printk("PWM init: %d\n", rq.error);

  return ble_gattdb_service_register(&led->dbs, db, &led_service);
}
