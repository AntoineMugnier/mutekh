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
KROUTINE_EXEC(led_done)
{
  struct dev_valio_rq_s *rq = dev_valio_rq_from_kr(kr);
  struct led_s *led = led_s_from_led_rq(rq);

  (void)led;
}

static
uint8_t on_led_color_write(struct ble_gattdb_client_s *client,
                           struct ble_gattdb_registry_s *service,
                           uint8_t charid)
{
  struct led_s *led = led_s_from_dbs(service);

  for (uint8_t i = 0; i < 3; ++i)
    led->color.lum[i] = led_color[i];

  DEVICE_OP(&led->led, request, &led->led_rq);

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

  err = device_get_accessor_by_path(&led->led.base, NULL, "led", DRIVER_CLASS_VALIO);
  if (err)
    return err;

  for (uint8_t i = 0; i < 3; ++i)
    led->color.lum[i] = 0;

  dev_valio_rq_init(&led->led_rq, led_done);
  led->led_rq.data = &led->color;
  led->led_rq.attribute = VALIO_LED;
  led->led_rq.type = DEVICE_VALIO_WRITE;

  DEVICE_OP(&led->led, request, &led->led_rq);

  return ble_gattdb_service_register(&led->dbs, db, &led_service);
}
