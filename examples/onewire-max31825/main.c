#include <stdint.h>
#include <string.h>
#define LOGK_MODULE_ID "main"

#include <mutek/printk.h>
#include <mutek/mem_alloc.h>
#include <mutek/startup.h>
#include <device/class/onewire.h>
#include <device/class/timer.h>
#include <device/driver.h>
#include <device/class/gpio.h>
#include <device/class/valio.h>
#include <device/valio/temperature.h>
struct app_s
{
  struct device_valio_s max31825_r_bus;
  struct dev_valio_rq_s max31825_r_rq;
  struct valio_temperature_s temp_r_u1_val_kelvin;
};

STRUCT_COMPOSE(app_s, max31825_r_rq);

static
KROUTINE_EXEC(app_temp_changed)
{
  struct dev_valio_rq_s *rq = dev_valio_rq_from_kr(kr);
  struct app_s *app = app_s_from_max31825_r_rq(rq);
  static uint32_t counter = 0;
  logk("Temperature now %d mK , cnt %d", app->temp_r_u1_val_kelvin, counter++);

  DEVICE_OP(&app->max31825_r_bus, request, &app->max31825_r_rq);
}

void app_start(void)
{

  struct app_s *app = mem_alloc(sizeof(*app), mem_scope_sys);
  error_t err;

  memset(app, 0, sizeof(*app));
  
  logk("1-Wire test");
  
  err = device_get_accessor_by_path(&app->max31825_r_bus.base, NULL, "max31825_r_u1", DRIVER_CLASS_VALIO);
  ensure(!err && "Error getting MAX31825 sensors device");

  dev_valio_rq_init(&app->max31825_r_rq, app_temp_changed);
  app->max31825_r_rq.data = &app->temp_r_u1_val_kelvin;
  app->max31825_r_rq.type = DEVICE_VALIO_READ;
  app->max31825_r_rq.attribute = VALIO_TEMPERATURE_VALUE;
  
  DEVICE_OP(&app->max31825_r_bus, request, &app->max31825_r_rq);

}
