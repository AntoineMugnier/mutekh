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

//Example of a max31825 driver instance definition
//
//  DEV_DECLARE_STATIC(max31825_r_u1_dev, "max31825_u1", 0, max31825_drv,
//                    DEV_STATIC_RES_DEV_ICU("/cpu"),
//                     DEV_STATIC_RES_DEV_ONEWIRE("/onewire_r"),
//                     DEV_STATIC_RES_UINT_PARAM("init_charging_time_us", 2500),
//                     DEV_STATIC_RES_UINT_PARAM("device_address", 0b111111)
//                     );


 struct max31825_rq_data{
  struct device_valio_s* max31825_dev;
  struct valio_temperature_s* temp_kelvin;
  uint8_t sensor_id;
};

struct app_s
{
  struct device_valio_s max31825_dev[4];
  struct max31825_rq_data max31825_rq_data[4];
  struct dev_valio_rq_s max31825_rq[4];
};


static
KROUTINE_EXEC(temp_update)
{
  struct dev_valio_rq_s *rq = dev_valio_rq_from_kr(kr);
  struct max31825_rq_data* data  = rq->base.pvdata;
  static uint32_t counter = 0;
  logk("Temperature now U%d %d mK , cnt %d", data->sensor_id, data->temp_kelvin, counter++);

  DEVICE_OP(data->max31825_dev, request, rq);
}

void app_start(void)
{
  logk("1-Wire test");

  // Initialize app, creating storage for each of its members
  struct app_s *app = mem_alloc(sizeof(*app), mem_scope_sys);
  memset(app, 0, sizeof(*app));
  error_t err;
  
  char* device_instance_name[4] = {"max31825_u1", "max31825_u2", "max31825_u3", "max31825_u4"};
  for (uint8_t sensor_index = 0; sensor_index < 4; sensor_index++) {

      // Initialize MAX31825 device
    err = device_get_accessor_by_path(&app->max31825_dev[sensor_index].base, NULL, device_instance_name[sensor_index], DRIVER_CLASS_VALIO);
    if(err){
      logk_error("Error getting %s device", device_instance_name[sensor_index]);
      return ;
    }
    // Initialize request data
    app->max31825_rq_data[sensor_index].sensor_id = sensor_index + 1;
    app->max31825_rq_data[sensor_index].max31825_dev = &app->max31825_dev[sensor_index];

    // Initialize request
    dev_valio_rq_init(&app->max31825_rq[sensor_index], temp_update);
    app->max31825_rq[sensor_index].data = &app->max31825_rq_data[sensor_index].temp_kelvin;
    app->max31825_rq[sensor_index].type = DEVICE_VALIO_READ;
    app->max31825_rq[sensor_index].attribute = VALIO_TEMPERATURE_VALUE;
    app->max31825_rq[sensor_index].base.pvdata = &app->max31825_rq_data[sensor_index];

    DEVICE_OP(&app->max31825_dev[sensor_index], request, &app->max31825_rq[sensor_index]);
  }

}
