
#include <hexo/types.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/driver.h>

#include <device/class/valio.h>
#include <device/valio/motion.h>

#include <pthread.h>
#include <string.h>
#include <time.h>

void main()
{
  struct device_valio_s      accel;
  struct valio_motion_data_s data;

  if (!cpu_isbootstrap())
    while(1)
      ;

  if (device_get_accessor_by_path(&accel.base, 0, "accel*", DRIVER_CLASS_VALIO))
    {
      printk("i2c-adxl345: failed to get accessor.\n");
      goto err;
    }

  printk("i2c-adxl345: start measuring ...\n");

  if (device_start(&accel.base))
    {
      printk("i2c-adxl345: cannot start device.\n");
      goto err_accessor;
    }

  while (1)
    {
      if (dev_valio_wait_read(&accel, VALIO_MOTION_DATA, &data))
        goto err_accessor;

      printk("i2c-adxl345: X:%d Y:%d Z:%d\n",
             data.accel.x, data.accel.y, data.accel.z);
    }

  device_stop(&accel.base);

err_accessor:
  device_put_accessor(&accel.base);

err:
  while (1)
    ;
}

