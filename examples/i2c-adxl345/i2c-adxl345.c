
#include <hexo/types.h>
#include <pthread.h>
#include <mutek/printk.h>

#include <device/class/i2c.h>

#include <time.h>

void main()
{
  struct device_i2c_ctrl_s i2c;

  if (cpu_isbootstrap())
  {
    if (device_get_accessor_by_path(&i2c, 0, "i2c*", DRIVER_CLASS_I2C))
      abort();
  }

  uint8_t devid;
  ssize_t nbytes;
  do
    {
      nbytes = dev_i2c_wait_read(
        &i2c, DEV_I2C_ADDR_7_BITS, 0x53, 0x00, &devid, 1);

      if (nbytes == 1)
        {
          printk("i2c-adxl345: device id is 0x%x.\n", devid);
        }
      else
        printk("i2c-adxl345: error %d.\n", nbytes);
    }
  while (nbytes == -EBUSY);

  while (1)
  do
    {
      uint16_t data_x;

      nbytes = dev_i2c_wait_read(
        &i2c, DEV_I2C_ADDR_7_BITS, 0x53, 0x32, (uint8_t*)&data_x, 2);

      if (nbytes == 2)
        {
          printk("i2c-adxl345: data X is 0x%x.\n", data_x);
        }
      else
        printk("i2c-adxl345: error %d.\n", nbytes);
    }
  while (nbytes == -EBUSY);

  while (1)
    ;
}

