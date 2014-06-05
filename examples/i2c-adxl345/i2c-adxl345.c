
#include <hexo/types.h>
#include <mutek/printk.h>

#include <device/class/i2c.h>

#include <pthread.h>
#include <string.h>
#include <time.h>

#define ADXL345_I2C_ADDR    0x53

static error_t adxl345_read_bytes(struct device_i2c_ctrl_s *bus,
                                  uint8_t                  reg,
                                  uint8_t                  *buffer,
                                  size_t                   count)
{
  ssize_t nbytes;
  do
    {
      nbytes = dev_i2c_wait_read(
        bus,
        DEV_I2C_ADDR_7_BITS,
        ADXL345_I2C_ADDR,
        reg,
        buffer,
        count
      );
    }
  while (nbytes < count || nbytes == -EBUSY);
  return nbytes;
}

static error_t adxl345_write_bytes(struct device_i2c_ctrl_s *bus,
                                   uint8_t                  reg,
                                   uint8_t const            *buffer,
                                   size_t                   count)
{
  ssize_t nbytes;
  do
    {
      nbytes = dev_i2c_wait_write(
        bus,
        DEV_I2C_ADDR_7_BITS,
        ADXL345_I2C_ADDR,
        reg,
        buffer,
        count
      );
    }
  while (nbytes < count || nbytes == -EBUSY);
  return nbytes;
}

void main()
{
  struct device_i2c_ctrl_s i2c;

  if (cpu_isbootstrap())
  {
    if (device_get_accessor_by_path(&i2c, 0, "i2c*", DRIVER_CLASS_I2C))
      abort();
  }

  uint8_t data[16];

  /* check device id. */
  if (adxl345_read_bytes(&i2c, 0x00, data, 1) < 0)
    {
      printk("i2c-adxl345: cannot read device id register.\n");
      abort();
    }

  printk("i2c-adxl345: device id is 0x%lx.\n", (int32_t)data[0]);

  /* configuration resolution to full-mode and range +/- 16g. */
  data[0] = (0x1 << 3) | 0x3;
  if (adxl345_write_bytes(&i2c, 0x31, data, 1) < 0)
    {
      printk("i2c-adxl345: failed to setup full-mode and range.\n");
      abort();
    }

  /* enable measuring (leave in FIFO bypass) and link. */
  data[0] = (0x1 << 5) | (0x1 << 3);
  if (adxl345_write_bytes(&i2c, 0x2d, data, 1) < 0)
    {
      printk("i2c-adxl345: failed to enable measuring and link.\n");
      abort();
    }

  int16_t calib[16][3];
  uint_fast8_t s;
  for (s = 0; s < 16; ++s)
    {
      ssize_t err;
      if ((err = adxl345_read_bytes(&i2c, 0x32, (uint8_t*)calib[s], 6)) < 0)
        {
          printk("i2c-adxl345: error %d while reading data.\n", -err);
          memset(calib[s], 6, 0);
        }
    }

  /* disable measuring (leave in FIFO bypass) and link. */
  data[0] = 0x0;
  if (adxl345_write_bytes(&i2c, 0x2d, data, 1) < 0)
    {
      printk("i2c-adxl345: failed to pause measuring.\n");
      abort();
    }

  /* compute calibration. */
  int16_t off_x = 0, off_y = 0, off_z = 0;
  for (s = 0; s < 16; ++s)
    {
      off_x += calib[s][0];
      off_y += calib[s][1];
      off_z += calib[s][2];
    }
  /* compute average of 16 values with a scale factor of 2 at +/- 16g. */
  off_x >>= (4 - 1);
  off_y >>= (4 - 1);
  off_z >>= (4 - 1);

  /* substract sensitivity of Z axis at +/- 16g. */
  off_z -= 32;

  *(int8_t *)&data[0] = (int8_t) -off_x;
  *(int8_t *)&data[1] = (int8_t) -off_y;
  *(int8_t *)&data[2] = (int8_t) -off_z;

  /* write offset registers. */
  if (adxl345_write_bytes(&i2c, 0x1e, data, 3) < 0)
    {
      printk("i2c-adxl345: failed to set axis offsets.\n");
      abort();
    }

  printk("i2c-adxl345: calibration done.\n");

  /* re-enable measuring (leave in FIFO bypass) and link. */
  data[0] = (0x1 << 5) | (0x1 << 3);
  if (adxl345_write_bytes(&i2c, 0x2d, data, 1) < 0)
    {
      printk("i2c-adxl345: failed to resume measuring.\n");
      abort();
    }

  printk("i2c-adxl345: start measuring ...\n");

  while (1)
    {
      if (adxl345_read_bytes(&i2c, 0x32, data, 6) < 0)
        {
          printk("i2c-adxl345: error %d while reading data.\n");
        }
      else
        {
          printk("i2c-adxl345: X = %d, Y = %d, Z = %d.\n",
                 *(int16_t *)&data[0],
                 *(int16_t *)&data[2],
                 *(int16_t *)&data[4]);
        }
    }
}

