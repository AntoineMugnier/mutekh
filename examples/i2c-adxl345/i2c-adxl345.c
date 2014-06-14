
#include <hexo/types.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/char.h>
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

/* data sent through serial link. */
struct adxl345_data_s
  {
    struct
      {
        uint16_t const magic;
        uint16_t       __padding__:12;
        size_t         count:4;
      } __attribute__((packed)) hdr;
    uint16_t data[16];
  } __attribute__((packed));

void main()
{
  struct device_char_s     serial;
  struct device_i2c_ctrl_s i2c;

  if (!cpu_isbootstrap())
    while(1)
      ;

  printk("i2c-adxl345: start !\n");

  if (device_get_accessor_by_path(&i2c, 0, "i2c1 i2c*", DRIVER_CLASS_I2C))
    {
      printk("i2c-adxl345: failed to get i2c accessor.\n");
      goto err_global;
    }

  uint8_t data[16];

  printk("i2c-adxl345: check device id.\n");

  /* check device id. */
  if (adxl345_read_bytes(&i2c, 0x00, data, 1) < 0)
    {
      printk("i2c-adxl345: cannot read device id register.\n");
      goto err_i2c;
    }

  printk("i2c-adxl345: device id is 0x%lx.\n", (int32_t)data[0]);

  /* configuration resolution to full-mode and range +/- 16g. */
  data[0] = (0x1 << 3) | 0x3;
  if (adxl345_write_bytes(&i2c, 0x31, data, 1) < 0)
    {
      printk("i2c-adxl345: failed to setup full-mode and range.\n");
      goto err_i2c;
    }

  /* enable measuring (leave in FIFO bypass) and link. */
  data[0] = (0x1 << 5) | (0x1 << 3);
  if (adxl345_write_bytes(&i2c, 0x2d, data, 1) < 0)
    {
      printk("i2c-adxl345: failed to enable measuring and link.\n");
      goto err_i2c;
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
      goto err_i2c;
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
      goto err_i2c;
    }

  printk("i2c-adxl345: calibration done.\n");

  /* re-enable measuring (leave in FIFO bypass) and link. */
  data[0] = (0x1 << 5) | (0x1 << 3);
  if (adxl345_write_bytes(&i2c, 0x2d, data, 1) < 0)
    {
      printk("i2c-adxl345: failed to resume measuring.\n");
      goto err_i2c;
    }

  printk("i2c-adxl345: start measuring ...\n");

  if (device_get_accessor_by_path(
    &serial,
    0,
    "uart1",
    DRIVER_CLASS_CHAR)
  )
    {
      printk("i2c-adxl345: failed to get serial accessor.\n");
      goto err_serial;
    }

  int16_t samples[3];
  while (1)
    {
      if (adxl345_read_bytes(&i2c, 0x32, (uint8_t *)samples, 6) < 0)
        printk("i2c-adxl345: error %d while reading data.\n");
      else
        {
          static struct adxl345_data_s val = { .hdr.magic = 0x123 };
          val.hdr.count = 3;
          val.data[0] = samples[0];
          val.data[1] = samples[1];
          val.data[2] = samples[2];

          dev_char_wait_write(
            &serial,
            (const uint8_t *)&val,
            sizeof(val.hdr) + (val.hdr.count * sizeof(val.data[0]))
          );
          printk(".");
        }
    }

err_serial:
  printk("i2c-adxl345: error while seting up serial.\n");
  device_put_accessor(&serial);

err_i2c:
  printk("i2c-adxl345: error while seting up i2c.\n");
  device_put_accessor(&i2c);

err_global:
  printk("i2c-adxl345: cannot start !\n");
  pthread_yield();
}

