
#include <pthread.h>
#include <mutek/printk.h>

#include <device/class/i2c.h>

#include <time.h>

void main()
{
  uint8_t                  addr, count = 0;
  struct device_i2c_ctrl_s i2c;

  if (cpu_isbootstrap())
  {
    if (device_get_accessor_by_path(&i2c, 0, "i2c*", DRIVER_CLASS_I2C))
      abort();

    printk("i2c: start scanning ...\n");
    for (addr = 0; addr < 128; ++addr)
      {
        error_t err;
        do
          {
            if (!(err = dev_i2c_wait_scan(&i2c, DEV_I2C_ADDR_7_BITS, addr)))
              {
                printk("i2c: found a device with address 0x%lx.\n", addr);
                ++count;
              }
          }
        while (err == -EBUSY);
#if 0
#if defined(CONFIG_DEVICE_TIMER)
        usleep(200);
#else
        int i;
        for (i = 0; i < 1000; ++i);
#endif
#endif
      }
    printk("i2c: done.\n");
    printk("i2c: found %u devices.\n", count);
  }

  pthread_yield();
}

