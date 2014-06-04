
#include <pthread.h>
#include <mutek/printk.h>

#include <device/class/i2c.h>

#include <time.h>

void main()
{
  uint8_t                  addr, count;
  struct device_i2c_ctrl_s i2c;

  if (cpu_isbootstrap())
  {
    if (device_get_accessor_by_path(&i2c, 0, "i2c*", DRIVER_CLASS_I2C))
      abort();

    printk("i2c: start scanning (spin)...\n");
    count = 0;
    for (addr = 0x0; addr < 0x7f; ++addr)
      {
        ssize_t nb;
        do
          {
            if ((nb = dev_i2c_spin_scan(&i2c, DEV_I2C_ADDR_7_BITS, addr)) == 0)
              {
                printk("i2c: found a device with address 0x%lx.\n", addr);
                ++count;
              }
          }
        while (nb == -EBUSY);
      }
    printk("i2c: done.\n");
    printk("i2c: found %u devices.\n", count);

    printk("\n");

    printk("i2c: start scanning (wait)...\n");
    count = 0;
    for (addr = 0x0; addr < 0x7f; ++addr)
      {
        ssize_t nb;
        do
          {
            if ((nb = dev_i2c_wait_scan(&i2c, DEV_I2C_ADDR_7_BITS, addr)) == 0)
              {
                printk("i2c: found a device with address 0x%lx.\n", addr);
                ++count;
              }
          }
        while (nb == -EBUSY);
      }
    printk("i2c: done.\n");
    printk("i2c: found %u devices.\n", count);
  }

  while (1)
    ;
}

