
#include <device/class/timer.h>
#include <device/class/char.h>
#include <device/resources.h>
#include <device/driver.h>
#include <mutek/thread.h>
#include <mutek/printk.h>
#include <hexo/power.h>

DEV_DECLARE_STATIC(async0_dev, "async0", 0, devinit_test_drv,
                   DEV_STATIC_RES_DEV_PARAM("timer", "/timer*"),
                   );

DEV_DECLARE_STATIC(async1_dev, "async1", 0, devinit_test_drv,
                   DEV_STATIC_RES_DEV_PARAM("timer", "/timer*"),
                   DEV_STATIC_RES_DEVCLASS_PARAM("dep", "/async0", DRIVER_CLASS_TIMER),
                   );

static CONTEXT_ENTRY(thread)
{
  printk("thread start\n");
  struct device_timer_s t;
  struct device_char_s c;

#ifdef CONFIG_DEVICE_INIT_ASYNC

  if (!device_get_accessor(&c.base, &async0_dev, DRIVER_CLASS_CHAR, 0))
    goto err;

  if (!device_get_accessor(&c.base, &async1_dev, DRIVER_CLASS_CHAR, 0))
    goto err;

#  ifdef CONFIG_DEVICE_INIT_PARTIAL
  if (device_get_accessor(&t.base, &async1_dev, DRIVER_CLASS_TIMER, 0))
    goto err;
  device_put_accessor(&t.base);
#  endif

# ifdef CONFIG_DEVICE_INIT_ENUM
  if (device_wait_accessor(&c.base, &async0_dev, DRIVER_CLASS_CHAR, 0))
    goto err;
  device_put_accessor(&c.base);

  if (device_get_accessor(&c.base, &async0_dev, DRIVER_CLASS_CHAR, 0))
    goto err;
  device_put_accessor(&c.base);

  if (!device_get_accessor(&c.base, &async1_dev, DRIVER_CLASS_CHAR, 0))
    goto err;

  if (device_wait_accessor(&c.base, &async1_dev, DRIVER_CLASS_CHAR, 0))
    goto err;
  device_put_accessor(&c.base);

  if (device_get_accessor(&c.base, &async1_dev, DRIVER_CLASS_CHAR, 0))
    goto err;
  device_put_accessor(&c.base);
# endif
#else

  if (device_get_accessor(&c.base, &async0_dev, DRIVER_CLASS_CHAR, 0))
    goto err;
  device_put_accessor(&c.base);

  if (device_get_accessor(&c.base, &async1_dev, DRIVER_CLASS_CHAR, 0))
    goto err;
  device_put_accessor(&c.base);

#endif

  printk("++SUCCESS++%u++\n", 0);
  power_shutdown();
  power_reboot();
  return;

 err:
  printk("++FAIL++\n");
  power_shutdown();
  power_reboot();
}

void app_start()
{
  thread_create(thread, NULL, NULL);
}

