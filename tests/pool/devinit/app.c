
#include <device/class/timer.h>
#include <device/class/char.h>
#include <device/resources.h>
#include <device/irq.h>
#include <device/driver.h>
#include <mutek/thread.h>
#include <mutek/printk.h>
#include <mutek/startup.h>
#include <hexo/power.h>

#if defined(CONFIG_ARCH_SOCLIB) && !defined(CONFIG_DEVICE_ENUM)
DEV_DECLARE_STATIC(cpu_dev, "cpu0", DEVICE_FLAG_CPU,
                   arm32_drv,
                   DEV_STATIC_RES_ID(0, 0),
                   DEV_STATIC_RES_FREQ(1000000, 1)
                   );

DEV_DECLARE_STATIC(xicu_dev, "xicu", 0, soclib_xicu_drv,
                   DEV_STATIC_RES_MEM(0xd2200000, 0xd2201000),
                   DEV_STATIC_RES_DEV_ICU("/cpu0"),
                   DEV_STATIC_RES_IRQ(0, 0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_UINT_PARAM("pti-count", 1),
                   DEV_STATIC_RES_UINT_PARAM("hwi-count", 5),
                   DEV_STATIC_RES_UINT_PARAM("wti-count", 1),
                   DEV_STATIC_RES_UINT_PARAM("irq-count", 1),
                   );

DEV_DECLARE_STATIC(timer_dev, "timer", 0, enst_rttimer_drv,
                   DEV_STATIC_RES_MEM(0xd6000000, 0xd6000100),
                   DEV_STATIC_RES_DEV_ICU("/xicu"),
                   DEV_STATIC_RES_IRQ(0, 4, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_FREQ(1000000, 1)
                   );

DEV_DECLARE_STATIC(tty_dev, "tty", 0, soclib_tty_drv,
                   DEV_STATIC_RES_MEM(0xd0200000, 0xd0200010),
                   DEV_STATIC_RES_DEV_ICU("/xicu"),
                   DEV_STATIC_RES_IRQ(0, 0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   );
#endif

DEV_DECLARE_STATIC(async0_dev, "async0", DEVICE_FLAG_NO_STARTUP_WAIT, devinit_test_drv,
                   DEV_STATIC_RES_DEV_PARAM("timer", "/timer*"),
                   );

DEV_DECLARE_STATIC(async1_dev, "async1", DEVICE_FLAG_NO_STARTUP_WAIT, devinit_test_drv,
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

# ifdef CONFIG_DEVICE_ENUM
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

