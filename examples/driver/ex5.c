/*
   Additional topics covered by this example:
     - Implement a custom `use' function of the driver API

   This is an improved version of the previous example.

   In this example, the timer is not started on device initialization,
   the user of the timer API has to explicitly start and stop the
   timer for the hardware to run.
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/error.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/timer.h>

enum {
  REG_VALUE    = 0,
  REG_CTRL     = 4,
  REG_PERIOD   = 8,
};

enum {
  CTRL_RUNNING = 1,
};

DRIVER_PV(struct mydrv_context_s
{
  uintptr_t addr;
});

static DEV_TIMER_REQUEST(mydrv_timer_request)
{
  return -ENOTSUP;
}

static DEV_TIMER_CANCEL(mydrv_timer_cancel)
{
  return -ENOTSUP;
}

static DEV_TIMER_GET_VALUE(mydrv_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct mydrv_context_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  if (dev->start_count == 0)
    return -EBUSY;

  *value = endian_le32(cpu_mem_read_32(pv->addr + REG_VALUE));
  return 0;
}

static DEV_TIMER_CONFIG(mydrv_timer_config)
{
  if (cfg)
    {
      cfg->freq = DEV_FREQ(0, 0, 0, 0);
      cfg->max = 0xffffffff;
      cfg->rev = 1;
      cfg->res = 1;
      cfg->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_TICKLESS
        | DEV_TIMER_CAP_KEEPVALUE | DEV_TIMER_CAP_HIGHRES;
    }

  if (res != 1)
    return -ERANGE;

  return 0;
}

/* We provide our own use function in this driver */
static DEV_USE(mydrv_use)
{
  switch (op)
    {
    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct mydrv_context_s *pv = dev->drv_pv;

      /* starts the timer hardware */
      cpu_mem_write_32(pv->addr + REG_CTRL, endian_le32(CTRL_RUNNING));
      return 0;
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct mydrv_context_s *pv = dev->drv_pv;

      /* stops the timer hardware */
      cpu_mem_write_32(pv->addr + REG_CTRL, 0);
      return 0;
    }

    default:
      /* relies on the generic implementation for other cases */
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(mydrv_init)
{
  struct mydrv_context_s *pv;

  uintptr_t addr;
  if (device_res_get_mem(dev, 0, &addr, NULL))
    return -EINVAL;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;
  pv->addr = addr;

  /* set the timer period */
  cpu_mem_write_32(pv->addr + REG_PERIOD, endian_le32(0xffffffff));

  return 0;
}

static DEV_CLEANUP(mydrv_cleanup)
{
  struct mydrv_context_s *pv = dev->drv_pv;

  mem_free(pv);
  return 0;
}

DRIVER_DECLARE(mydrv5_drv, 0, "Simple timer driver example with start/stop", mydrv,
               DRIVER_TIMER_METHODS(mydrv_timer)
               );

DRIVER_REGISTER(mydrv5_drv);
