/*
   Additional topics covered by this example:
     - Add a dependency on an other device

   In this example we implements a wrapper driver which simply forward
   the timer API calls to an other device driver. The wrapper logs the
   call to the wrapped timer API so that it can be used for debug
   purpose.

   Of course, this driver does not drive any hardware directly and
   can be used with any other timer device.

   The actual device used as slave timer is specified in device
   resources:

   DEV_DECLARE_STATIC(my_dev7, "mydev7", 0, mydrv7_drv,
                    DEV_STATIC_RES_DEV_PARAM("myslave", "/timer")
                    );

   It must be initialized and its driver must implement the timer API
   for our wrapper driver to initialize properly.  Because libdevice
   handle dependencies between devices, the initialization of our
   driver will be postponed until the user specified slave driver is
   ready for use.

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

DRIVER_PV(struct mydrv_context_s
{
  /* An accessor to the slave timer. This is as if we were to
     use a timer from an application. */
  struct device_timer_s slave;
});

/* below are the 4 wrappers for the timer API calls */

static DEV_TIMER_REQUEST(mydrv_timer_request)
{
  struct device_s *dev = accessor->dev;
  struct mydrv_context_s *pv = dev->drv_pv;

  printk("(call %s, dev %p)\n", __func__, dev);

  return DEVICE_OP(&pv->slave, request, rq);
}

static DEV_TIMER_CANCEL(mydrv_timer_cancel)
{
  struct device_s *dev = accessor->dev;
  struct mydrv_context_s *pv = dev->drv_pv;

  printk("(call %s, dev %p)\n", __func__, dev);

  return DEVICE_OP(&pv->slave, cancel, rq);
}

static DEV_TIMER_GET_VALUE(mydrv_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct mydrv_context_s *pv = dev->drv_pv;

  printk("(call %s, dev %p)\n", __func__, dev);

  return DEVICE_OP(&pv->slave, get_value, value, rev);
}

static DEV_TIMER_CONFIG(mydrv_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct mydrv_context_s *pv = dev->drv_pv;

  printk("(call %s, dev %p)\n", __func__, dev);

  return DEVICE_OP(&pv->slave, config, cfg, res);
}

/* we need to forward device
   start and stop actions as well */

static DEV_USE(mydrv_use)
{
  switch (op)
    {
    case DEV_USE_START: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct mydrv_context_s *pv = dev->drv_pv;

      printk("(start, dev %p)\n", dev);

      return device_start(&pv->slave);
    }

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      struct device_s *dev = acc->dev;
      struct mydrv_context_s *pv = dev->drv_pv;

      printk("(stop, dev %p)\n", dev);

      return device_stop(&pv->slave);
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(mydrv_init)
{
  struct mydrv_context_s *pv;

  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;

  /* this helper function query the device resources and get an
     accessor to the timer device the user as chosen to wrap. */
  error_t err = device_get_param_dev_accessor(dev, "myslave", &pv->slave, DRIVER_CLASS_TIMER);
  if (err)
    {
      mem_free(pv);
      return err;
    }

  return 0;
}

static DEV_CLEANUP(mydrv_cleanup)
{
  struct mydrv_context_s *pv = dev->drv_pv;

  /* release the slave timer device */
  device_put_accessor(&pv->slave);

  mem_free(pv);
  return 0;
}

DRIVER_DECLARE(mydrv7_drv, 0, "Simple timer wrapper example", mydrv,
               DRIVER_TIMER_METHODS(mydrv_timer)
               );

DRIVER_REGISTER(mydrv7_drv);
