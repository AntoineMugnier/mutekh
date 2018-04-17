/*
   Additional topics covered by this example:
     - Introduce the timer device class API
     - Initializes and cleanup the device hardware

   This example implements a driver for a simple timer device.  The
   driver is not able to schedule requests, it only supports returning
   the current value of the timer.

   The hardware timer we drive here has the following
   32 bits little endian memory mapped registers:

       offset 0:  VALUE  : current value of the timer, incremented by the hardware
       offset 4:  CTRL   : bit 0 is used to start the timer
       offset 8:  PERIOD : timer value is reset when period is reached

   This driver works with the SoCLib timer component.

   For this driver initialization to succeed, any static instance of
   the device must specify the memory address range of the timer
   registers by using a resource entry:

   DEV_DECLARE_STATIC(my_dev4, "mydev4", 0, mydrv4_drv,
                      DEV_STATIC_RES_MEM(0xd3200000, 0xd3200020),
                      );
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

  /* This enters a crtical section up to the end of the scope. */
  LOCK_SPIN_IRQ_SCOPED(&dev->lock);

  /* read the current value of the timer from hardware */
  *value = endian_le32(cpu_mem_read_32(pv->addr + REG_VALUE));

  return 0;
}

static DEV_TIMER_CONFIG(mydrv_timer_config)
{
  if (cfg)
    {
      /* report driver and harware capabilities */
      cfg->freq = DEV_FREQ(0, 0, 0, 0);
      cfg->max = 0xffffffff;
      cfg->rev = 1;
      cfg->res = 1;
      cfg->cap = DEV_TIMER_CAP_KEEPVALUE | DEV_TIMER_CAP_HIGHRES
         | DEV_TIMER_CAP_TICKLESS;
    }

  if (res != 1)                 /* only support resolution == 1 */
    return -ERANGE;

  return 0;
}

#define mydrv_use dev_use_generic

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

  /* starts the timer harware */
  cpu_mem_write_32(addr + REG_CTRL, endian_le32(CTRL_RUNNING));

  return 0;
}

static DEV_CLEANUP(mydrv_cleanup)
{
  struct mydrv_context_s *pv = dev->drv_pv;

  /* stops the timer harware */
  cpu_mem_write_32(pv->addr + REG_CTRL, 0);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(mydrv4_drv, 0, "Simple timer driver example", mydrv,
               DRIVER_TIMER_METHODS(mydrv_timer)
               );

DRIVER_REGISTER(mydrv4_drv);
