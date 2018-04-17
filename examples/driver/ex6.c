/*
   Additional topics covered by this example:
     - Use of an hardware interrupt handler

   This is an modified version of the previous example.

   In this example, the timer API do not expose the value of the
   hardware timer directly. Instead it use the hardware timer to
   generate an interrupt which then increase a software counter.  The
   software counter is exposed as the timer value.

   In addition to the memory resource entry, some resources are needed
   to specifies the interrupt controller and the line used. A platform
   dependent example of static device instantiation is provided here:

   DEV_DECLARE_STATIC(my_dev6, "mydev6", 0, mydrv6_drv,
                     DEV_STATIC_RES_MEM(0xd3200000, 0xd3200020),
                     DEV_STATIC_RES_DEV_ICU("/icu"),
                     DEV_STATIC_RES_IRQ(0, 2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                    );

   Unlike the memory resource which is directly queried by the driver,
   all the interrupt related resource queries as well as related
   initializations of the interrupt controller are handled by an
   helper function provided by libdevice.
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
#include <device/irq.h>

enum {
  REG_VALUE    = 0,
  REG_CTRL     = 4,
  REG_PERIOD   = 8,
  REG_RESETIRQ = 12
};

enum {
  CTRL_RUNNING = 1,
  CTRL_IRQEN   = 2,
};

DRIVER_PV(struct mydrv_context_s
{
  uintptr_t addr;

  /* our software timer value */
  uint32_t ticks;

  /* Interrupt endpoint. libdevice needs this in order to setup the
     link to the interrupt controller device. */
  struct dev_irq_src_s irq_eps;
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

  /* we do not expose the hardware timer value,
     return the software counter instead. */
  *value = pv->ticks;

  return 0;
}

static DEV_TIMER_CONFIG(mydrv_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct mydrv_context_s *pv = dev->drv_pv;

  if (cfg)
    {
      cfg->freq = DEV_FREQ(0, 0, 0, 0);
      cfg->max = __MAXOF_TYPE(pv->ticks);
      cfg->rev = 1;
      cfg->res = 1;
      cfg->cap = DEV_TIMER_CAP_KEEPVALUE;
    }

  if (res != 1)
    return -ERANGE;

  return 0;
}

/* This is called when the hardware device raise its irq line. With
   our timer, this occurs when the harware timer value has reached 0
   and just wrapped. */
static DEV_IRQ_SRC_PROCESS(mydrv_timer_irq)
{
  struct device_s *dev = ep->base.dev;
  struct mydrv_context_s *pv = dev->drv_pv;

  /* enter critical section */
  lock_spin(&dev->lock);

  /* Increment our software timer value. */
  pv->ticks++;

  /* This hardware requires us to acknowledge the handling of the irq */
  cpu_mem_write_32(pv->addr + REG_RESETIRQ, 0);

  lock_release(&dev->lock);
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

  /* This helper setups the link to the interrupt controller as
     specified in the device resources and install our interrupt
     handler function. */
  device_irq_source_init(dev, &pv->irq_eps, 1, mydrv_timer_irq);
  if (device_irq_source_link(dev, &pv->irq_eps, 1, 1))
    {
      mem_free(pv);
      return -EINVAL;
    }

  /* initial software timer value */
  pv->ticks = 0;

  /* generate an irq evry 100000 cycles */
  cpu_mem_write_32(addr + REG_PERIOD, endian_le32(100000));

  /* starts the timer hardware and enable interrupts */
  cpu_mem_write_32(addr + REG_CTRL, endian_le32(CTRL_RUNNING | CTRL_IRQEN));

  return 0;
}

static DEV_CLEANUP(mydrv_cleanup)
{
  struct mydrv_context_s *pv = dev->drv_pv;

  /* stops the timer hardware and disable interrupts */
  cpu_mem_write_32(pv->addr + REG_CTRL, 0);

  /* release our interrupt link */
  device_irq_source_unlink(dev, &pv->irq_eps, 1);

  mem_free(pv);
  return 0;
}

DRIVER_DECLARE(mydrv6_drv, 0, "Simple timer driver example with irqs", mydrv,
               DRIVER_TIMER_METHODS(mydrv_timer)
               );

DRIVER_REGISTER(mydrv6_drv);
