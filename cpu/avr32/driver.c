/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2012

*/

#include <string.h>

#include <hexo/types.h>
#include <hexo/interrupt.h>
#include <hexo/local.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/class/cpu.h>
#include <device/class/timer.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#define AVR32_IRQ_COUNT 4

struct avr32_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_ep_s	sinks[AVR32_IRQ_COUNT];
#endif

#ifdef CONFIG_ARCH_SMP
  uint_fast8_t id;
  void *cls;            //< cpu local storage
#endif
};

/************************************************************************
        Interrupts controller driver part
************************************************************************/

#ifdef CONFIG_DEVICE_IRQ

static CPU_LOCAL struct device_s *avr32_icu_dev;

static CPU_INTERRUPT_HANDLER(avr32_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(avr32_icu_dev);
  struct avr32_dev_private_s  *pv = dev->drv_pv;

  if ( irq < AVR32_IRQ_COUNT ) {
    struct dev_irq_ep_s *sink = pv->sinks + irq;
    int_fast16_t id = 0;

    sink->process(sink, &id);
  }
}

static DEVICU_GET_ENDPOINT(avr32_icu_get_endpoint)
{
  struct device_s *dev = idev->dev;
  struct avr32_dev_private_s  *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK:
      if (id < AVR32_IRQ_COUNT)
        return pv->sinks + id;
    default:
      return NULL;
    }
}

static DEVICU_ENABLE_IRQ(avr32_icu_enable_irq)
{
  struct device_s *dev = idev->dev;
  struct avr32_dev_private_s  *pv = dev->drv_pv;

  // inputs are single wire, logical irq id must be 0
  if (irq_id > 0)
    return 0;

# ifdef CONFIG_ARCH_SMP
  if (!arch_cpu_irq_affinity_test(dev, dev_ep))
    return 0;

# else
  /* Enable irq line. On SMP platforms, all lines are already enabled on init. */
  uint_fast8_t icu_in_id = sink - pv->sinks;
  reg_t status;
  asm volatile ("mfsr	%0, 0" : "=r" (status));
  status &= ~(2 << icu_in_id);
  asm volatile ("mtsr	0, %0" :: "r" (status));
# endif

  return 1;
}

# ifndef CONFIG_ARCH_SMP
/* Disable irq line. On SMP platforms, all lines must remain enabled. */
static DEVICU_DISABLE_IRQ(avr32_icu_disable_irq)
{
  struct device_s *dev = idev->dev;
  struct avr32_dev_private_s  *pv = dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sinks;

  reg_t status;
  asm volatile ("mfsr	%0, 0" : "=r" (status));
  status |= 2 << icu_in_id;
  asm volatile ("mtsr	0, %0" :: "r" (status));
}
# endif

const struct driver_icu_s  avr32_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_endpoint  = avr32_icu_get_endpoint,
  .f_enable_irq   = avr32_icu_enable_irq,
# ifndef CONFIG_ARCH_SMP
  .f_disable_irq   = avr32_icu_disable_irq,
# endif
};

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

static DEVCPU_REG_INIT(avr32_cpu_reg_init)
{
  struct device_s *dev = cdev->dev;
  __unused__ struct avr32_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  assert(pv->id == cpu_id());

# error avr32 support has no CPU local storage register for SMP

# ifdef CONFIG_DEVICE_IRQ
  /* enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
  reg_t status;
  asm volatile ("mfsr	%0, 0" : "=r" (status));
  status &= ~0x1e;
  asm volatile ("mtsr	0, %0" :: "r" (status));
# endif
#endif

  CPU_LOCAL_SET(cpu_device, dev);
}

#ifdef CONFIG_ARCH_SMP
static DEVCPU_GET_STORAGE(avr32_cpu_get_storage)
{
  struct device_s *dev = cdev->dev;
  struct avr32_dev_private_s *pv = dev->drv_pv;
  return pv->cls;
}
#endif

const struct driver_cpu_s  avr32_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = avr32_cpu_reg_init,
#ifdef CONFIG_ARCH_SMP
  .f_get_storage   = avr32_cpu_get_storage,
#endif
};

/************************************************************************
        Timer driver part
************************************************************************/

static DEVTIMER_REQUEST(avr32_timer_request)
{
  return -ENOTSUP;
}

static DEVTIMER_START_STOP(avr32_timer_start_stop)
{
  return -ENOTSUP;
}

static DEVTIMER_GET_VALUE(avr32_timer_get_value)
{
  struct device_s *dev = tdev->dev;
  __unused__ struct avr32_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  if (pv->id != cpu_id())
    return -EIO;
#endif

  uint32_t ret;
  asm volatile ("mfsr %0, 264" : "=r" (ret)); // COUNT register
  *value = ret;

  return 0;
}

static DEVTIMER_RESOLUTION(avr32_timer_resolution)
{
  error_t err = 0;

  if (res)
    {
      if (*res != 0)
        err = -ENOTSUP;
      *res = 1;
    }

  if (max)
    *max = 0xffffffff;

  return err;
}

static const struct driver_timer_s  avr32_timer_drv =
{
  .class_          = DRIVER_CLASS_TIMER,
  .f_request       = avr32_timer_request,
  .f_start_stop    = avr32_timer_start_stop,
  .f_get_value     = avr32_timer_get_value,
  .f_resolution    = avr32_timer_resolution,
};

/************************************************************************/

static DEV_CLEANUP(avr32_cleanup);
static DEV_INIT(avr32_init);

static const struct devenum_ident_s  avr32_ids[] =
{
#ifdef CONFIG_FDT
  DEVENUM_FDTNAME_ENTRY("cpu:avr32"),
#endif
  { 0 }
};

const struct driver_s  avr32_drv =
{
  .desc           = "AVR32 processor",
  .id_table       = avr32_ids,

  .f_init         = avr32_init,
  .f_cleanup      = avr32_cleanup,

  .classes        = {
    &avr32_cpu_drv,
#ifdef CONFIG_DEVICE_IRQ
    &avr32_icu_drv,
#endif
    &avr32_timer_drv,
    0
  }
};

REGISTER_DRIVER(avr32_drv);

static DEV_INIT(avr32_init)
{
  struct avr32_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
#ifdef CONFIG_ARCH_SMP
    PRINTK_RET(-ENOENT, "avr32: device has no ID resource")
#endif
      ;

  if (sizeof(*pv))
    {
      /* FIXME allocation scope ? */
      pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

      if ( pv == NULL )
        return -ENOMEM;

      memset(pv, 0, sizeof(*pv));
      dev->drv_pv = pv;
    }

#ifdef CONFIG_ARCH_SMP
  /* allocate cpu local storage */
  pv->cls = arch_cpudata_alloc();
  pv->id = id;
  if (!pv->cls)
    goto err_mem;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* init avr32 irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, AVR32_IRQ_COUNT);

# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->cls, avr32_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->cls, avr32_irq_handler);
# else
  if (id == 0)
    {
      CPU_LOCAL_SET(avr32_icu_dev, dev);
      cpu_interrupt_sethandler(avr32_irq_handler);
    }
# endif
#endif

  dev->drv = &avr32_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
#ifdef CONFIG_ARCH_SMP
 err_mem:
  if (sizeof(*pv))
    mem_free(pv);
  return -1;
#endif
}

static DEV_CLEANUP(avr32_cleanup)
{
  struct avr32_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
  reg_t status;
  asm volatile ("mustr	%0" : "=r" (status));
  status |= 0x1e;
  asm volatile ("musfr	%0" :: "r" (status));
# endif
  /* detach avr32 irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, AVR32_IRQ_COUNT);
#endif

  if (sizeof(*pv))
    mem_free(pv);
}

