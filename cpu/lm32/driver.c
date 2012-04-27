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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2011

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

struct lm32_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_ep_s	sinks[CONFIG_CPU_LM32_IRQ_COUNT];
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

static CPU_LOCAL struct device_s *lm32_icu_dev;

static CPU_INTERRUPT_HANDLER(lm32_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(lm32_icu_dev);
  struct lm32_dev_private_s  *pv = dev->drv_pv;

  if ( irq < CONFIG_CPU_LM32_IRQ_COUNT ) {
    struct dev_irq_ep_s *sink = pv->sinks + irq;
    int_fast16_t id = 0;

    sink->process(sink, &id);
  }
}

#ifdef CONFIG_HEXO_IPI
static DEVICU_SETUP_IPI_EP(lm32_icu_setup_ipi_ep)
{
  abort(); // FIXME
  return -1;
}
#endif

static DEVICU_DISABLE_SINK(lm32_icu_disable_sink)
{
# ifndef CONFIG_ARCH_SMP
  struct device_s *dev = idev->dev;
  struct lm32_dev_private_s  *pv = dev->drv_pv;
  uint_fast8_t i = sink - pv->sinks;

  /* Disable irq line. On SMP platforms, all lines must remain enabled. */
  reg_t status;
  asm volatile ("rcsr	%0, IM" : "=r" (status));
  status &= ~(1 << i);
  asm volatile ("wcsr	IM, %0" :: "r" (status));
# endif
}

static DEVICU_GET_SINK(lm32_icu_get_sink)
{
  struct device_s *dev = idev->dev;
  struct lm32_dev_private_s  *pv = dev->drv_pv;

  if (icu_in_id >= CONFIG_CPU_LM32_IRQ_COUNT)
    return NULL;

# ifndef CONFIG_ARCH_SMP
  /* Enable irq line. On SMP platforms, all lines are already enabled on init. */
  reg_t status;
  asm volatile ("rcsr	%0, IM" : "=r" (status));
  status |= 1 << icu_in_id;
  asm volatile ("wcsr	IM, %0" :: "r" (status));
# endif

  return pv->sinks + icu_in_id;
}

const struct driver_icu_s  lm32_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_sink     = lm32_icu_get_sink,
  .f_disable_sink = lm32_icu_disable_sink,
#ifdef CONFIG_HEXO_IPI
  .f_setup_ipi_ep = lm32_icu_setup_ipi_ep,
#endif
};

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

static DEVCPU_REG_INIT(lm32_cpu_reg_init)
{
  struct device_s *dev = cdev->dev;
  struct lm32_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  assert(pv->id == cpu_id());

# error lm32 support has no CPU local storage register for SMP

# ifdef CONFIG_DEVICE_IRQ
  /* Enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
  asm volatile ("wcsr	IM, %0" :: "r" ((1 << CONFIG_CPU_LM32_IRQ_COUNT)-1));
# endif
#endif

  CPU_LOCAL_SET(cpu_device, dev);
}

#ifdef CONFIG_ARCH_SMP
static DEVCPU_GET_STORAGE(lm32_cpu_get_storage)
{
  struct device_s *dev = cdev->dev;
  struct lm32_dev_private_s *pv = dev->drv_pv;
  return pv->cls;
}
#endif

const struct driver_cpu_s  lm32_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = lm32_cpu_reg_init,
#ifdef CONFIG_ARCH_SMP
  .f_get_storage   = lm32_cpu_get_storage,
#endif
};

/************************************************************************
        Timer driver part
************************************************************************/

static DEVTIMER_GET_VALUE(lm32_timer_get_value)
{
  struct device_s *dev = tdev->dev;
  struct lm32_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  assert(pv->id == cpu_id());
#endif

  uint32_t ret;
  asm volatile ("rcsr %0, CC" : "=r" (ret));
  return ret;
}

static DEVTIMER_RESOLUTION(lm32_timer_resolution)
{
  error_t err = 0;

  if (res)
    {
      if (*res != 0)
        err = -ENOTSUP;
      *res = DEVTIMER_RES_FIXED_POINT(1.0);
    }

  if (max)
    *max = 0xffffffff;
}

static const struct driver_timer_s  lm32_timer_drv =
{
  .class_          = DRIVER_CLASS_TIMER,
  .f_get_value     = lm32_timer_get_value,
  .f_resolution    = lm32_timer_resolution,
};

/************************************************************************/

static DEV_CLEANUP(lm32_cleanup);
static DEV_INIT(lm32_init);

static const struct devenum_ident_s  lm32_ids[] =
{
#ifdef CONFIG_FDT
  DEVENUM_FDTNAME_ENTRY("cpu:lm32"),
#endif
  { 0 }
};

const struct driver_s  lm32_drv =
{
  .desc           = "LM32 processor",
  .id_table       = lm32_ids,

  .f_init         = lm32_init,
  .f_cleanup      = lm32_cleanup,

  .classes        = {
    &lm32_cpu_drv,
#ifdef CONFIG_DEVICE_IRQ
    &lm32_icu_drv,
#endif
    &lm32_timer_drv,
    0
  }
};

REGISTER_DRIVER(lm32_drv);

static DEV_INIT(lm32_init)
{
  struct lm32_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
#ifdef CONFIG_ARCH_SMP
    PRINTK_RET(-ENOENT, "lm32: device has no ID resource")
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
  /* init lm32 irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, CONFIG_CPU_LM32_IRQ_COUNT, NULL);

# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->cls, lm32_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->cls, lm32_irq_handler);
# else
  if (id == 0)
    {
      CPU_LOCAL_SET(lm32_icu_dev, dev);
      cpu_interrupt_sethandler(lm32_irq_handler);
    }
# endif
#endif

  dev->drv = &lm32_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(lm32_cleanup)
{
  struct lm32_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
  asm volatile ("wcsr	IM, %0" :: "r" (0));
# endif
  /* detach lm32 irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, CONFIG_CPU_LM32_IRQ_COUNT);
#endif

  if (sizeof(*pv))
    mem_free(pv);
}

