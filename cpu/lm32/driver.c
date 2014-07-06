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
#include <device/resources.h>
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

  struct cpu_tree_s node;
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

static DEVICU_GET_ENDPOINT(lm32_icu_get_endpoint)
{
  struct device_s *dev = idev->dev;
  struct lm32_dev_private_s  *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK:
      if (id < CONFIG_CPU_LM32_IRQ_COUNT)
        return pv->sinks + id;
    default:
      return NULL;
    }
}

static DEVICU_ENABLE_IRQ(lm32_icu_enable_irq)
{
  struct device_s *dev = idev->dev;
  struct lm32_dev_private_s  *pv = dev->drv_pv;

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
  asm volatile ("rcsr	%0, IM" : "=r" (status));
  status |= 1 << icu_in_id;
  asm volatile ("wcsr	IM, %0" :: "r" (status));
# endif

  return 1;
}

# ifndef CONFIG_ARCH_SMP
/* Disable irq line. On SMP platforms, all lines must remain enabled. */
static DEVICU_DISABLE_IRQ(lm32_icu_disable_irq)
{
  struct device_s *dev = idev->dev;
  struct lm32_dev_private_s  *pv = dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sinks;

  reg_t status;
  asm volatile ("rcsr	%0, IM" : "=r" (status));
  status &= ~(1 << icu_in_id);
  asm volatile ("wcsr	IM, %0" :: "r" (status));
}
# endif

const struct driver_icu_s  lm32_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_endpoint  = lm32_icu_get_endpoint,
  .f_enable_irq   = lm32_icu_enable_irq,
# ifndef CONFIG_ARCH_SMP
  .f_disable_irq   = lm32_icu_disable_irq,
# endif
};

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

static DEVCPU_REG_INIT(lm32_cpu_reg_init)
{
  struct device_s *dev = cdev->dev;
  __unused__ struct lm32_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  assert(pv->node.cpu_id == cpu_id());

# error lm32 support has no CPU local storage register for SMP

# ifdef CONFIG_DEVICE_IRQ
  /* Enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
  asm volatile ("wcsr	IM, %0" :: "r" ((1 << CONFIG_CPU_LM32_IRQ_COUNT)-1));
# endif
#endif

  CPU_LOCAL_SET(cpu_device, dev);
}

#ifdef CONFIG_ARCH_SMP
static DEVCPU_GET_NODE(lm32_cpu_get_node)
{
  struct device_s *dev = cdev->dev;
  struct lm32_dev_private_s *pv = dev->drv_pv;
  return &pv->node;
}
#endif

const struct driver_cpu_s  lm32_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = lm32_cpu_reg_init,
#ifdef CONFIG_ARCH_SMP
  .f_get_node   = lm32_cpu_get_node,
#endif
};

/************************************************************************
        Timer driver part
************************************************************************/

#ifdef CONFIG_CPU_LM32_TIMER_CYCLECOUNTER

static DEVTIMER_START_STOP(lm32_timer_start_stop)
{
  return 0;
}

static DEVTIMER_GET_VALUE(lm32_timer_get_value)
{
  struct device_s *dev = tdev->dev;
  __unused__ struct lm32_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  if (pv->node.cpu_id != cpu_id())
    return -EIO;
#endif

  uint32_t ret;
  asm volatile ("rcsr %0, CC" : "=r" (ret));
  *value = ret;

  return 0;
}

static DEVTIMER_RESOLUTION(lm32_timer_resolution)
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

static const struct driver_timer_s  lm32_timer_drv =
{
  .class_          = DRIVER_CLASS_TIMER,
  .f_start_stop    = lm32_timer_start_stop,
  .f_get_value     = lm32_timer_get_value,
  .f_get_freq      = dev_timer_drv_get_freq,
  .f_resolution    = lm32_timer_resolution,
  .f_request       = (devtimer_request_t*)&dev_driver_notsup_fcn,
  .f_cancel        = (devtimer_request_t*)&dev_driver_notsup_fcn,
};

#endif

/************************************************************************/

static DEV_CLEANUP(lm32_cleanup);
static DEV_INIT(lm32_init);

static const struct devenum_ident_s  lm32_ids[] =
{
#ifdef CONFIG_LIBFDT
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
#ifdef CONFIG_CPU_LM32_TIMER_CYCLECOUNTER
    &lm32_timer_drv,
#endif
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
    PRINTK_RET(-ENOENT, "lm32: device has no ID resource")
      ;

  pv = mem_alloc_cpu(sizeof (*pv), (mem_scope_sys), cpu);
  if (pv == NULL)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if (cpu_tree_node_init(&pv->node, id, dev))
    goto err_pv;

#ifdef CONFIG_DEVICE_IRQ
  /* init lm32 irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, CONFIG_CPU_LM32_IRQ_COUNT,
#ifdef CONFIG_CPU_LM32_SOCLIB
                       DEV_IRQ_SENSE_HIGH_LEVEL
#else
                       DEV_IRQ_SENSE_LOW_LEVEL
#endif
                       );

# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->node.cls, lm32_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->node.cls, lm32_irq_handler);
# else
  if (id == CONFIG_ARCH_BOOTSTRAP_CPU_ID)
    {
      CPU_LOCAL_SET(lm32_icu_dev, dev);
      cpu_interrupt_sethandler(lm32_irq_handler);
    }
# endif
#endif

  if (cpu_tree_insert(&pv->node))
    goto err_node;

  dev->drv = &lm32_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_node:
  cpu_tree_node_cleanup(&pv->node);
 err_pv:
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

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);
}

