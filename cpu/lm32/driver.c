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
#include <device/clock.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#ifdef CONFIG_CPU_LM32_SOCLIB
# define LM32_IRQ_SENSE_MODE DEV_IRQ_SENSE_HIGH_LEVEL
#else
# define LM32_IRQ_SENSE_MODE DEV_IRQ_SENSE_LOW_LEVEL
#endif

DRIVER_PV(struct lm32_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_sink_s	sinks[CONFIG_CPU_LM32_IRQ_COUNT];
#endif

  struct cpu_tree_s node;
  struct dev_freq_s freq;
  struct dev_clock_sink_ep_s clk_ep;
#ifdef CONFIG_DEVICE_CLOCK
  dev_timer_cfgrev_t timer_rev;
#endif
});

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
    struct dev_irq_sink_s *sink = pv->sinks + irq;
    device_irq_sink_process(sink, 0);
  }
}

static DEV_ICU_GET_SINK(lm32_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct lm32_dev_private_s  *pv = dev->drv_pv;

  if (id < CONFIG_CPU_LM32_IRQ_COUNT)
    return pv->sinks + id;
  return NULL;
}

static DEV_IRQ_SINK_UPDATE(lm32_icu_sink_update)
{
#ifndef CONFIG_ARCH_SMP
  struct device_s *dev = sink->base.dev;
  struct lm32_dev_private_s  *pv = dev->drv_pv;
  uint_fast8_t sink_id = sink - pv->sinks;

  reg_t status;
  asm volatile ("rcsr	%0, IM" : "=r" (status));

  switch (sense)
    {
    case DEV_IRQ_SENSE_NONE:
      status &= ~(1 << sink_id);
      break;
    case LM32_IRQ_SENSE_MODE:
      status |= 1 << sink_id;
      break;
    default:
      return;
    }

  asm volatile ("wcsr	IM, %0" :: "r" (status));
#endif
}

#define lm32_icu_link device_icu_dummy_link

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

static DEV_CPU_REG_INIT(lm32_cpu_reg_init)
{
  struct device_s *dev = accessor->dev;
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
static DEV_CPU_GET_NODE(lm32_cpu_get_node)
{
  struct device_s *dev = accessor->dev;
  struct lm32_dev_private_s *pv = dev->drv_pv;
  return &pv->node;
}
#endif

/************************************************************************
        Timer driver part
************************************************************************/

#ifdef CONFIG_CPU_LM32_TIMER_CYCLECOUNTER

static DEV_TIMER_GET_VALUE(lm32_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct lm32_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_CLOCK
  if (rev && rev != pv->timer_rev)
#else
  if (rev && rev != 1)
#endif
    return -EAGAIN;

#ifdef CONFIG_ARCH_SMP
  if (pv->node.cpu_id != cpu_id())
    return -EIO;
#endif

  uint32_t ret;
  asm volatile ("rcsr %0, CC" : "=r" (ret));
  *value = ret;

  return 0;
}

static DEV_TIMER_CONFIG(lm32_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct lm32_dev_private_s *pv = dev->drv_pv;
  error_t err = 0;

      if (res > 1)
        err = -ERANGE;
      if (cfg)
        {
          cfg->freq = pv->freq;
          cfg->max = 0xffffffff;
          cfg->cap = DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_KEEPVALUE | DEV_TIMER_CAP_TICKLESS;
#ifdef CONFIG_DEVICE_CLOCK
          if (pv->clk_ep.flags & DEV_CLOCK_EP_VARFREQ)
            cfg->cap |= DEV_TIMER_CAP_VARFREQ;
          cfg->rev = pv->timer_rev;
#else
          cfg->rev = 1;
#endif
          cfg->res = 1;
        }

  return err;
}

#endif

/************************************************************************/


static DEV_USE(lm32_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct lm32_dev_private_s *pv = dev->drv_pv;
      pv->freq = chg->freq;
      pv->timer_rev += 2;
      return 0;
    }
#endif
    default:
      return dev_use_generic(param, op);
    }
}

#define lm32_timer_request (dev_timer_request_t*)&dev_driver_notsup_fcn
#define lm32_timer_cancel  (dev_timer_cancel_t*)&dev_driver_notsup_fcn

static DEV_INIT(lm32_init)
{
  struct lm32_dev_private_s  *pv;


  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
    PRINTK_RET(-ENOENT, "lm32: device has no ID resource")
      ;

  pv = mem_alloc_cpu(sizeof (*pv), (mem_scope_sys), id);
  if (pv == NULL)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if (cpu_tree_node_init(&pv->node, id, dev))
    goto err_pv;

  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, DEV_CLOCK_EP_FREQ_NOTIFY |
                         DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_GATING_SYNC, &pv->freq))
    goto err_node;

#ifdef CONFIG_DEVICE_CLOCK
  pv->timer_rev = 1;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* init lm32 irq sink endpoints */
  device_irq_sink_init(dev, pv->sinks, CONFIG_CPU_LM32_IRQ_COUNT,
                       &lm32_icu_sink_update, LM32_IRQ_SENSE_MODE);

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
    goto err_clk;


  return 0;

 err_clk:
  dev_drv_clock_cleanup(dev, &pv->clk_ep);
 err_node:
  cpu_tree_node_cleanup(&pv->node);
 err_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(lm32_cleanup)
{
  struct lm32_dev_private_s *pv = dev->drv_pv;

  dev_drv_clock_cleanup(dev, &pv->clk_ep);

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(lm32_drv, DRIVER_FLAGS_EARLY_INIT, "LM32 processor", lm32,
#ifdef CONFIG_DEVICE_IRQ
               DRIVER_ICU_METHODS(lm32_icu),
#endif
#ifdef CONFIG_CPU_LM32_TIMER_CYCLECOUNTER
               DRIVER_TIMER_METHODS(lm32_timer),
#endif
               DRIVER_CPU_METHODS(lm32_cpu));

DRIVER_REGISTER(lm32_drv,
                DEV_ENUM_FDTNAME_ENTRY("cpu:lm32"));

