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
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/class/cpu.h>
#include <device/class/timer.h>
#include <device/clock.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#define AVR32_IRQ_COUNT 4

struct avr32_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_sink_s	sinks[AVR32_IRQ_COUNT];
#endif

  struct cpu_tree_s node;
  struct dev_freq_s freq;
  struct dev_clock_sink_ep_s clk_ep;
#ifdef CONFIG_DEVICE_CLOCK
  dev_timer_cfgrev_t timer_rev;
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
    struct dev_irq_sink_s *sink = pv->sinks + irq;
    device_irq_sink_process(sink, 0);
  }
}

static DEV_ICU_GET_SINK(avr32_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct avr32_dev_private_s  *pv = dev->drv_pv;

  if (id < AVR32_IRQ_COUNT)
    return pv->sinks + id;
  return NULL;
}

static DEV_IRQ_SINK_UPDATE(avr32_icu_sink_update)
{
#ifndef CONFIG_ARCH_SMP
  struct device_s *dev = sink->base.dev;
  struct nios2_dev_private_s  *pv = dev->drv_pv;

  reg_t status;
  asm volatile ("mfsr	%0, 0" : "=r" (status));

  switch (sense)
    {
    case DEV_IRQ_SENSE_NONE:
      status &= ~(2 << sink_id);
      break;
    case DEV_IRQ_SENSE_ID_BUS:
      status |= 2 << sink_id;
      break;
    default:
      return;
    }

  asm volatile ("mtsr	0, %0" :: "r" (status));
#endif
}

#define avr32_icu_link device_icu_dummy_link

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

static DEV_CPU_REG_INIT(avr32_cpu_reg_init)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct avr32_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  assert(pv->node.cpu_id == cpu_id());

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
static DEV_CPU_GET_NODE(avr32_cpu_get_node)
{
  struct device_s *dev = accessor->dev;
  struct avr32_dev_private_s *pv = dev->drv_pv;
  return &pv->node;
}
#endif

/************************************************************************
        Timer driver part
************************************************************************/

#ifdef CONFIG_CPU_AVR32_TIMER_CYCLECOUNTER

static DEV_TIMER_GET_VALUE(avr32_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct avr32_dev_private_s *pv = dev->drv_pv;

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
  asm volatile ("mfsr %0, 264" : "=r" (ret)); // COUNT register
  *value = ret;

  return 0;
}

static DEV_TIMER_CONFIG(arv32_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct arv32_dev_private_s *pv = dev->drv_pv;
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


static DEV_USE(avr32_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_NOTIFY: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct avr32_dev_private_s *pv = dev->drv_pv;
      pv->freq = chg->freq;
      pv->timer_rev += 2;
      return 0;
    }
#endif
    default:
      return dev_use_generic(param, op);
    }
}

#define avr32_timer_request (dev_timer_request_t*)&dev_driver_notsup_fcn
#define avr32_timer_cancel  (dev_timer_cancel_t*)&dev_driver_notsup_fcn

static DEV_INIT(avr32_init)
{
  struct avr32_dev_private_s  *pv;


  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
    PRINTK_RET(-ENOENT, "avr32: device has no ID resource")
      ;

  /* FIXME allocation scope ? */
  pv = mem_alloc_cpu(sizeof (*pv), (mem_scope_sys), id);

  if ( pv == NULL )
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if (cpu_tree_node_init(&pv->node, id, dev))
    goto err_pv;

  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, DEV_CLOCK_EP_SINK_NOTIFY |
                         DEV_CLOCK_EP_POWER_CLOCK | DEV_CLOCK_EP_SINK_SYNC, &pv->freq))
    goto err_node;

#ifdef CONFIG_DEVICE_CLOCK
  pv->timer_rev = 1;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* init avr32 irq sink endpoints */
  device_irq_sink_init(dev, pv->sinks, AVR32_IRQ_COUNT,
                       &avr32_icu_sink_update, DEV_IRQ_SENSE_ID_BUS);

# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->node.cls, avr32_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->node.cls, avr32_irq_handler);
# else
  if (id == CONFIG_ARCH_BOOTSTRAP_CPU_ID)
    {
      CPU_LOCAL_SET(avr32_icu_dev, dev);
      cpu_interrupt_sethandler(avr32_irq_handler);
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

static DEV_CLEANUP(avr32_cleanup)
{
  struct avr32_dev_private_s *pv = dev->drv_pv;

  dev_drv_clock_cleanup(dev, &pv->clk_ep);

  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(avr32_drv, DRIVER_FLAGS_EARLY_INIT, "AVR32 processor", avr32,
#ifdef CONFIG_DEVICE_IRQ
               DRIVER_ICU_METHODS(avr32_icu),
#endif
#ifdef CONFIG_CPU_AVR32_TIMER_CYCLECOUNTER
               DRIVER_TIMER_METHODS(avr32_timer),
#endif
               DRIVER_CPU_METHODS(avr32_cpu));

DRIVER_REGISTER(avr32_drv,
                DEV_ENUM_FDTNAME_ENTRY("cpu:avr32"));

