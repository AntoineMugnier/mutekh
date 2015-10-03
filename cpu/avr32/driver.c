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
#include <device/class/clock.h>
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
#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s clk_ep;
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

  switch (accessor->number)
    {
    case 0: {          /* cycle counter */
      if (res > 1)
        err = -ERANGE;
      if (cfg)
        {
          cfg->freq = pv->freq;
          cfg->acc = DEV_FREQ_ACC_INVALID;
          cfg->max = 0xffffffff;
          cfg->cap = DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_KEEPVALUE | DEV_TIMER_CAP_TICKLESS;
#ifdef CONFIG_DEVICE_CLOCK
          cfg->cap |= DEV_TIMER_CAP_VARFREQ;
          cfg->rev = pv->timer_rev;
#else
          cfg->rev = 1;
#endif
          cfg->res = 1;
        }
      break;
    }

    default:
      err = -ENOTSUP;
    }

  return err;
}

#endif

/************************************************************************/

static DEV_CLEANUP(avr32_cleanup);
static DEV_INIT(avr32_init);

#ifdef CONFIG_DEVICE_CLOCK
static DEV_CLOCK_SINK_CHANGED(arv32_clk_changed)
{
  struct device_s *dev = ep->base.dev;
  struct arv32_dev_private_s *pv = dev->drv_pv;
  LOCK_SPIN_IRQ(&dev->lock);
  pv->freq = *freq;
  pv->timer_rev += 2;
  LOCK_RELEASE_IRQ(&dev->lock);
}
#endif

static DEV_USE(arv32_use)
{
  if (accessor->number > 0)
    return -ENOTSUP;

  switch (accessor->api->class_)
    {
    case DRIVER_CLASS_TIMER:
      return 0;

    case DRIVER_CLASS_CPU:
    case DRIVER_CLASS_ICU:
      switch (op)
        {
        case DEV_USE_GET_ACCESSOR:
        case DEV_USE_PUT_ACCESSOR:
          return 0;
        default:
          break;
        }
    default:
      break;
    }

  return -ENOTSUP;
}

#define avr32_timer_request (dev_timer_request_t*)&dev_driver_notsup_fcn
#define avr32_timer_cancel  (dev_timer_cancel_t*)&dev_driver_notsup_fcn

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

static DEV_INIT(avr32_init)
{
  struct avr32_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

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

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_init(dev, &pv->clk_ep, &arv32_clk_changed);

  struct dev_clock_link_info_s ckinfo;
  if (dev_clock_sink_link(dev, &pv->clk_ep, &ckinfo, 0, 0))
    goto err_node;
  pv->freq = ckinfo.freq;
  pv->timer_rev = 1;

  if (dev_clock_sink_hold(&pv->clk_ep, NULL))
    goto err_clku;
#else
  if (device_get_res_freq(dev, &pv->freq, 0))
    pv->freq = DEV_FREQ_INVALID;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* init avr32 irq sink end-points */
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

  dev->drv = &avr32_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_clk:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
#endif
 err_clku:
#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif
 err_node:
  cpu_tree_node_cleanup(&pv->node);
 err_pv:
  mem_free(pv);
  return -1;
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

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);
}

