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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2009

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

struct ppc_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
#define ICU_PPC_MAX_VECTOR	1
  struct dev_irq_ep_s	sinks[ICU_PPC_MAX_VECTOR];
#endif

  struct cpu_tree_s node;
  struct dev_freq_s freq;
#ifdef CONFIG_DEVICE_CLOCK
  struct dev_clock_sink_ep_s clk_ep;
#endif
};

/************************************************************************
        Interrupts controller driver part
************************************************************************/

#ifdef CONFIG_DEVICE_IRQ

static CPU_LOCAL struct device_s *ppc_icu_dev;

static CPU_INTERRUPT_HANDLER(ppc_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(ppc_icu_dev);
  struct ppc_dev_private_s  *pv = dev->drv_pv;

  if ( irq < ICU_PPC_MAX_VECTOR ) {
    struct dev_irq_ep_s *sink = pv->sinks + irq;
    int_fast16_t id = 0;

    sink->process(sink, &id);
  }
}

static DEV_ICU_GET_ENDPOINT(ppc_icu_get_endpoint)
{
  struct device_s *dev = accessor->dev;
  struct ppc_dev_private_s  *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK:
      if (id < ICU_PPC_MAX_VECTOR)
        return pv->sinks + id;
    default:
      return NULL;
    }
}

static DEV_ICU_ENABLE_IRQ(ppc_icu_enable_irq)
{
  __unused__ struct device_s *dev = accessor->dev;

  // inputs are single wire, logical irq id must be 0
  if (irq_id > 0)
    return 0;

# ifdef CONFIG_ARCH_SMP
  if (!arch_cpu_irq_affinity_test(dev, dev_ep))
    return 0;

# else
  /* Enable irq line. On SMP platforms, all lines are already enabled on init. */
# endif

  return 1;
}

const struct driver_icu_s  ppc_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_endpoint  = ppc_icu_get_endpoint,
  .f_enable_irq    = ppc_icu_enable_irq,
};

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

static DEV_CPU_REG_INIT(ppc_cpu_reg_init)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct ppc_dev_private_s *pv = dev->drv_pv;

  /* set exception vector */
  extern __ldscript_symbol_t CPU_NAME_DECL(exception_vector);
  asm volatile("mtevpr %0" : : "r"(&CPU_NAME_DECL(exception_vector)));

#ifdef CONFIG_ARCH_SMP
  assert(pv->node.cpu_id == cpu_id());

  asm volatile("mtspr 0x115, %0" : : "r" (pv->node.cls)); /* SPRG5 is cls */

# ifdef CONFIG_DEVICE_IRQ
  /* Enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
# endif
#endif

  CPU_LOCAL_SET(cpu_device, dev);
}

#ifdef CONFIG_ARCH_SMP
static DEV_CPU_GET_NODE(ppc_cpu_get_node)
{
  struct device_s *dev = accessor->dev;
  struct ppc_dev_private_s *pv = dev->drv_pv;
  return &pv->node;
}
#endif

const struct driver_cpu_s  ppc_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = ppc_cpu_reg_init,
#ifdef CONFIG_ARCH_SMP
  .f_get_node   = ppc_cpu_get_node,
#endif
};

/************************************************************************
        Timer driver part
************************************************************************/

#ifdef CONFIG_CPU_PPC_TIMER_CYCLECOUNTER

static DEV_TIMER_START_STOP(ppc_timer_start_stop)
{
  return 0;
}

static DEV_TIMER_GET_VALUE(ppc_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct ppc_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  if(pv->node.cpu_id != cpu_id())
    return -EIO;
#endif

  uint32_t      result;
  asm volatile ("mftbl %0" : "=r" (result));
  *value = result;

  return 0;
}

static DEV_TIMER_RESOLUTION(ppc_timer_resolution)
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

static const struct driver_timer_s  ppc_timer_drv =
{
  .class_          = DRIVER_CLASS_TIMER,
  .f_start_stop    = ppc_timer_start_stop,
  .f_get_value     = ppc_timer_get_value,
  .f_get_freq      = dev_timer_drv_get_freq,
  .f_resolution    = ppc_timer_resolution,
  .f_request       = (dev_timer_request_t*)&dev_driver_notsup_fcn,
  .f_cancel        = (dev_timer_request_t*)&dev_driver_notsup_fcn,
};

#endif

/************************************************************************/

static DEV_CLEANUP(ppc_cleanup);
static DEV_INIT(ppc_init);

#ifdef CONFIG_DEVICE_CLOCK
static DEV_CLOCK_SINK_CHANGED(ppc_clk_changed)
{
  struct device_s *dev = ep->dev;
  struct ppc_dev_private_s *pv = dev->drv_pv;
  LOCK_SPIN_IRQ(&dev->lock);
  pv->freq = *freq;
  LOCK_RELEASE_IRQ(&dev->lock);
}
#endif

static const struct dev_enum_ident_s  ppc_ids[] =
{
#ifdef CONFIG_LIBFDT
  DEV_ENUM_FDTNAME_ENTRY("cpu:ppc"),
  DEV_ENUM_FDTNAME_ENTRY("cpu:powerpc"),
#endif
  { 0 }
};

const struct driver_s  ppc_drv =
{
  .desc           = "PowerPC processor",
  .id_table       = ppc_ids,

  .f_init         = ppc_init,
  .f_cleanup      = ppc_cleanup,

  .classes        = {
    &ppc_cpu_drv,
#ifdef CONFIG_DEVICE_IRQ
    &ppc_icu_drv,
#endif
#ifdef CONFIG_CPU_PPC_TIMER_CYCLECOUNTER
    &ppc_timer_drv,
#endif
    0
  }
};

REGISTER_DRIVER(ppc_drv);

static DEV_INIT(ppc_init)
{
  struct ppc_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  /* get processor device id specifed in resources */
  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
    PRINTK_RET(-ENOENT, "ppc: device has no ID resource")
      ;

  /* allocate device private data */
  pv = mem_alloc_cpu(sizeof (*pv), (mem_scope_sys), id);

  if ( pv == NULL )
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if (cpu_tree_node_init(&pv->node, id, dev))
    goto err_pv;

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_init(dev, &pv->clk_ep, &ppc_clk_changed);

  struct dev_clock_link_info_s ckinfo;
  if (dev_clock_sink_link(dev, &pv->clk_ep, &ckinfo, 0, 0))
    goto err_node;
  pv->freq = ckinfo.freq;

  if (dev_clock_sink_hold(&pv->clk_ep, NULL))
    goto err_clku;
#else
  if (device_get_res_freq(dev, &pv->freq, 0))
    pv->freq = DEV_FREQ_INVALID;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* init ppc irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_PPC_MAX_VECTOR, DEV_IRQ_SENSE_HIGH_LEVEL);

# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->node.cls, ppc_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->node.cls, ppc_irq_handler);
# else
  if (id == CONFIG_ARCH_BOOTSTRAP_CPU_ID)
    {
      CPU_LOCAL_SET(ppc_icu_dev, dev);
      cpu_interrupt_sethandler(ppc_irq_handler);
    }
# endif
#endif

  if (cpu_tree_insert(&pv->node))
    goto err_clk;

  dev->drv = &ppc_drv;
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

static DEV_CLEANUP(ppc_cleanup)
{
  struct ppc_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
# endif
  /* detach ppc irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, ICU_PPC_MAX_VECTOR);
#endif

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);
}

