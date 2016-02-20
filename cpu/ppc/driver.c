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
  struct dev_irq_sink_s	sinks[ICU_PPC_MAX_VECTOR];
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

static CPU_LOCAL struct device_s *ppc_icu_dev;

static CPU_INTERRUPT_HANDLER(ppc_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(ppc_icu_dev);
  struct ppc_dev_private_s  *pv = dev->drv_pv;

  if ( irq < ICU_PPC_MAX_VECTOR ) {
    struct dev_irq_sink_s *sink = pv->sinks + irq;
    device_irq_sink_process(sink, 0);
  }
}

static DEV_IRQ_SINK_UPDATE(ppc_icu_sink_update)
{
}

static DEV_ICU_GET_SINK(ppc_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct ppc_dev_private_s  *pv = dev->drv_pv;

  if (id < ICU_PPC_MAX_VECTOR)
    return pv->sinks + id;
  return NULL;
}

#define ppc_icu_link device_icu_dummy_link

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

/************************************************************************
        Timer driver part
************************************************************************/

#ifdef CONFIG_CPU_PPC_TIMER_CYCLECOUNTER

static DEV_TIMER_GET_VALUE(ppc_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct ppc_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_CLOCK
  if (rev && rev != pv->timer_rev)
#else
  if (rev && rev != 1)
#endif
    return -EAGAIN;

#ifdef CONFIG_ARCH_SMP
  if(pv->node.cpu_id != cpu_id())
    return -EIO;
#endif

  uint32_t      result;
  asm volatile ("mftbl %0" : "=r" (result));
  *value = result;

  return 0;
}

static DEV_TIMER_CONFIG(ppc_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct ppc_dev_private_s *pv = dev->drv_pv;
  error_t err = 0;

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

  return err;
}

#endif

/************************************************************************/

static DEV_CLEANUP(ppc_cleanup);
static DEV_INIT(ppc_init);

#ifdef CONFIG_DEVICE_CLOCK
static DEV_CLOCK_SINK_CHANGED(ppc_clk_changed)
{
  struct device_s *dev = ep->base.dev;
  struct ppc_dev_private_s *pv = dev->drv_pv;
  LOCK_SPIN_IRQ(&dev->lock);
  pv->freq = *freq;
  pv->timer_rev += 2;
  LOCK_RELEASE_IRQ(&dev->lock);
}
#endif

#define ppc_use dev_use_generic
#define ppc_timer_request (dev_timer_request_t*)&dev_driver_notsup_fcn
#define ppc_timer_cancel  (dev_timer_cancel_t*)&dev_driver_notsup_fcn

DRIVER_DECLARE(ppc_drv, DRIVER_FLAGS_EARLY_INIT, "PowerPC processor", ppc,
#ifdef CONFIG_DEVICE_IRQ
               DRIVER_ICU_METHODS(ppc_icu),
#endif
#ifdef CONFIG_CPU_PPC_TIMER_CYCLECOUNTER
               DRIVER_TIMER_METHODS(ppc_timer),
#endif
               DRIVER_CPU_METHODS(ppc_cpu));

DRIVER_REGISTER(ppc_drv
#ifdef CONFIG_LIBFDT
                ,DEV_ENUM_FDTNAME_ENTRY("cpu:ppc")
                ,DEV_ENUM_FDTNAME_ENTRY("cpu:powerpc")
#endif
                );

static DEV_INIT(ppc_init)
{
  struct ppc_dev_private_s  *pv;


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
  pv->timer_rev = 1;

  if (dev_clock_sink_hold(&pv->clk_ep, NULL))
    goto err_clku;
#else
  if (device_get_res_freq(dev, &pv->freq, 0))
    pv->freq = DEV_FREQ_INVALID;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* init ppc irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_PPC_MAX_VECTOR,
                       &ppc_icu_sink_update, DEV_IRQ_SENSE_HIGH_LEVEL);

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
#endif

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);

  return 0;
}

