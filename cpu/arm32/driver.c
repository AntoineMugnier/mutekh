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
#include <device/clock.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/soclib/mem_checker.h>
#endif

#if defined(CONFIG_CPU_ARM32_SOCLIB)
# define ARM_IRQ_SENSE_MODE    DEV_IRQ_SENSE_HIGH_LEVEL
#else
# define ARM_IRQ_SENSE_MODE    DEV_IRQ_SENSE_LOW_LEVEL
#endif


struct arm_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
#define ICU_ARM_MAX_VECTOR	1
  struct dev_irq_sink_s	sinks[ICU_ARM_MAX_VECTOR];
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

static CPU_LOCAL struct device_s *arm_icu_dev;

static CPU_INTERRUPT_HANDLER(arm_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(arm_icu_dev);
  struct arm_dev_private_s  *pv = dev->drv_pv;

  if ( irq < ICU_ARM_MAX_VECTOR ) {
    struct dev_irq_sink_s *sink = pv->sinks + irq;
    device_irq_sink_process(sink, 0);
  }
}

static DEV_IRQ_SINK_UPDATE(arm_icu_sink_update)
{
}

static DEV_ICU_GET_SINK(arm_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct arm_dev_private_s  *pv = dev->drv_pv;

  if (id < ICU_ARM_MAX_VECTOR)
    return &pv->sinks[id];
  return NULL;
}

#define arm_icu_link device_icu_dummy_link

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

static DEV_CPU_REG_INIT(arm_cpu_reg_init)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  assert(pv->node.cpu_id == cpu_id());

# if CONFIG_CPU_ARM32_ARCH_VERSION >= 6
   asm volatile ("mcr p15,0,%0,c13,c0,3":: "r" (pv->node.cls));
# else
#  error SMP and TLS unsupported
# endif

# ifdef CONFIG_DEVICE_IRQ
  /* Enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
# endif
#endif

  CPU_LOCAL_SET(cpu_device, dev);

#ifdef CONFIG_SOCLIB_MEMCHECK
  /* all these function may execute with invalid stack pointer
     register due to arm shadow registers bank switching. */
  void CPU_NAME_DECL(exception_vector)();
  void CPU_NAME_DECL(exception_vector_end)();
  soclib_mem_bypass_sp_check(&CPU_NAME_DECL(exception_vector), &CPU_NAME_DECL(exception_vector_end));
  void arm_exc_undef();
  void arm_exc_undef_end();
  soclib_mem_bypass_sp_check(&arm_exc_undef, &arm_exc_undef_end);
  void arm_exc_pabt();
  void arm_exc_pabt_end();
  soclib_mem_bypass_sp_check(&arm_exc_pabt, &arm_exc_pabt_end);
  void arm_exc_dabt();
  void arm_exc_dabt_end();
  soclib_mem_bypass_sp_check(&arm_exc_dabt, &arm_exc_dabt_end);
  void arm_exc_common_asm();
  void arm_exc_common_asm_end();
  soclib_mem_bypass_sp_check(&arm_exc_common_asm, &arm_exc_common_asm_end);
# ifdef CONFIG_HEXO_IRQ
  void arm_exc_irq();
  void arm_exc_irq_end();
  soclib_mem_bypass_sp_check(&arm_exc_irq, &arm_exc_irq_end);
# endif

# ifdef CONFIG_HEXO_USERMODE
  void arm_exc_swi();
  void arm_exc_swi_end();
  soclib_mem_bypass_sp_check(&arm_exc_swi, &arm_exc_swi_end);

  void cpu_context_set_user();
  void cpu_context_set_user_end();
  soclib_mem_bypass_sp_check(&cpu_context_set_user, &cpu_context_set_user_end);
# endif

  void cpu_context_jumpto();
  void cpu_context_jumpto_end();
  soclib_mem_bypass_sp_check(&cpu_context_jumpto, &cpu_context_jumpto_end);
#endif
}

#ifdef CONFIG_ARCH_SMP
static DEV_CPU_GET_NODE(arm_cpu_get_node)
{
  struct device_s *dev = accessor->dev;
  struct arm_dev_private_s *pv = dev->drv_pv;
  return &pv->node;
}
#endif

/************************************************************************
        Timer driver part
************************************************************************/

#ifdef CONFIG_CPU_ARM32_TIMER_CYCLECOUNTER

static DEV_TIMER_GET_VALUE(arm_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_CLOCK
  if (rev && rev != pv->timer_rev)
#else
  if (rev && rev != 1)
#endif
    return -EAGAIN;

# ifdef CONFIG_ARCH_SMP
  if(pv->node.cpu_id != cpu_id())
    return -EIO;
# endif

      uint32_t ret;
      THUMB_TMP_VAR;
      asm volatile (
                    THUMB_TO_ARM
                    "mrc p15, 0, %[ret], c15, c12, 1\n\t"
                    ARM_TO_THUMB
                    : [ret] "=r"(ret) /*,*/ THUMB_OUT(,));
      *value = ret;

  return 0;
}

static DEV_TIMER_CONFIG(arm_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct arm_dev_private_s *pv = dev->drv_pv;
  error_t err = 0;

      if (res > 1)
        err = -ERANGE;
      if (cfg)
        {
          cfg->freq = pv->freq;
          cfg->max = 0xffffffff;
          cfg->cap = DEV_TIMER_CAP_KEEPVALUE | DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_TICKLESS;
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

#define arm_timer_request (dev_timer_request_t*)dev_driver_notsup_fcn
#define arm_timer_cancel (dev_timer_cancel_t*)dev_driver_notsup_fcn

#endif

/************************************************************************/


static DEV_USE(arm_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct arm_dev_private_s *pv = dev->drv_pv;
      pv->freq = chg->freq;
      pv->timer_rev += 2;
      return 0;
    }
#endif
    default:
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(arm_init)
{
  struct arm_dev_private_s  *pv;


  /* get processor device id specifed in resources */
  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
    PRINTK_RET(-ENOENT, "arm: device has no ID resource")
      ;

  /* allocate device private data */
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
  /* init arm irq sink endpoints */
  device_irq_sink_init(dev, pv->sinks, ICU_ARM_MAX_VECTOR,
                       &arm_icu_sink_update, ARM_IRQ_SENSE_MODE);

  /* set processor interrupt handler */
# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->node.cls, arm_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->node.cls, arm_irq_handler);
# else
  if (id == CONFIG_ARCH_BOOTSTRAP_CPU_ID)
    {
      CPU_LOCAL_SET(arm_icu_dev, dev);
      cpu_interrupt_sethandler(arm_irq_handler);
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

static DEV_CLEANUP(arm_cleanup)
{
  struct arm_dev_private_s *pv = dev->drv_pv;

  dev_drv_clock_cleanup(dev, &pv->clk_ep);

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(arm32_drv, DRIVER_FLAGS_EARLY_INIT, "Arm processor", arm,
#ifdef CONFIG_DEVICE_IRQ
               DRIVER_ICU_METHODS(arm_icu),
#endif
#ifdef CONFIG_CPU_ARM32_TIMER_CYCLECOUNTER
               DRIVER_TIMER_METHODS(arm_timer),
#endif
               DRIVER_CPU_METHODS(arm_cpu));

DRIVER_REGISTER(arm32_drv,
                DEV_ENUM_FDTNAME_ENTRY("cpu:arm"));

