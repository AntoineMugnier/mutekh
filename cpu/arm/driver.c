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

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

struct arm_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
#define ICU_ARM_MAX_VECTOR	1
  struct dev_irq_ep_s	sinks[ICU_ARM_MAX_VECTOR];
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

static CPU_LOCAL struct device_s *arm_icu_dev;

static CPU_INTERRUPT_HANDLER(arm_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(arm_icu_dev);
  struct arm_dev_private_s  *pv = dev->drv_pv;

  if ( irq < ICU_ARM_MAX_VECTOR ) {
    struct dev_irq_ep_s *sink = pv->sinks + irq;
    int_fast16_t id = 0;

    sink->process(sink, &id);
  }
}

static DEV_ICU_GET_ENDPOINT(arm_icu_get_endpoint)
{
  struct device_s *dev = accessor->dev;
  struct arm_dev_private_s  *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK:
      if (id < ICU_ARM_MAX_VECTOR)
        return pv->sinks + id;
    default:
      return NULL;
    }
}

static DEV_ICU_ENABLE_IRQ(arm_icu_enable_irq)
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

const struct driver_icu_s  arm_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_endpoint  = arm_icu_get_endpoint,
  .f_enable_irq    = arm_icu_enable_irq,
};

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

# if CONFIG_CPU_ARM_ARCH_VERSION >= 6
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
  void arm_exc_fiq();
  void arm_exc_fiq_end();
  soclib_mem_bypass_sp_check(&arm_exc_fiq, &arm_exc_fiq_end);
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

const struct driver_cpu_s  arm_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = arm_cpu_reg_init,
#ifdef CONFIG_ARCH_SMP
  .f_get_node   = arm_cpu_get_node,
#endif
};

/************************************************************************
        Timer driver part
************************************************************************/

#ifdef CONFIG_CPU_ARM_TIMER_CYCLECOUNTER

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

  switch (accessor->number)
    {
    case 0: {          /* cycle counter */
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

    default:
      return -ENOTSUP;
    }

  return 0;
}

static DEV_TIMER_CONFIG(arm_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct arm_dev_private_s *pv = dev->drv_pv;
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
          cfg->cap = DEV_TIMER_CAP_KEEPVALUE | DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_TICKLESS;
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

static const struct driver_timer_s  arm_timer_drv =
{
  .class_          = DRIVER_CLASS_TIMER,
  .f_get_value     = arm_timer_get_value,
  .f_config        = arm_timer_config,
  .f_request       = (dev_timer_request_t*)&dev_driver_notsup_fcn,
  .f_cancel        = (dev_timer_request_t*)&dev_driver_notsup_fcn,
};

#endif

/************************************************************************/

static DEV_CLEANUP(arm_cleanup);
static DEV_INIT(arm_init);

#ifdef CONFIG_DEVICE_CLOCK
static DEV_CLOCK_SINK_CHANGED(arm_clk_changed)
{
  struct device_s *dev = ep->dev;
  struct arm_dev_private_s *pv = dev->drv_pv;
  LOCK_SPIN_IRQ(&dev->lock);
  pv->freq = *freq;
  pv->timer_rev += 2;
  LOCK_RELEASE_IRQ(&dev->lock);
}
#endif

static DEV_USE(arm_use)
{
  if (accessor->number > 0)
    return -ENOTSUP;

  switch (accessor->api->class_)
    {
#ifdef CONFIG_CPU_ARM_TIMER_CYCLECOUNTER
    case DRIVER_CLASS_TIMER:
      return 0;
#endif

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

static const struct dev_enum_ident_s  arm_ids[] =
{
#ifdef CONFIG_LIBFDT
  DEV_ENUM_FDTNAME_ENTRY("cpu:arm"),
#endif
  { 0 }
};

const struct driver_s  arm_drv =
{
  .desc           = "Arm processor",
  .id_table       = arm_ids,

  .f_init         = arm_init,
  .f_cleanup      = arm_cleanup,
  .f_use          = arm_use,

  .classes        = {
    &arm_cpu_drv,
#ifdef CONFIG_DEVICE_IRQ
    &arm_icu_drv,
#endif
#ifdef CONFIG_CPU_ARM_TIMER_CYCLECOUNTER
    &arm_timer_drv,
#endif
    0
  }
};

REGISTER_DRIVER(arm_drv);

static DEV_INIT(arm_init)
{
  struct arm_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

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

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_init(dev, &pv->clk_ep, &arm_clk_changed);

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
  /* init arm irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_ARM_MAX_VECTOR,
# if defined(CONFIG_CPU_ARM_SOCLIB)
                       DEV_IRQ_SENSE_HIGH_LEVEL
# else
                       DEV_IRQ_SENSE_LOW_LEVEL
# endif
                       );

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

  dev->drv = &arm_drv;
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

static DEV_CLEANUP(arm_cleanup)
{
  struct arm_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
# endif
  /* detach arm irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, ICU_ARM_MAX_VECTOR);
#endif

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);
}

