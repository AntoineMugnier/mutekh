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
#include <hexo/segment.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/class/cpu.h>
#include <device/class/timer.h>
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

#ifdef CONFIG_ARCH_SMP
  uint_fast8_t id;
  void *cls;            //< cpu local storage
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

static DEVICU_GET_ENDPOINT(arm_icu_get_endpoint)
{
  struct device_s *dev = idev->dev;
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

static DEVICU_ENABLE_IRQ(arm_icu_enable_irq)
{
  __unused__ struct device_s *dev = idev->dev;

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

static DEVCPU_REG_INIT(arm_cpu_reg_init)
{
  struct device_s *dev = cdev->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  assert(pv->id == cpu_id());

# if !defined(CONFIG_CPU_ARM_TLS_IN_C15)
#  error SMP and TLS unsupported
# endif
  asm volatile ("mcr p15,0,%0,c13,c0,3":: "r" (pv->cls));

# ifdef CONFIG_DEVICE_IRQ
  /* Enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
# endif
#endif

  CPU_LOCAL_SET(cpu_device, dev);

#ifdef CONFIG_SOCLIB_MEMCHECK
  /* all these function may execute with invalid stack pointer
     register due to arm shadow registers bank switching. */
  void cpu_boot();
  void cpu_boot_end();
  soclib_mem_bypass_sp_check(&cpu_boot, &cpu_boot_end);
  void arm_exc_undef();
  void arm_exc_undef_end();
  soclib_mem_bypass_sp_check(&arm_exc_undef, &arm_exc_undef_end);
  void arm_exc_pabt();
  void arm_exc_pabt_end();
  soclib_mem_bypass_sp_check(&arm_exc_pabt, &arm_exc_pabt_end);
  void arm_exc_dabt();
  void arm_exc_dabt_end();
  soclib_mem_bypass_sp_check(&arm_exc_dabt, &arm_exc_dabt_end);
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
  void arm_context_jumpto_internal_end();
  soclib_mem_bypass_sp_check(&cpu_context_jumpto, &arm_context_jumpto_internal_end);
#endif
}

#ifdef CONFIG_ARCH_SMP
static DEVCPU_GET_STORAGE(arm_cpu_get_storage)
{
  struct device_s *dev = cdev->dev;
  struct arm_dev_private_s *pv = dev->drv_pv;
  return pv->cls;
}
#endif

const struct driver_cpu_s  arm_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = arm_cpu_reg_init,
#ifdef CONFIG_ARCH_SMP
  .f_get_storage   = arm_cpu_get_storage,
#endif
};

/************************************************************************
        Timer driver part
************************************************************************/

#ifdef CONFIG_CPU_ARM_CYCLES_COUNTER

static DEVTIMER_REQUEST(arm_timer_request)
{
  return -ENOTSUP;
}

static DEVTIMER_START_STOP(arm_timer_start_stop)
{
  return -ENOTSUP;
}

static DEVTIMER_GET_VALUE(arm_timer_get_value)
{
  struct device_s *dev = tdev->dev;
  __unused__ struct arm_dev_private_s *pv = dev->drv_pv;

# ifdef CONFIG_ARCH_SMP
  if(pv->id != cpu_id())
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

static DEVTIMER_RESOLUTION(arm_timer_resolution)
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

static const struct driver_timer_s  arm_timer_drv =
{
  .class_          = DRIVER_CLASS_TIMER,
  .f_request       = arm_timer_request,
  .f_start_stop    = arm_timer_start_stop,
  .f_get_value     = arm_timer_get_value,
  .f_resolution    = arm_timer_resolution,
};

#endif

/************************************************************************/

static DEV_CLEANUP(arm_cleanup);
static DEV_INIT(arm_init);

static const struct devenum_ident_s  arm_ids[] =
{
#ifdef CONFIG_LIBFDT
  DEVENUM_FDTNAME_ENTRY("cpu:arm"),
#endif
  { 0 }
};

const struct driver_s  arm_drv =
{
  .desc           = "Arm processor",
  .id_table       = arm_ids,

  .f_init         = arm_init,
  .f_cleanup      = arm_cleanup,

  .classes        = {
    &arm_cpu_drv,
#ifdef CONFIG_DEVICE_IRQ
    &arm_icu_drv,
#endif
#ifdef CONFIG_CPU_ARM_CYCLES_COUNTER
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
#ifdef CONFIG_ARCH_SMP
    PRINTK_RET(-ENOENT, "arm: device has no ID resource")
#endif
      ;

  /* allocate device private data */
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
  /* init arm irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_ARM_MAX_VECTOR);

  /* set processor interrupt handler */
# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->cls, arm_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->cls, arm_irq_handler);
# else
  if (id == 0)
    {
      CPU_LOCAL_SET(arm_icu_dev, dev);
      cpu_interrupt_sethandler(arm_irq_handler);
    }
# endif
#endif

  dev->drv = &arm_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
#ifdef CONFIG_ARCH_SMP
 err_mem:
  if (sizeof(*pv))
    mem_free(pv);
  return -1;
#endif
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

  if (sizeof(*pv))
    mem_free(pv);
}

