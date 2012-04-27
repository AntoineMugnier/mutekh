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

#ifdef CONFIG_HEXO_IPI
static DEVICU_SETUP_IPI_EP(arm_icu_setup_ipi_ep)
{
  abort(); // FIXME
  return -1;
}
#endif

static DEVICU_DISABLE_SINK(arm_icu_disable_sink)
{
# ifndef CONFIG_ARCH_SMP
  /* Disable irq line. On SMP platforms, all lines must remain enabled. */
# endif
}

static DEVICU_GET_SINK(arm_icu_get_sink)
{
  struct device_s *dev = idev->dev;
  struct arm_dev_private_s  *pv = dev->drv_pv;

  if (icu_in_id >= ICU_ARM_MAX_VECTOR)
    return NULL;

# ifndef CONFIG_ARCH_SMP
  /* Enable irq line. On SMP platforms, all lines are already enabled on init. */
# endif

  return pv->sinks + icu_in_id;
}

const struct driver_icu_s  arm_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_sink     = arm_icu_get_sink,
  .f_disable_sink = arm_icu_disable_sink,
#ifdef CONFIG_HEXO_IPI
  .f_setup_ipi_ep = arm_icu_setup_ipi_ep,
#endif
};

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

static DEVCPU_REG_INIT(arm_cpu_reg_init)
{
  struct device_s *dev = cdev->dev;
  struct arm_dev_private_s *pv = dev->drv_pv;

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

static DEVTIMER_GET_VALUE(arm_timer_get_value)
{
  struct device_s *dev = tdev->dev;
  struct arm_dev_private_s *pv = dev->drv_pv;

# ifdef CONFIG_ARCH_SMP
  assert(pv->id == cpu_id());
# endif

  uint32_t ret;
  THUMB_TMP_VAR;

  asm volatile (
                THUMB_TO_ARM
                "mrc p15, 0, %[ret], c15, c12, 1\n\t"
                ARM_TO_THUMB
                : [ret] "=r"(ret) /*,*/ THUMB_OUT(,));

  return ret;
}

static DEVTIMER_RESOLUTION(arm_timer_resolution)
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

static const struct driver_timer_s  arm_timer_drv =
{
  .class_          = DRIVER_CLASS_TIMER,
  .f_get_value     = arm_timer_get_value,
  .f_resolution    = arm_timer_resolution,
};

#endif

/************************************************************************/

static DEV_CLEANUP(arm_cleanup);
static DEV_INIT(arm_init);

static const struct devenum_ident_s  arm_ids[] =
{
#ifdef CONFIG_FDT
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
  device_irq_sink_init(dev, pv->sinks, ICU_ARM_MAX_VECTOR, NULL);

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
 err_mem:
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

  if (sizeof(*pv))
    mem_free(pv);
}

