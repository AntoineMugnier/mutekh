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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

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

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

CPU_LOCAL void *__context_data_base;

struct mips_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
#define ICU_MIPS_MAX_VECTOR	6
  struct dev_irq_ep_s	sinks[ICU_MIPS_MAX_VECTOR];
#endif

  struct cpu_tree_s node;
};

/************************************************************************
        Interrupts controller driver part
************************************************************************/

#ifdef CONFIG_DEVICE_IRQ

static CPU_LOCAL struct device_s *mips_icu_dev;

static CPU_INTERRUPT_HANDLER(mips_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(mips_icu_dev);
  struct mips_dev_private_s  *pv = dev->drv_pv;

  if ( irq < ICU_MIPS_MAX_VECTOR ) {
    struct dev_irq_ep_s *sink = pv->sinks + irq;
    int_fast16_t id = 0;

    sink->process(sink, &id);
  }
}

static DEVICU_GET_ENDPOINT(mips_icu_get_endpoint)
{
  struct device_s *dev = idev->dev;
  struct mips_dev_private_s *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK:
      if (id < ICU_MIPS_MAX_VECTOR)
        return pv->sinks + id;
    default:
      return NULL;
    }
}

static DEVICU_ENABLE_IRQ(mips_icu_enable_irq)
{
  struct device_s *dev = idev->dev;

  // inputs are single wire, logical irq id must be 0
  if (irq_id > 0)
    return 0;

# ifdef CONFIG_ARCH_SMP
  if (!arch_cpu_irq_affinity_test(dev, dev_ep))
    return 0;

# else
  struct mips_dev_private_s *pv = dev->drv_pv;
  /* Enable irq line. On SMP platforms, all lines are already enabled on init. */
  uint_fast8_t icu_in_id = sink - pv->sinks;
  reg_t status = cpu_mips_mfc0(CPU_MIPS_STATUS, 0);
  status |= 1 << (icu_in_id + 10);
  cpu_mips_mtc0(CPU_MIPS_STATUS, 0, status);
# endif

  return 1;
}

# ifndef CONFIG_ARCH_SMP
/* Disable irq line. On SMP platforms, all lines must remain enabled. */
static DEVICU_DISABLE_IRQ(mips_icu_disable_irq)
{
  struct device_s *dev = idev->dev;
  struct mips_dev_private_s  *pv = dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sinks;

  reg_t status = cpu_mips_mfc0(CPU_MIPS_STATUS, 0);
  status &= ~(1 << (icu_in_id + 10));
  cpu_mips_mtc0(CPU_MIPS_STATUS, 0, status);
}
# endif

static const struct driver_icu_s  mips_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_endpoint  = mips_icu_get_endpoint,
  .f_enable_irq   = mips_icu_enable_irq,
# ifndef CONFIG_ARCH_SMP
  .f_disable_irq   = mips_icu_disable_irq,
# endif
};

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

#if defined(CONFIG_ARCH_SMP) && defined(CONFIG_HEXO_USERMODE)
void * cpu_local_storage[CONFIG_ARCH_LAST_CPU_ID + 1]; /* used to restore cls reg when back from user mode */
#endif

static DEVCPU_REG_INIT(mips_cpu_reg_init)
{
  struct device_s *dev = cdev->dev;
  __unused__ struct mips_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  assert(pv->node.cpu_id == cpu_id());

  /* set cpu local storage register base pointer */
  asm volatile("move $27, %0" : : "r" (pv->node.cls));

  /* Set exception vector */
  extern __ldscript_symbol_t CPU_NAME_DECL(exception_vector);
  cpu_mips_mtc0(15, 1, (reg_t)&CPU_NAME_DECL(exception_vector));

# ifdef CONFIG_DEVICE_IRQ
  /* enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
  reg_t status = cpu_mips_mfc0(CPU_MIPS_STATUS, 0);
  status |= 0xfc00;
  cpu_mips_mtc0(CPU_MIPS_STATUS, 0, status);
# endif

# ifdef CONFIG_HEXO_USERMODE
  cpu_local_storage[pv->id] = pv->node.cls;
# endif
#endif

#if defined(CONFIG_SOCLIB_MEMCHECK) && defined(CONFIG_HEXO_USERMODE)
  void cpu_context_set_user();
  void cpu_context_set_user_end();
  soclib_mem_bypass_sp_check(&cpu_context_set_user, &cpu_context_set_user_end);
#endif

  CPU_LOCAL_SET(cpu_device, dev);
}


#ifdef CONFIG_ARCH_SMP
static DEVCPU_GET_NODE(mips_cpu_get_node)
{
  struct device_s *dev = cdev->dev;
  struct mips_dev_private_s *pv = dev->drv_pv;
  return &pv->node;
}
#endif


static const struct driver_cpu_s  mips_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = mips_cpu_reg_init,
#ifdef CONFIG_ARCH_SMP
  .f_get_node   = mips_cpu_get_node,
#endif
};

/************************************************************************
        Timer driver part
************************************************************************/

# if CONFIG_CPU_MIPS_VERSION >= 32

#define MIPS_HAS_TIMER

static DEVTIMER_START_STOP(mips_timer_start_stop)
{
  return 0;
}

static DEVTIMER_GET_VALUE(mips_timer_get_value)
{
  struct device_s *dev = tdev->dev;
  __unused__ struct mips_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  if (pv->node.cpu_id != cpu_id())
    return -EIO;
#endif

  *value = cpu_mips_mfc0(9, 0);

  return 0;
}

static DEVTIMER_RESOLUTION(mips_timer_resolution)
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

static const struct driver_timer_s  mips_timer_drv =
{
  .class_          = DRIVER_CLASS_TIMER,
  .f_start_stop    = mips_timer_start_stop,
  .f_get_value     = mips_timer_get_value,
  .f_resolution    = mips_timer_resolution,
  .f_request       = (devtimer_request_t*)&dev_driver_notsup_fcn,
  .f_cancel        = (devtimer_request_t*)&dev_driver_notsup_fcn,
};

#endif

/************************************************************************/

static DEV_CLEANUP(mips_cleanup);
static DEV_INIT(mips_init);

static const struct devenum_ident_s  mips_ids[] =
{
#ifdef CONFIG_LIBFDT
  DEVENUM_FDTNAME_ENTRY("cpu:mips"),
# ifdef CONFIG_CPU_ENDIAN_LITTLE
  DEVENUM_FDTNAME_ENTRY("cpu:mipsel"),
  DEVENUM_FDTNAME_ENTRY("cpu:mips32el"),
# endif
# ifdef CONFIG_CPU_ENDIAN_BIG
  DEVENUM_FDTNAME_ENTRY("cpu:mipseb"),
  DEVENUM_FDTNAME_ENTRY("cpu:mips32eb"),
# endif
#endif
  { 0 }
};

const struct driver_s  mips_drv =
{
  .desc           = "Mips processor",
  .id_table       = mips_ids,

  .f_init         = mips_init,
  .f_cleanup      = mips_cleanup,

  .classes        = {
    &mips_cpu_drv,
#ifdef CONFIG_DEVICE_IRQ
    &mips_icu_drv,
#endif
#ifdef MIPS_HAS_TIMER
    &mips_timer_drv,
#endif
    0
  }
};

REGISTER_DRIVER(mips_drv);

static DEV_INIT(mips_init)
{
  struct mips_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  /* get processor device id specifed in resources */
  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
    PRINTK_RET(-ENOENT, "mips: device has no ID resource")
      ;

  /* allocate device private data */
  pv = mem_alloc_cpu(sizeof (*pv), (mem_scope_sys), id);
  if (pv == NULL)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if (cpu_tree_node_init(&pv->node, id, dev))
    goto err_pv;

#ifdef CONFIG_DEVICE_IRQ
  /* init mips irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_MIPS_MAX_VECTOR,
                       DEV_IRQ_SENSE_HIGH_LEVEL);

# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->node.cls, mips_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->node.cls, mips_irq_handler);
# else
  if (id == CONFIG_ARCH_BOOTSTRAP_CPU_ID)
    {
      CPU_LOCAL_SET(mips_icu_dev, dev);
      cpu_interrupt_sethandler(mips_irq_handler);
    }
# endif
#endif

  if (cpu_tree_insert(&pv->node))
    goto err_node;

  dev->drv = &mips_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_node:
  cpu_tree_node_cleanup(&pv->node);
 err_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(mips_cleanup)
{
  struct mips_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
  reg_t status = cpu_mips_mfc0(CPU_MIPS_STATUS, 0);
  status &= ~0xfc00;
  cpu_mips_mtc0(CPU_MIPS_STATUS, 0, status);
# endif
  /* detach mips irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, ICU_MIPS_MAX_VECTOR);
#endif

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);
}

