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
#include <device/class/clock.h>
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
  struct dev_irq_sink_s	sinks[ICU_MIPS_MAX_VECTOR];
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

static CPU_LOCAL struct device_s *mips_icu_dev;

static CPU_INTERRUPT_HANDLER(mips_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(mips_icu_dev);
  struct mips_dev_private_s  *pv = dev->drv_pv;

  if ( irq < ICU_MIPS_MAX_VECTOR ) {
    struct dev_irq_sink_s *sink = pv->sinks + irq;
    device_irq_sink_process(sink, 0);
  }
}

static DEV_IRQ_SINK_UPDATE(mips_icu_sink_update)
{
#ifndef CONFIG_ARCH_SMP
  struct device_s *dev = sink->base.dev;
  struct mips_dev_private_s  *pv = dev->drv_pv;
  uint_fast8_t sink_id = sink - pv->sinks;
  reg_t status = cpu_mips_mfc0(CPU_MIPS_STATUS, 0);

  switch (sense)
    {
    case DEV_IRQ_SENSE_NONE:
      status &= ~(1 << (sink_id + 10));
      break;
    case DEV_IRQ_SENSE_HIGH_LEVEL:
      status |= 1 << (sink_id + 10);
      break;
    default:
      return;
    }

  cpu_mips_mtc0(CPU_MIPS_STATUS, 0, status);
#endif
}

static DEV_ICU_GET_SINK(mips_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct mips_dev_private_s *pv = dev->drv_pv;

  if (id < ICU_MIPS_MAX_VECTOR)
    return pv->sinks + id;
  return NULL;
}

#define mips_icu_link device_icu_dummy_link

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

#if defined(CONFIG_ARCH_SMP) && defined(CONFIG_HEXO_USERMODE)
void * cpu_local_storage[CONFIG_ARCH_LAST_CPU_ID + 1]; /* used to restore cls reg when back from user mode */
#endif

static DEV_CPU_REG_INIT(mips_cpu_reg_init)
{
  struct device_s *dev = accessor->dev;
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
  cpu_local_storage[pv->node.cpu_id] = pv->node.cls;
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
static DEV_CPU_GET_NODE(mips_cpu_get_node)
{
  struct device_s *dev = accessor->dev;
  struct mips_dev_private_s *pv = dev->drv_pv;
  return &pv->node;
}
#endif


/************************************************************************
        Timer driver part
************************************************************************/

# ifdef CONFIG_CPU_MIPS_TIMER_CYCLECOUNTER

static DEV_TIMER_GET_VALUE(mips_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct mips_dev_private_s *pv = dev->drv_pv;

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

  *value = cpu_mips_mfc0(9, 0);

  return 0;
}

static DEV_TIMER_CONFIG(mips_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct mips_dev_private_s *pv = dev->drv_pv;
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

static DEV_CLEANUP(mips_cleanup);
static DEV_INIT(mips_init);

#ifdef CONFIG_DEVICE_CLOCK
static DEV_CLOCK_SINK_CHANGED(mips_clk_changed)
{
  struct device_s *dev = ep->base.dev;
  struct mips_dev_private_s *pv = dev->drv_pv;
  LOCK_SPIN_IRQ(&dev->lock);
  pv->freq = *freq;
  pv->timer_rev += 2;
  LOCK_RELEASE_IRQ(&dev->lock);
}
#endif

static DEV_USE(mips_use)
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

#define mips_timer_request (dev_timer_request_t*)&dev_driver_notsup_fcn
#define mips_timer_cancel  (dev_timer_cancel_t*)&dev_driver_notsup_fcn

DRIVER_DECLARE(mips_drv, DRIVER_FLAGS_EARLY_INIT, "MIPS processor", mips,
#ifdef CONFIG_DEVICE_IRQ
               DRIVER_ICU_METHODS(mips_icu),
#endif
#ifdef CONFIG_CPU_MIPS_TIMER_CYCLECOUNTER
               DRIVER_TIMER_METHODS(mips_timer),
#endif
               DRIVER_CPU_METHODS(mips_cpu));

DRIVER_REGISTER(mips_drv
#ifdef CONFIG_LIBFDT
                ,DEV_ENUM_FDTNAME_ENTRY("cpu:mips")
# ifdef CONFIG_CPU_ENDIAN_LITTLE
                ,DEV_ENUM_FDTNAME_ENTRY("cpu:mipsel")
                ,DEV_ENUM_FDTNAME_ENTRY("cpu:mips32el")
# endif
# ifdef CONFIG_CPU_ENDIAN_BIG
                ,DEV_ENUM_FDTNAME_ENTRY("cpu:mipseb")
                ,DEV_ENUM_FDTNAME_ENTRY("cpu:mips32eb")
# endif
#endif
                );

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

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_init(dev, &pv->clk_ep, &mips_clk_changed);

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
  /* init mips irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_MIPS_MAX_VECTOR,
                       &mips_icu_sink_update,
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
    goto err_clk;

  dev->drv = &mips_drv;
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

#ifdef CONFIG_DEVICE_CLOCK
  dev_clock_sink_release(&pv->clk_ep);
  dev_clock_sink_unlink(dev, &pv->clk_ep, 1);
#endif

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);
}

