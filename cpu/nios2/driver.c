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

/** pointer to context local storage in cpu local storage */
CPU_LOCAL void *__context_data_base;

struct nios2_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
#define ICU_NIOS2_MAX_VECTOR	32
  struct dev_irq_ep_s	sinks[ICU_NIOS2_MAX_VECTOR];
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

static CPU_LOCAL struct device_s *nios2_icu_dev;

static CPU_INTERRUPT_HANDLER(nios2_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(nios2_icu_dev);
  struct nios2_dev_private_s  *pv = dev->drv_pv;

  if ( irq < ICU_NIOS2_MAX_VECTOR ) {
    struct dev_irq_ep_s *sink = pv->sinks + irq;
    int_fast16_t id = 0;

    sink->process(sink, &id);
  }
}

static DEVICU_GET_ENDPOINT(nios2_icu_get_endpoint)
{
  struct device_s *dev = idev->dev;
  struct nios2_dev_private_s  *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK:
      if (id < ICU_NIOS2_MAX_VECTOR)
        return pv->sinks + id;
    default:
      return NULL;
    }
}

static DEVICU_ENABLE_IRQ(nios2_icu_enable_irq)
{
  struct device_s *dev = CPU_LOCAL_GET(nios2_icu_dev);
  struct nios2_dev_private_s  *pv = dev->drv_pv;

  // inputs are single wire, logical irq id must be 0
  if (irq_id > 0)
    return 0;

# ifdef CONFIG_ARCH_SMP
  if (!arch_cpu_irq_affinity_test(dev, dev_ep))
    return 0;

# else
  /* Enable irq line. On SMP platforms, all lines are already enabled on init. */
  uint_fast8_t icu_in_id = sink - pv->sinks;
  reg_t status = cpu_nios2_read_ctrl_reg(3);
  status |= 1 << icu_in_id;
  cpu_nios2_write_ctrl_reg(3, status);
# endif

  return 1;
}

# ifndef CONFIG_ARCH_SMP
static DEVICU_DISABLE_IRQ(nios2_icu_disable_irq)
{
  struct device_s *dev = CPU_LOCAL_GET(nios2_icu_dev);
  struct nios2_dev_private_s  *pv = dev->drv_pv;
  uint_fast8_t icu_in_id = sink - pv->sinks;

  /* Disable irq line. On SMP platforms, all lines must remain enabled. */
  reg_t status = cpu_nios2_read_ctrl_reg(3);
  status &= ~(1 << icu_in_id);
  cpu_nios2_write_ctrl_reg(3, status);
}
# endif

const struct driver_icu_s  nios2_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_endpoint  = nios2_icu_get_endpoint,
  .f_enable_irq    = nios2_icu_enable_irq,
# ifndef CONFIG_ARCH_SMP
  .f_disable_irq   = nios2_icu_disable_irq,
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

static DEVCPU_REG_INIT(nios2_cpu_reg_init)
{
  struct device_s *dev = cdev->dev;
  __unused__ struct nios2_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  assert(pv->id == cpu_id());

  /* set cpu local storage register base pointer */
  __asm__ volatile("mov " ASM_STR(CPU_NIOS2_CLS_REG) ", %0" : : "r" (pv->cls));

# ifdef CONFIG_DEVICE_IRQ
  /* Enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
  cpu_nios2_write_ctrl_reg(3, 0xffffffff);
# endif

# ifdef CONFIG_HEXO_USERMODE
  cpu_local_storage[pv->id] = pv->cls;
# endif
#endif

  CPU_LOCAL_SET(cpu_device, dev);
}

#ifdef CONFIG_ARCH_SMP
static DEVCPU_GET_STORAGE(nios2_cpu_get_storage)
{
  struct device_s *dev = cdev->dev;
  struct nios2_dev_private_s *pv = dev->drv_pv;
  return pv->cls;
}
#endif

const struct driver_cpu_s  nios2_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = nios2_cpu_reg_init,
#ifdef CONFIG_ARCH_SMP
  .f_get_storage   = nios2_cpu_get_storage,
#endif
};

/************************************************************************
        Timer driver part
************************************************************************/

static DEVTIMER_REQUEST(nios2_timer_request)
{
  return -ENOTSUP;
}

static DEVTIMER_START_STOP(nios2_timer_start_stop)
{
  return -ENOTSUP;
}

static DEVTIMER_GET_VALUE(nios2_timer_get_value)
{
  struct device_s *dev = tdev->dev;
  __unused__ struct nios2_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  if (pv->id != cpu_id())
    return -EIO;
#endif

  *value = cpu_nios2_read_ctrl_reg(31);

  return 0;
}

static DEVTIMER_RESOLUTION(nios2_timer_resolution)
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

static const struct driver_timer_s  nios2_timer_drv =
{
  .class_          = DRIVER_CLASS_TIMER,
  .f_request       = nios2_timer_request,
  .f_start_stop    = nios2_timer_start_stop,
  .f_get_value     = nios2_timer_get_value,
  .f_resolution    = nios2_timer_resolution,
};

/************************************************************************/

static DEV_CLEANUP(nios2_cleanup);
static DEV_INIT(nios2_init);

static const struct devenum_ident_s  nios2_ids[] =
{
#ifdef CONFIG_LIBFDT
  DEVENUM_FDTNAME_ENTRY("cpu:nios2"),
#endif
  { 0 }
};

const struct driver_s  nios2_drv =
{
  .desc           = "Nios II processor",
  .id_table       = nios2_ids,

  .f_init         = nios2_init,
  .f_cleanup      = nios2_cleanup,

  .classes        = {
    &nios2_cpu_drv,
#ifdef CONFIG_DEVICE_IRQ
    &nios2_icu_drv,
#endif
    &nios2_timer_drv,
    0
  }
};

REGISTER_DRIVER(nios2_drv);

static DEV_INIT(nios2_init)
{
  struct nios2_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  /* get processor device id specifed in resources */
  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
#ifdef CONFIG_ARCH_SMP
    PRINTK_RET(-ENOENT, "nios2: device has no ID resource")
#endif
      ;

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
  /* init nios2 irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_NIOS2_MAX_VECTOR);

# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->cls, nios2_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->cls, nios2_irq_handler);
# else
  if (id == 0)
    {
      CPU_LOCAL_SET(nios2_icu_dev, dev);
      cpu_interrupt_sethandler(nios2_irq_handler);
    }
# endif
#endif

  dev->drv = &nios2_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
#ifdef CONFIG_ARCH_SMP
 err_mem:
  if (sizeof(*pv))
    mem_free(pv);
  return -1;
#endif
}

static DEV_CLEANUP(nios2_cleanup)
{
  struct nios2_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
  cpu_nios2_write_ctrl_reg(3, 0);
# endif
  /* detach nios2 irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, ICU_NIOS2_MAX_VECTOR);
#endif

  if (sizeof(*pv))
    mem_free(pv);
}

