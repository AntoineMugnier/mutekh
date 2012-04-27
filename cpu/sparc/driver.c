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

    Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2011 Institut Telecom / Telecom ParisTech

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
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

#ifdef CONFIG_CPU_SPARC_SINGLE_IRQ_EP
#define ICU_SPARC_MAX_VECTOR	1
#else
#define ICU_SPARC_MAX_VECTOR	15
#endif

struct sparc_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_ep_s	sinks[ICU_SPARC_MAX_VECTOR];
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

static CPU_LOCAL struct device_s *sparc_icu_dev;

static CPU_INTERRUPT_HANDLER(sparc_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(sparc_icu_dev);
  struct sparc_dev_private_s  *pv = dev->drv_pv;

#ifdef CONFIG_CPU_SPARC_SINGLE_IRQ_EP
    struct dev_irq_ep_s *sink = pv->sinks;
    int_fast16_t id = irq;

    sink->process(sink, &id);
#else
  if ( irq < ICU_SPARC_MAX_VECTOR ) {
    struct dev_irq_ep_s *sink = pv->sinks + irq;
    int_fast16_t id = 0;

    sink->process(sink, &id);
  }
#endif
}

#ifdef CONFIG_HEXO_IPI
static DEVICU_SETUP_IPI_EP(sparc_icu_setup_ipi_ep)
{
  abort(); // FIXME
  return -1;
}
#endif

static DEVICU_DISABLE_SINK(sparc_icu_disable_sink)
{
# ifndef CONFIG_ARCH_SMP
  /* Disable irq line. On SMP platforms, all lines must remain enabled. */
# endif
}

static DEVICU_GET_SINK(sparc_icu_get_sink)
{
  struct device_s *dev = idev->dev;
  struct sparc_dev_private_s  *pv = dev->drv_pv;

  if (icu_in_id >= ICU_SPARC_MAX_VECTOR)
    return NULL;

# ifndef CONFIG_ARCH_SMP
  /* Enable irq line. On SMP platforms, all lines are already enabled on init. */
# endif

  return pv->sinks + icu_in_id;
}

const struct driver_icu_s  sparc_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_sink     = sparc_icu_get_sink,
  .f_disable_sink = sparc_icu_disable_sink,
#ifdef CONFIG_HEXO_IPI
  .f_setup_ipi_ep = sparc_icu_setup_ipi_ep,
#endif
};

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

static DEVCPU_REG_INIT(sparc_cpu_reg_init)
{
  struct device_s *dev = cdev->dev;

#ifdef CONFIG_ARCH_SMP
  struct sparc_dev_private_s *pv = dev->drv_pv;

  assert(pv->id == cpu_id());

  /* set cpu local storage register base pointer */
  asm volatile("mov %0, %%g6" : : "r" (pv->cls));

# ifdef CONFIG_DEVICE_IRQ
  /* Enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
# endif
#endif

  CPU_LOCAL_SET(cpu_device, dev);

#ifdef CONFIG_SOCLIB_MEMCHECK
  /* all these functions may execute with briefly invalid stack & frame
     pointer registers due to register window switch. */

  void cpu_context_jumpto();
  void cpu_context_jumpto_end();
  soclib_mem_bypass_sp_check(&cpu_context_jumpto, &cpu_context_jumpto_end);

  extern __ldscript_symbol_t __exception_base_ptr;
  extern __ldscript_symbol_t __exception_base_ptr_end;
  soclib_mem_bypass_sp_check(&__exception_base_ptr, &__exception_base_ptr_end);

  void sparc_excep_entry();
  void sparc_excep_entry_end();
  soclib_mem_bypass_sp_check(&sparc_excep_entry, &sparc_excep_entry_end);

  void sparc_except_restore();
  void sparc_except_restore_end();
  soclib_mem_bypass_sp_check(&sparc_except_restore, &sparc_except_restore_end);

# ifdef CONFIG_HEXO_IRQ
  void sparc_irq_entry();
  void sparc_irq_entry_end();
  soclib_mem_bypass_sp_check(&sparc_irq_entry, &sparc_irq_entry_end);
# endif

# ifdef CONFIG_HEXO_USERMODE
  void sparc_syscall_entry();
  void sparc_syscall_entry_end();
  soclib_mem_bypass_sp_check(&sparc_syscall_entry, &sparc_syscall_entry_end);

  void cpu_context_set_user();
  void cpu_context_set_user_end();
  soclib_mem_bypass_sp_check(&cpu_context_set_user, &cpu_context_set_user_end);
# endif
#endif
}

#ifdef CONFIG_ARCH_SMP
static DEVCPU_GET_STORAGE(sparc_cpu_get_storage)
{
  struct device_s *dev = cdev->dev;
  struct sparc_dev_private_s *pv = dev->drv_pv;
  return pv->cls;
}
#endif

const struct driver_cpu_s  sparc_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = sparc_cpu_reg_init,
#ifdef CONFIG_ARCH_SMP
  .f_get_storage   = sparc_cpu_get_storage,
#endif
};

/************************************************************************/

static DEV_CLEANUP(sparc_cleanup);
static DEV_INIT(sparc_init);

static const struct devenum_ident_s  sparc_ids[] =
{
#ifdef CONFIG_FDT
  DEVENUM_FDTNAME_ENTRY("cpu:sparc"),
#endif
#ifdef CONFIG_ARCH_GAISLER
  DEVENUM_GAISLER_ENTRY(0x1, 0x003), /* leon 3 */
  DEVENUM_GAISLER_ENTRY(0x1, 0x048), /* leon 4 */
#endif
  { 0 }
};

const struct driver_s  sparc_drv =
{
  .desc           = "Sparc processor",
  .id_table       = sparc_ids,

  .f_init         = sparc_init,
  .f_cleanup      = sparc_cleanup,

  .classes        = {
    &sparc_cpu_drv,
#ifdef CONFIG_DEVICE_IRQ
    &sparc_icu_drv,
#endif
    0
  }
};

REGISTER_DRIVER(sparc_drv);

static DEV_INIT(sparc_init)
{
  struct sparc_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  assert(cpu_sparc_wincount() == CONFIG_CPU_SPARC_WINCOUNT);

  /* get processor device id specifed in resources */
  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
#ifdef CONFIG_ARCH_SMP
    PRINTK_RET(-ENOENT, "sparc: device has no ID resource")
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
  /* init sparc irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_SPARC_MAX_VECTOR, NULL);

# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->cls, sparc_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->cls, sparc_irq_handler);
# else
  if (id == 0)
    {
      CPU_LOCAL_SET(sparc_icu_dev, dev);
      cpu_interrupt_sethandler(sparc_irq_handler);
    }
# endif
#endif

  dev->drv = &sparc_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(sparc_cleanup)
{
  struct sparc_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
# endif
  /* detach sparc irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, ICU_SPARC_MAX_VECTOR);
#endif

  if (sizeof(*pv))
    mem_free(pv);
}

