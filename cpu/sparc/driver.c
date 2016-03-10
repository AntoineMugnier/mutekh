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

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/class/cpu.h>
#include <device/clock.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/soclib/mem_checker.h>
#endif

#define ICU_SPARC_MAX_VECTOR    	14  /* exclude nmi */

#if defined(CONFIG_CPU_SPARC_LEON3)
#define ICU_SPARC_SINKS_COUNT	1
#define SPARC_IRQ_SENSE_MODE    DEV_IRQ_SENSE_ID_BUS
#define SPARC_SINGLE_IRQ_EP
#elif defined(CONFIG_CPU_SPARC_SOCLIB)
#define ICU_SPARC_SINKS_COUNT	ICU_SPARC_MAX_VECTOR
#define SPARC_IRQ_SENSE_MODE    DEV_IRQ_SENSE_HIGH_LEVEL
#else
# error
#endif

struct sparc_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_sink_s	sinks[ICU_SPARC_SINKS_COUNT];
#endif

  struct cpu_tree_s node;
  struct dev_clock_sink_ep_s clk_ep;
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

#ifdef SPARC_SINGLE_IRQ_EP

  struct dev_irq_sink_s *sink = pv->sinks;
  return device_irq_sink_process(sink, irq);

#else
  if ( irq < ICU_SPARC_SINKS_COUNT ) {
    struct dev_irq_sink_s *sink = pv->sinks + irq;

    return device_irq_sink_process(sink, 0);
  }
#endif
}

static DEV_IRQ_SINK_UPDATE(sparc_icu_sink_update)
{
}

static DEV_ICU_GET_SINK(sparc_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct sparc_dev_private_s  *pv = dev->drv_pv;

  if (id < ICU_SPARC_SINKS_COUNT)
    return pv->sinks + id;
  return NULL;
}

static DEV_ICU_LINK(sparc_icu_link)
{
  return 0;
}

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

#if defined(CONFIG_ARCH_SMP) && defined(CONFIG_HEXO_USERMODE)
void * cpu_local_storage[CONFIG_ARCH_LAST_CPU_ID + 1]; /* used to restore cls reg when back from user mode */
#endif

static DEV_CPU_REG_INIT(sparc_cpu_reg_init)
{
  struct device_s *dev = accessor->dev;

#ifdef CONFIG_ARCH_SMP
  struct sparc_dev_private_s *pv = dev->drv_pv;

  assert(pv->node.cpu_id == cpu_id());

  /* set cpu local storage register base pointer */
  asm volatile("mov %0, %%g6" : : "r" (pv->node.cls));

# ifdef CONFIG_HEXO_USERMODE
  cpu_local_storage[pv->node.cpu_id] = pv->node.cls;
# endif

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

  extern __ldscript_symbol_t CPU_NAME_DECL(exception_vector);
  extern __ldscript_symbol_t CPU_NAME_DECL(exception_vector_end);
  soclib_mem_bypass_sp_check(&CPU_NAME_DECL(exception_vector), &CPU_NAME_DECL(exception_vector_end));

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
static DEV_CPU_GET_NODE(sparc_cpu_get_node)
{
  struct device_s *dev = accessor->dev;
  struct sparc_dev_private_s *pv = dev->drv_pv;
  return &pv->node;
}
#endif

/************************************************************************/

static DEV_CLEANUP(sparc_cleanup);
static DEV_INIT(sparc_init);
#define sparc_use dev_use_generic

DRIVER_DECLARE(sparc_drv, DRIVER_FLAGS_EARLY_INIT, "Sparc processor", sparc,
#ifdef CONFIG_DEVICE_IRQ
               DRIVER_ICU_METHODS(sparc_icu),
#endif
               DRIVER_CPU_METHODS(sparc_cpu));

DRIVER_REGISTER(sparc_drv
#ifdef CONFIG_LIBFDT
                ,DEV_ENUM_FDTNAME_ENTRY("cpu:sparc")
#endif
#ifdef CONFIG_ARCH_GAISLER
                ,DEV_ENUM_GAISLER_ENTRY(0x1, 0x003) /* leon 3 */
                ,DEV_ENUM_GAISLER_ENTRY(0x1, 0x048) /* leon 4 */
#endif
                );

static DEV_INIT(sparc_init)
{
  struct sparc_dev_private_s  *pv;


  assert(cpu_sparc_wincount() == CONFIG_CPU_SPARC_WINCOUNT);

  /* get processor device id specifed in resources */
  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
    PRINTK_RET(-ENOENT, "sparc: device has no ID resource")
      ;

  pv = mem_alloc_cpu(sizeof (*pv), (mem_scope_sys), id);

  if ( pv == NULL )
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if (cpu_tree_node_init(&pv->node, id, dev))
    goto err_pv;

  if (dev_drv_clock_init(dev, &pv->clk_ep, 0, DEV_CLOCK_EP_POWER_CLOCK |
                         DEV_CLOCK_EP_SINK_SYNC, NULL))
    goto err_node;

#ifdef CONFIG_DEVICE_IRQ
  /* init sparc irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_SPARC_SINKS_COUNT,
                       &sparc_icu_sink_update, SPARC_IRQ_SENSE_MODE);

# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->node.cls, sparc_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->node.cls, sparc_irq_handler);
# else
  if (id == CONFIG_ARCH_BOOTSTRAP_CPU_ID)
    {
      CPU_LOCAL_SET(sparc_icu_dev, dev);
      cpu_interrupt_sethandler(sparc_irq_handler);
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

static DEV_CLEANUP(sparc_cleanup)
{
  struct sparc_dev_private_s *pv = dev->drv_pv;

  dev_drv_clock_cleanup(dev, &pv->clk_ep);

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);

  return 0;
}

