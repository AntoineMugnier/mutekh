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
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/mem_checker.h>
#endif

#define ICU_SPARC_MAX_VECTOR    	14  /* exclude nmi */

#if defined(CONFIG_CPU_SPARC_LEON3)
#define ICU_SPARC_SINKS_COUNT	1
#define SPARC_IRQ_SENSE_MODE    DEV_IRQ_SENSE_RISING_EDGE
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
  struct dev_irq_ep_s	sinks[ICU_SPARC_SINKS_COUNT];
#if defined(SPARC_SINGLE_IRQ_EP) && defined(CONFIG_DEVICE_IRQ_BYPASS)
  struct dev_irq_bypass_s bypass[ICU_SPARC_MAX_VECTOR];
#endif
#endif

  struct cpu_tree_s node;
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
  int_fast16_t id = irq;

# ifdef CONFIG_DEVICE_IRQ_BYPASS
  struct dev_irq_ep_s *src = pv->bypass[id].src;
  if (src)
    return src->process(src, &id);
# endif

  struct dev_irq_ep_s *sink = pv->sinks;
  return sink->process(sink, &id);

#else
  if ( irq < ICU_SPARC_SINKS_COUNT ) {
    struct dev_irq_ep_s *sink = pv->sinks + irq;
    int_fast16_t id = 0;

    return sink->process(sink, &id);
  }
#endif
}

static DEV_ICU_GET_ENDPOINT(sparc_icu_get_endpoint)
{
  struct device_s *dev = accessor->dev;
  struct sparc_dev_private_s  *pv = dev->drv_pv;

  switch (type)
    {
    case DEV_IRQ_EP_SINK:
      if (id < ICU_SPARC_SINKS_COUNT)
        return pv->sinks + id;
      return NULL;

#if defined(SPARC_SINGLE_IRQ_EP) && defined(CONFIG_DEVICE_IRQ_BYPASS)
    case DEV_IRQ_EP_BYPASS:
      if (id < ICU_SPARC_MAX_VECTOR)
        return pv->bypass + id;
      return NULL;
#endif

    default:
      return NULL;
    }
}

static DEV_ICU_ENABLE_IRQ(sparc_icu_enable_irq)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct sparc_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  if (!arch_cpu_irq_affinity_test(dev, dev_ep))
    return 0;
#endif

#if defined(SPARC_SINGLE_IRQ_EP)
  if (irq_id >= ICU_SPARC_MAX_VECTOR)
    return 0;

# if defined(CONFIG_DEVICE_IRQ_BYPASS)
  /* try to create a bypass link or make bypass unusable for this irq */
  device_irq_bypass_link(src, pv->bypass + irq_id);
# endif

#else // ! SPARC_SINGLE_IRQ_EP

  /* inputs are single wire, logical irq id must be 0 */
  if (irq_id > 0)
    return 0;
#endif

#if !defined(CONFIG_ARCH_SMP)
  /* Enable irq line. On SMP platforms, all lines are already enabled on init. */
#endif

  return 1;
}

const struct driver_icu_s  sparc_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_endpoint  = sparc_icu_get_endpoint,
  .f_enable_irq    = sparc_icu_enable_irq,
};

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

# ifdef CONFIG_DEVICE_IRQ
  /* Enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
# endif
#endif

  CPU_LOCAL_SET(cpu_device, dev);

# ifdef CONFIG_HEXO_USERMODE
  cpu_local_storage[pv->node.cpu_id] = pv->node.cls;
# endif

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

const struct driver_cpu_s  sparc_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = sparc_cpu_reg_init,
#ifdef CONFIG_ARCH_SMP
  .f_get_node   = sparc_cpu_get_node,
#endif
};

/************************************************************************/

static DEV_CLEANUP(sparc_cleanup);
static DEV_INIT(sparc_init);

static const struct dev_enum_ident_s  sparc_ids[] =
{
#ifdef CONFIG_LIBFDT
  DEV_ENUM_FDTNAME_ENTRY("cpu:sparc"),
#endif
#ifdef CONFIG_ARCH_GAISLER
  DEV_ENUM_GAISLER_ENTRY(0x1, 0x003), /* leon 3 */
  DEV_ENUM_GAISLER_ENTRY(0x1, 0x048), /* leon 4 */
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
    PRINTK_RET(-ENOENT, "sparc: device has no ID resource")
      ;

  pv = mem_alloc_cpu(sizeof (*pv), (mem_scope_sys), id);

  if ( pv == NULL )
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if (cpu_tree_node_init(&pv->node, id, dev))
    goto err_pv;

#ifdef CONFIG_DEVICE_IRQ
  /* init sparc irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_SPARC_SINKS_COUNT,
                       SPARC_IRQ_SENSE_MODE);

# if defined(SPARC_SINGLE_IRQ_EP) && defined(CONFIG_DEVICE_IRQ_BYPASS)
  device_irq_bypass_init(pv->bypass, ICU_SPARC_MAX_VECTOR);
# endif

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
    goto err_node;

  dev->drv = &sparc_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_node:
  cpu_tree_node_cleanup(&pv->node);
 err_pv:
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
# if defined(SPARC_SINGLE_IRQ_EP) && defined(CONFIG_DEVICE_IRQ_BYPASS)
  device_irq_bypass_cleanup(pv->bypass, ICU_SPARC_MAX_VECTOR);
# endif
  /* detach sparc irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, ICU_SPARC_SINKS_COUNT);
#endif

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);
}

