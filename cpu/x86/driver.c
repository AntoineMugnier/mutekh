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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2012

*/

#include <string.h>

#include <hexo/types.h>
#include <hexo/interrupt.h>
#include <hexo/local.h>
#include <hexo/iospace.h>

#include <cpu/hexo/pmode.h>
#include <cpu/hexo/msr.h>

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

#include "apic.h"
#include "cpu_private.h"

struct x86_dev_private_s
{
#ifdef CONFIG_HEXO_USERMODE
  struct cpu_x86_tss_s tss;
  uint16_t tss_seg;
#endif

#ifdef CONFIG_DEVICE_IRQ

#ifdef CONFIG_CPU_X86_APIC
# define CPU_X86_IRQ_SINKS 1
#else
# define CPU_X86_IRQ_SINKS 1
#endif
  struct dev_irq_sink_s	sinks[CPU_X86_IRQ_SINKS];
#endif

#ifdef CONFIG_ARCH_SMP
  uint16_t cls_seg;     //< cpu local storage segment
#endif

#if defined(CONFIG_HEXO_USERMODE) || defined(CONFIG_HEXO_INTERRUPT_STACK)
  uint8_t *cpu_interrupt_stack;
#endif

  struct cpu_tree_s node;
  struct dev_freq_s freq;
  struct dev_clock_sink_ep_s clk_ep;
#ifdef CONFIG_DEVICE_CLOCK
  dev_timer_cfgrev_t timer_rev;
#endif
};

cpu_id_t cpu_id(void)
{
#ifdef CONFIG_ARCH_SMP
  uintptr_t x = cpu_x86_read_msr(IA32_APIC_BASE_MSR) & ~0xfff;
  return cpu_mem_read_32(x + APIC_REG_LAPIC_ID) >> 24;
#else
  return 0;
#endif
}

/************************************************************************
        Interrupts controller driver part
************************************************************************/

#ifdef CONFIG_DEVICE_IRQ

static CPU_LOCAL struct device_s *x86_icu_dev;

static CPU_INTERRUPT_HANDLER(x86_irq_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(x86_icu_dev);
  struct x86_dev_private_s  *pv = dev->drv_pv;

  struct dev_irq_sink_s *sink = &pv->sinks[0];

  device_irq_sink_process(sink, irq);
}

static DEV_ICU_GET_SINK(x86_icu_get_sink)
{
  struct device_s *dev = accessor->dev;
  struct x86_dev_private_s  *pv = dev->drv_pv;

  if (id < CPU_X86_IRQ_SINKS)
    return pv->sinks + id;
  return NULL;
}

static DEV_IRQ_SINK_UPDATE(x86_sink_update)
{
}

#define x86_icu_link device_icu_dummy_link

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

/** pointer to cpu local storage */
CPU_LOCAL void *__cpu_data_base;

static DEV_CPU_REG_INIT(x86_cpu_reg_init)
{
  struct device_s *dev = accessor->dev;
  struct x86_dev_private_s *pv = dev->drv_pv;

  /* set GDT pointer */
  cpu_x86_set_gdt(gdt, ARCH_GDT_SIZE);

  /* set IDT pointer */
  cpu_x86_set_idt(cpu_idt, CPU_MAX_INTERRUPTS);

  /* setup segments */
  cpu_x86_dataseg_use(ARCH_GDT_DATA_INDEX, 0);
  cpu_x86_stackseg_use(ARCH_GDT_DATA_INDEX, 0);
  cpu_x86_codeseg_use(ARCH_GDT_CODE_INDEX, 0);

#ifdef CONFIG_ARCH_SMP
  assert(pv->node.cpu_id == cpu_id());

  /* setup cpu local storage segment */
  cpu_x86_datasegfs_use(pv->cls_seg, 0);
  CPU_LOCAL_SET(__cpu_data_base, pv->node.cls);

# ifdef CONFIG_DEVICE_IRQ
  /* enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
# endif

#endif

#ifdef CONFIG_HEXO_USERMODE
  cpu_x86_taskseg_use(pv->tss_seg);

# ifdef CONFIG_CPU_X86_SYSENTER
  cpu_x86_write_msr(SYSENTER_CS_MSR, CPU_X86_SEG_SEL(ARCH_GDT_CODE_INDEX, 0));
  cpu_x86_write_msr(SYSENTER_EIP_MSR, (uintptr_t)x86_interrupt_sys_enter);
  cpu_x86_write_msr(SYSENTER_ESP_MSR, pv->tss.esp0);
# endif
#endif

  CPU_LOCAL_SET(cpu_device, dev);
}


#ifdef CONFIG_ARCH_SMP
static DEV_CPU_GET_NODE(x86_cpu_get_node)
{
  struct device_s *dev = accessor->dev;
  struct x86_dev_private_s *pv = dev->drv_pv;
  return &pv->node;
}
#endif


/************************************************************************
        Timer driver part
************************************************************************/

#ifdef CONFIG_CPU_X86_TIMER_CYCLECOUNTER

static DEV_TIMER_GET_VALUE(x86_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct x86_dev_private_s *pv = dev->drv_pv;

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

  uint32_t      low, high;
  asm volatile("rdtsc" : "=a" (low), "=d" (high));

  *value = low | ((uint64_t)high << 32);

  return 0;
}

static DEV_TIMER_CONFIG(x86_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct x86_dev_private_s *pv = dev->drv_pv;
  error_t err = 0;

      if (res > 1)
        err = -ERANGE;
      if (cfg)
        {
          cfg->max = 0xffffffffffffffffULL;
          cfg->freq = pv->freq;
          cfg->cap = DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_KEEPVALUE | DEV_TIMER_CAP_TICKLESS;
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

#endif

/************************************************************************/


static DEV_USE(x86_use)
{
  switch (op)
    {
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_SINK_FREQ_CHANGED: {
      struct dev_clock_notify_s *chg = param;
      struct dev_clock_sink_ep_s *sink = chg->sink;
      struct device_s *dev = sink->dev;
      struct x86_dev_private_s *pv = dev->drv_pv;
      pv->freq = chg->freq;
      pv->timer_rev += 2;
      return 0;
    }
#endif
    default:
      return dev_use_generic(param, op);
    }
}

#define x86_timer_request (dev_timer_request_t*)&dev_driver_notsup_fcn
#define x86_timer_cancel  (dev_timer_cancel_t*)&dev_driver_notsup_fcn

static DEV_INIT(x86_init)
{
  struct x86_dev_private_s  *pv;


  /* get processor device id specifed in resources */
  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
    PRINTK_RET(-ENOENT, "x86: device has no ID resource")
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

#ifdef CONFIG_ARCH_SMP
  extern __ldscript_symbol_t __cpu_data_start, __cpu_data_end;

  pv->cls_seg = cpu_x86_segment_alloc((uintptr_t)pv->node.cls,
                                      (char*)&__cpu_data_end - (char*)&__cpu_data_start,
                                      CPU_X86_SEG_DATA_UP_RW);
  if (!pv->cls_seg)
    goto err_clk;
#endif

#ifdef CONFIG_HEXO_USERMODE
  pv->cpu_interrupt_stack = mem_alloc(CONFIG_HEXO_INTERRUPT_STACK_SIZE, (mem_scope_sys));
  if (!pv->cpu_interrupt_stack)
    goto err_cls_seg;

  pv->tss.ss0 = ARCH_GDT_DATA_INDEX << 3;
  pv->tss.esp0 = (uintptr_t)pv->cpu_interrupt_stack + CONFIG_HEXO_INTERRUPT_STACK_SIZE;

  pv->tss_seg = cpu_x86_segment_alloc((uintptr_t)&pv->tss, sizeof(pv->tss), CPU_X86_SEG_CONTEXT32);
  if (!pv->tss_seg)
    goto err_irq_stack;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* init x86 irq sink endpoints */
  device_irq_sink_init(dev, pv->sinks, CPU_X86_IRQ_SINKS,
                       &x86_sink_update, DEV_IRQ_SENSE_ID_BUS);

# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->node.cls, x86_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->node.cls, x86_irq_handler);
# else
  if (id == CONFIG_ARCH_BOOTSTRAP_CPU_ID)
    {
      CPU_LOCAL_SET(x86_icu_dev, dev);
      cpu_interrupt_sethandler(x86_irq_handler);
    }
# endif
#endif

  if (cpu_tree_insert(&pv->node))
    goto err_tss_seg;


  return 0;

 err_tss_seg:
#ifdef CONFIG_HEXO_USERMODE
  cpu_x86_segdesc_free(pv->tss_seg);
 err_irq_stack:
  mem_free(pv->cpu_interrupt_stack);
#endif
 err_cls_seg:
#ifdef CONFIG_ARCH_SMP
  cpu_x86_segdesc_free(pv->cls_seg);
#endif
 err_clk:
  dev_drv_clock_cleanup(dev, &pv->clk_ep);
 err_node:
  cpu_tree_node_cleanup(&pv->node);
 err_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(x86_cleanup)
{
  struct x86_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_HEXO_USERMODE
  mem_free(pv->cpu_interrupt_stack);
  cpu_x86_segdesc_free(pv->tss_seg);
#endif

#ifdef CONFIG_ARCH_SMP
  cpu_x86_segdesc_free(pv->cls_seg);
#endif

  dev_drv_clock_cleanup(dev, &pv->clk_ep);

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(x86_drv, DRIVER_FLAGS_EARLY_INIT, "x86 32-bit processor", x86,
#ifdef CONFIG_DEVICE_IRQ
               DRIVER_ICU_METHODS(x86_icu),
#endif
#ifdef CONFIG_CPU_X86_TIMER_CYCLECOUNTER
               DRIVER_TIMER_METHODS(x86_timer),
#endif
               DRIVER_CPU_METHODS(x86_cpu));

DRIVER_REGISTER(x86_drv,
                DEV_ENUM_FDTNAME_ENTRY("cpu:x86"));

