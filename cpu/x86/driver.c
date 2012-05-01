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
#include <hexo/segment.h>
#include <hexo/iospace.h>

#include <cpu/hexo/pmode.h>
#include <cpu/hexo/msr.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/class/cpu.h>
#include <device/class/timer.h>
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
  struct dev_irq_ep_s	sinks[CPU_X86_IRQ_SINKS];
#endif

#ifdef CONFIG_ARCH_SMP
  uint_fast8_t id;
  void *cls;                    //< cpu local storage
  uint16_t cls_seg;     //< cpu local storage segment
#endif

#if defined(CONFIG_HEXO_USERMODE) || defined(CONFIG_HEXO_INTERRUPT_STACK)
  uint8_t *cpu_interrupt_stack;
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

  struct dev_irq_ep_s *sink = &pv->sinks[0];
  int_fast16_t id = irq;

  sink->process(sink, &id);
}

#ifdef CONFIG_HEXO_IPI
static DEVICU_SETUP_IPI_EP(x86_icu_setup_ipi_ep)
{
  abort(); // FIXME
  return -1;
}
#endif

static DEVICU_DISABLE_SINK(x86_icu_disable_sink)
{
}

static DEVICU_GET_SINK(x86_icu_get_sink)
{
  struct device_s *dev = idev->dev;
  struct x86_dev_private_s  *pv = dev->drv_pv;

  if (icu_in_id >= CPU_X86_IRQ_SINKS)
    return NULL;

  return &pv->sinks[icu_in_id];
}

static const struct driver_icu_s  x86_icu_drv =
{
  .class_          = DRIVER_CLASS_ICU,
  .f_get_sink     = x86_icu_get_sink,
  .f_disable_sink = x86_icu_disable_sink,
#ifdef CONFIG_HEXO_IPI
  .f_setup_ipi_ep = x86_icu_setup_ipi_ep,
#endif
};

#endif

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

/** pointer to cpu local storage */
CPU_LOCAL void *__cpu_data_base;
/** pointer to context local storage */
CONTEXT_LOCAL void *__context_data_base;

static DEVCPU_REG_INIT(x86_cpu_reg_init)
{
  struct device_s *dev = cdev->dev;
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
  assert(pv->id == cpu_id());

  /* setup cpu local storage segment */
  cpu_x86_datasegfs_use(pv->cls_seg, 0);
  CPU_LOCAL_SET(__cpu_data_base, pv->cls);

# ifdef CONFIG_DEVICE_IRQ
  /* enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */
# endif

#endif

#ifdef CONFIG_HEXO_USERMODE
  cpu_x86_taskseg_use(pv->tss_sel);

# ifdef CONFIG_CPU_X86_SYSENTER
  cpu_x86_write_msr(SYSENTER_CS_MSR, CPU_X86_SEG_SEL(ARCH_GDT_CODE_INDEX, 0));
  cpu_x86_write_msr(SYSENTER_EIP_MSR, (uintptr_t)x86_interrupt_sys_enter);
  cpu_x86_write_msr(SYSENTER_ESP_MSR, pv->tss.esp0);
# endif
#endif

  CPU_LOCAL_SET(cpu_device, dev);
}


#ifdef CONFIG_ARCH_SMP
static DEVCPU_GET_STORAGE(x86_cpu_get_storage)
{
  struct device_s *dev = cdev->dev;
  struct x86_dev_private_s *pv = dev->drv_pv;
  return pv->cls;
}
#endif


static const struct driver_cpu_s  x86_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = x86_cpu_reg_init,
#ifdef CONFIG_ARCH_SMP
  .f_get_storage   = x86_cpu_get_storage,
#endif
};

/************************************************************************
        Timer driver part
************************************************************************/

static DEVTIMER_GET_VALUE(x86_timer_get_value)
{
#ifdef CONFIG_ARCH_SMP
  struct device_s *dev = tdev->dev;
  struct x86_dev_private_s *pv = dev->drv_pv;

  assert(pv->id == cpu_id());
#endif

  uint32_t      low, high;
  asm volatile("rdtsc" : "=a" (low), "=d" (high));

  return (low | ((uint64_t)high << 32));
}

static DEVTIMER_RESOLUTION(x86_timer_resolution)
{
  error_t err = 0;

  if (res)
    {
      if (*res != 0)
        err = -ENOTSUP;
      *res = DEVTIMER_RES_FIXED_POINT(1.0);
    }

  if (max)
    *max = 0xffffffffffffffffULL;
}

static const struct driver_timer_s  x86_timer_drv =
{
  .class_          = DRIVER_CLASS_TIMER,
  .f_get_value     = x86_timer_get_value,
  .f_resolution    = x86_timer_resolution,
};

/************************************************************************/

static DEV_CLEANUP(x86_cleanup);
static DEV_INIT(x86_init);

static const struct devenum_ident_s  x86_ids[] =
{
#ifdef CONFIG_FDT
  DEVENUM_FDTNAME_ENTRY("cpu:x86"),
#endif
  { 0 }
};

const struct driver_s  x86_drv =
{
  .desc           = "x86 32-bits processor",
  .id_table       = x86_ids,

  .f_init         = x86_init,
  .f_cleanup      = x86_cleanup,

  .classes        = {
    &x86_cpu_drv,
#ifdef CONFIG_DEVICE_IRQ
    &x86_icu_drv,
#endif
    &x86_timer_drv,
    0
  }
};

REGISTER_DRIVER(x86_drv);

static DEV_INIT(x86_init)
{
  struct x86_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  /* get processor device id specifed in resources */
  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
#ifdef CONFIG_ARCH_SMP
    PRINTK_RET(-ENOENT, "x86: device has no ID resource")
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
  if (!pv->cls)
    goto err1;

  pv->cls_seg = cpu_x86_segment_alloc((uintptr_t)pv->cls, arch_cpudata_size(), CPU_X86_SEG_DATA_UP_RW);
  if (!pv->cls_seg)
    goto err2;

  pv->id = id;
#endif

#ifdef CONFIG_HEXO_USERMODE
  pv->cpu_interrupt_stack = mem_alloc(CONFIG_HEXO_INTERRUPT_STACK_SIZE, (mem_scope_sys));
  if (!pv->cpu_interrupt_stack)
    goto err3;

  pv->tss.ss0 = ARCH_GDT_DATA_INDEX << 3;
  pv->tss.esp0 = (uintptr_t)cpu_interrupt_stack + CONFIG_HEXO_INTERRUPT_STACK_SIZE;

  pv->tss_seg = cpu_x86_segment_alloc((uintptr_t)&pv->tss, sizeof(pv->tss), CPU_X86_SEG_CONTEXT32);
  if (!pv->tss_seg)
    goto err4;
#endif

#ifdef CONFIG_DEVICE_IRQ
  /* init x86 irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, CPU_X86_IRQ_SINKS, NULL);

# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->cls, x86_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->cls, x86_irq_handler);
# else
  if (id == 0)
    {
      CPU_LOCAL_SET(x86_icu_dev, dev);
      cpu_interrupt_sethandler(x86_irq_handler);
    }
# endif
#endif

  dev->drv = &x86_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
#ifdef CONFIG_HEXO_USERMODE
 err4:
  mem_free(pv->cpu_interrupt_stack);
 err3:
#endif
#ifdef CONFIG_ARCH_SMP
  cpu_x86_segdesc_free(pv->cls_seg);
 err2:
  mem_free(pv->cls);
 err1:
#endif
  if (sizeof(*pv))
    mem_free(pv);
  return -1;
}

static DEV_CLEANUP(x86_cleanup)
{
  struct x86_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ

# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
# endif

  /* detach x86 irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, CPU_X86_IRQ_SINKS);
#endif

#ifdef CONFIG_HEXO_USERMODE
  mem_free(pv->cpu_interrupt_stack);
  cpu_x86_segdesc_free(pv->tss_seg);
#endif

#ifdef CONFIG_ARCH_SMP
  cpu_x86_segdesc_free(pv->cls_seg);
  mem_free(pv->cls);
#endif

  if (sizeof(*pv))
    mem_free(pv);
}

