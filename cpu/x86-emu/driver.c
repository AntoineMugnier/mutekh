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

    Copyright (c) 2012 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>

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

CPU_LOCAL void *__context_data_base;
CPU_LOCAL void *__cpu_data_base;

struct x86_emu_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
#define ICU_X86_EMU_MAX_VECTOR	1
  struct dev_irq_ep_s	sinks[ICU_X86_EMU_MAX_VECTOR];
#endif

#ifdef CONFIG_ARCH_SMP
  __compiler_sint_t id;
  void *cls;            //< cpu local storage
#endif
};

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

static DEVCPU_REG_INIT(x86_emu_cpu_reg_init)
{
  struct device_s *dev = cdev->dev;
  __unused__ struct x86_emu_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  assert(pv->id == cpu_id());

  /* we use this variable in non shared page as a cls register
   * that's why we do not use CPU_LOCAL_SET here. */
  __cpu_data_base = pv->cls;

# ifdef CONFIG_DEVICE_IRQ
  /* enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */

# endif
#endif

#if defined(CONFIG_CPU_X86_ALIGNCHECK)
   /* enable alignment check */
    asm volatile("	pushf						\n"
  	       "	orl	$0x40000, (%esp)			\n"
  	       "	popf						\n");
#endif

  CPU_LOCAL_SET(cpu_device, dev);
}


#ifdef CONFIG_ARCH_SMP
static DEVCPU_GET_STORAGE(x86_emu_cpu_get_storage)
{
  struct device_s *dev = cdev->dev;
  struct x86_emu_dev_private_s *pv = dev->drv_pv;
  return pv->cls;
}
#endif


static const struct driver_cpu_s  x86_emu_cpu_drv =
{
  .class_          = DRIVER_CLASS_CPU,
  .f_reg_init      = x86_emu_cpu_reg_init,
#ifdef CONFIG_ARCH_SMP
  .f_get_storage   = x86_emu_cpu_get_storage,
#endif
};

static DEV_CLEANUP(x86_emu_cleanup);
static DEV_INIT(x86_emu_init);

const struct driver_s emu_cpu_drv =
{
  .desc           = "x86 32-bits UNIX process cpu",

  .f_init         = x86_emu_init,
  .f_cleanup      = x86_emu_cleanup,

  .classes        = {
    &x86_emu_cpu_drv,
    0
  }
};

REGISTER_DRIVER(emu_cpu_drv);

static DEV_INIT(x86_emu_init)
{
  struct x86_emu_dev_private_s  *pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  /* get processor device id specifed in resources */
  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
#ifdef CONFIG_ARCH_SMP
    PRINTK_RET(-ENOENT, "cpu: device has no ID resource")
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
  /* init mips irq sink end-points */
  device_irq_sink_init(dev, pv->sinks, ICU_X86_EMU_MAX_VECTOR);

# ifdef CONFIG_ARCH_SMP
  CPU_LOCAL_CLS_SET(pv->cls, x86_emu_icu_dev, dev);
  cpu_interrupt_cls_sethandler(pv->cls, x86_emu_irq_handler);
# else
  if (id == 0)
    {
      CPU_LOCAL_SET(x86_emu_icu_dev, dev);
      cpu_interrupt_sethandler(x86_emu_irq_handler);
    }
# endif
#endif

  dev->drv = &emu_cpu_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
#ifdef CONFIG_ARCH_SMP
 err_mem:
  if (sizeof(*pv))
    mem_free(pv);
  return -1;
#endif
}

static DEV_CLEANUP(x86_emu_cleanup)
{
  struct x86_emu_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_DEVICE_IRQ
# ifdef CONFIG_ARCH_SMP
  /* Disable all irq lines. */
  reg_t status = cpu_x86_emu_mfc0(CPU_X86_EMU_STATUS, 0);
  status &= ~0xfc00;
  cpu_x86_emu_mtc0(CPU_X86_EMU_STATUS, 0, status);
# endif
  /* detach mips irq sink end-points */
  device_irq_sink_unlink(dev, pv->sinks, ICU_X86_EMU_MAX_VECTOR);
#endif

  if (sizeof(*pv))
    mem_free(pv);
}

