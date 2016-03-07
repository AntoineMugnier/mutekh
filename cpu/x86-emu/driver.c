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

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/icu.h>
#include <device/class/cpu.h>
#include <device/class/timer.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

CPU_LOCAL void *__cpu_data_base;

#define ICU_X86_EMU_MAX_VECTOR 1

struct x86_emu_dev_private_s
{
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_sink_s	sinks[ICU_X86_EMU_MAX_VECTOR];
#endif

  struct cpu_tree_s node;
};

/************************************************************************
        CPU driver part
************************************************************************/

CPU_LOCAL struct device_s *cpu_device = NULL;

static DEV_CPU_REG_INIT(x86_emu_cpu_reg_init)
{
  struct device_s *dev = accessor->dev;
  __unused__ struct x86_emu_dev_private_s *pv = dev->drv_pv;

#ifdef CONFIG_ARCH_SMP
  assert(pv->node.cpu_id == cpu_id());

  /* we use this variable in non shared page as a cls register
   * that's why we do not use CPU_LOCAL_SET here. */
  __cpu_data_base = pv->node.cls;

# ifdef CONFIG_DEVICE_IRQ
  /* enable all irq lines. On SMP platforms other CPUs won't be able to enable these lines later. */

# endif
#endif

#if defined(CONFIG_CPU_X86_EMU_ALIGNCHECK)
   /* enable alignment check */
    asm volatile("	pushf						\n"
  	       "	orl	$0x40000, (%esp)			\n"
  	       "	popf						\n");
#endif

  CPU_LOCAL_SET(cpu_device, dev);
}


#ifdef CONFIG_ARCH_SMP
static DEV_CPU_GET_NODE(x86_emu_cpu_get_node)
{
  struct device_s *dev = accessor->dev;
  struct x86_emu_dev_private_s *pv = dev->drv_pv;
  return &pv->node;
}
#endif


/************************************************************************/

static DEV_CLEANUP(x86_emu_cleanup);
static DEV_INIT(x86_emu_init);
#define x86_emu_use dev_use_generic

DRIVER_DECLARE(emu_cpu_drv, DRIVER_FLAGS_EARLY_INIT, "x86 32-bits UNIX process cpu", x86_emu,
               DRIVER_CPU_METHODS(x86_emu_cpu));

DRIVER_REGISTER(emu_cpu_drv);

static DEV_INIT(x86_emu_init)
{
  struct x86_emu_dev_private_s  *pv;


  /* get processor device id specifed in resources */
  uintptr_t id = 0;
  if (device_res_get_uint(dev, DEV_RES_ID, 0, &id, NULL))
    PRINTK_RET(-ENOENT, "cpu: device has no ID resource")
      ;

  /* allocate device private data */
  pv = mem_alloc_cpu(sizeof (*pv), (mem_scope_sys), id);

  if ( pv == NULL )
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  dev->drv_pv = pv;

  if (cpu_tree_node_init(&pv->node, id, dev))
    goto err_pv;

#ifdef CONFIG_DEVICE_IRQ
  device_irq_sink_init(dev, pv->sinks, ICU_X86_EMU_MAX_VECTOR,
                       DEV_IRQ_SENSE_ID_BUS);
#endif

  if (cpu_tree_insert(&pv->node))
    goto err_node;


  return 0;

 err_node:
  cpu_tree_node_cleanup(&pv->node);
 err_pv:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(x86_emu_cleanup)
{
  struct x86_emu_dev_private_s *pv = dev->drv_pv;

  cpu_tree_remove(&pv->node);
  cpu_tree_node_cleanup(&pv->node);

  mem_free(pv);

  return 0;
}

