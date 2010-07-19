/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2009

*/

#include "icu-nios.h"

#include "icu-nios-private.h"

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <device/device.h>
#include <device/driver.h>
#include <hexo/iospace.h>
#include <mutek/mem_alloc.h>
#include <hexo/interrupt.h>

#include <mutek/printk.h>

void icu_nios_update(struct device_s *dev)
{
  /*
   * Prevent the Nios2-specific code to be compiled on heterogeneous
   * builds.
	 */
#if defined(__nios)
  struct icu_nios_private_s	*pv = dev->drv_pv;

  printk("ICU nios2 mask %x for cpu %d\n", pv->mask, cpu_id());

  reg_t ienable = cpu_nios_read_ctrl_reg(3);
  enable = ienable | ((uint32_t)pv->mask);
  cpu_nios_write_ctrl_reg(3, ienable);

  pv->must_update = 0;
#endif
}

DEVICU_ENABLE(icu_nios_enable)
{
  struct icu_nios_private_s	*pv = dev->drv_pv;
  reg_t mask = 1 << (irq);

//  printk("DEVICU_ENABLE: mask %x (enable: %d) - pv->mask:%x \n", mask, enable, pv->mask);

  if (enable)
    pv->mask |= mask;
  else
    pv->mask &= ~mask;
  pv->must_update = 1;

  return 0;
}

DEVICU_SETHNDL(icu_nios_sethndl)
{
  struct icu_nios_private_s	*pv = dev->drv_pv;
  struct icu_nios_handler_s	*h = pv->table + irq;

  h->hndl = hndl;
  h->data = data;

  return 0;
}

DEVICU_DELHNDL(icu_nios_delhndl)
{
  struct icu_nios_private_s	*pv = dev->drv_pv;
  struct icu_nios_handler_s	*h = pv->table + irq;

  reg_t mask = 1 << (irq);
  assert( (mask & pv->mask) == 0 && "You should have disabled this interrupt already" );

  printk("DEVICU_DELHNDL: mask %x - pv->mask:%x \n", mask, pv->mask);

  (void) (mask&pv->mask);

  h->hndl = NULL;
  h->data = NULL;

  return 0;
}

static CPU_INTERRUPT_HANDLER(icu_nios_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(cpu_interrupt_handler_dev);
  struct icu_nios_private_s	*pv = dev->drv_pv;
  struct icu_nios_handler_s	*h;
  if ( !irq )
    return;
  uint32_t irq_no = __builtin_ctz(irq);

   printk("DEVICU_ CPU_INTERRUPT_HANDLER: irq_no %d  \n", irq_no);

 if (irq_no >= ICU_NIOS_MAX_VECTOR) {
    printk("Nios2 %d got spurious interrupt %i\n", cpu_id(), irq_no);
    return;
  }

  h = pv->table + irq_no;

  if (h->hndl)
    h->hndl(h->data);
  else
    printk("NIOS2 %d lost interrupt %i\n", cpu_id(), irq_no);

  if ( pv->must_update )
    icu_nios_update(dev);
}

DEV_CLEANUP(icu_nios_cleanup)
{
  struct icu_nios_private_s *pv = dev->drv_pv;

//  printk("DEVICU_CLEANUP \n");

  mem_free(pv);
}

static const struct devenum_ident_s	icu_nios_ids[] =
{
	DEVENUM_FDTNAME_ENTRY("cpu:nios", 0, 0),
	{ 0 }
};


const struct driver_s	icu_nios_drv =
{
  .class		= device_class_icu,
  .id_table   = icu_nios_ids,
  .f_init		= icu_nios_init,
  .f_irq      = (dev_irq_t *)icu_nios_handler,
  .f_cleanup		= icu_nios_cleanup,
  .f.icu = {
    .f_enable		= icu_nios_enable,
    .f_sethndl		= icu_nios_sethndl,
    .f_delhndl		= icu_nios_delhndl,
  }
};

REGISTER_DRIVER(icu_nios_drv);

DEV_INIT(icu_nios_init)
{
  struct icu_nios_private_s	*pv;

  dev->drv = &icu_nios_drv;

// printk("DEVICU_INIT \n");

  /* FIXME allocation scope ? */
  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if ( pv == NULL )
    goto memerr;

  dev->drv_pv = pv;

  memset(pv, 0, sizeof(*pv));

  pv->must_update = 1;

  return 0;

 memerr:
  return -ENOMEM;
}
