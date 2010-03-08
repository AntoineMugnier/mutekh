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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/


#include <hexo/types.h>

#include <device/icu.h>
#include <device/device.h>
#include <device/driver.h>

#include <hexo/iospace.h>
#include <mutek/mem_alloc.h>
#include <hexo/interrupt.h>
#include <hexo/error.h>

#include <mutek/printk.h>

#include "icu-8259.h"

#include "icu-8259-private.h"

#include "8259.h"

DEVICU_ENABLE(icu_8259_enable)
{
  uint8_t			mask;

  if (irq < 8)
    {
      mask = pic_8259_getmask(dev->addr[ICU_ADDR_MASTER]);

      mask = enable
	? mask & ~(1 << irq)
	: mask |  (1 << irq);

      pic_8259_setmask(dev->addr[ICU_ADDR_MASTER], mask);
    }
  else
    {
      irq -= 8;

      mask = pic_8259_getmask(dev->addr[ICU_ADDR_SLAVE]);

      mask = enable
	? mask & ~(1 << irq)
	: mask |  (1 << irq);

      pic_8259_setmask(dev->addr[ICU_ADDR_SLAVE], mask);
    }
}

DEVICU_SETHNDL(icu_8259_sethndl)
{
  struct icu_8259_private_s	*pv = dev->drv_pv;
  struct icu_8259_handler_s	*h = pv->table + irq;

  device_obj_refnew(dev);

  h->hndl = hndl;
  h->data = data;

  return 0;
}

DEVICU_DELHNDL(icu_8259_delhndl)
{
  device_obj_refdrop(dev);

  /* FIXME */
  return 0;
}

static CPU_INTERRUPT_HANDLER(icu_8259_cpu_handler)
{
  struct device_s *dev = CPU_LOCAL_GET(cpu_interrupt_handler_dev);
  struct icu_8259_private_s	*pv = dev->drv_pv;
  struct icu_8259_handler_s	*h = pv->table + irq;

  /* reset interrupt line status on icu */
  if (irq < 8)
    pic_8259_irqend_master(pv->dev->addr[ICU_ADDR_MASTER], irq);
  else
    pic_8259_irqend_slave(pv->dev->addr[ICU_ADDR_MASTER],
			  pv->dev->addr[ICU_ADDR_SLAVE], irq - 8);

  /* call interrupt handler */
  if (h->hndl)
    h->hndl(h->data);
  else
    printk("lost interrupt %i\n", irq);
}

DEV_CLEANUP(icu_8259_cleanup)
{
  struct icu_8259_private_s	*pv = dev->drv_pv;

  mem_free(pv);
}


const struct driver_s	icu_8259_drv =
{
  .class		= device_class_icu,
  .f_init		= icu_8259_init,
  .f_irq        = (dev_irq_t *)icu_8259_cpu_handler,
  .f_cleanup		= icu_8259_cleanup,
  .f.icu = {
    .f_enable		= icu_8259_enable,
    .f_sethndl		= icu_8259_sethndl,
    .f_delhndl		= icu_8259_delhndl,
  }
};

DEV_INIT(icu_8259_init)
{
  struct icu_8259_private_s	*pv;

  dev->drv = &icu_8259_drv;

  if ((pv = mem_alloc(sizeof (*pv), (mem_scope_sys)))) /* FIXME allocation scope ? */
    {
      uint_fast8_t i;

      dev->drv_pv = pv;
      pv->dev = dev;

      for (i = 0; i < CPU_MAX_INTERRUPTS; i++)
	pv->table[i].hndl = NULL;

      cpu_interrupt_sethandler(icu_8259_cpu_handler);

      pic_8259_init(dev->addr[ICU_ADDR_MASTER], dev->addr[ICU_ADDR_SLAVE], CPU_HWINT_VECTOR);

      return 0;
    }

  return -ENOMEM;
}

