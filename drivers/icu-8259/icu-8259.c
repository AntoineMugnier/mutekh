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


#include <mutek/types.h>
#include <mutek/device.h>
#include <mutek/iospace.h>
#include <mutek/interrupt.h>

#include <mutek/drivers/icu-8259.h>

#include "8259.h"

struct icu_8259_handler_s
{
  dev_irq_t		*hndl;
  void			*data;
};

static struct icu_8259_handler_s icu_8259_table[CPU_MAX_INTERRUPTS] = { };

DEVICU_ENABLE(icu_8259_enable)
{
}

DEVICU_SETHNDL(icu_8259_sethndl)
{
  struct icu_8259_handler_s	*h = icu_8259_table + irq;
  uint8_t				mask;

  h->hndl = hndl;
  h->data = data;

  mask = pic_8259_getmask(dev->addr[ICU_ADDR_MASTER]);
  mask &= ~ (1 << irq);
  pic_8259_setmask(dev->addr[ICU_ADDR_MASTER], mask);

  return 0;
}

DEVICU_DELHNDL(icu_8259_delhndl)
{
  return 0;
}

DEV_CLEANUP(icu_8259_cleanup)
{
}

static CPU_INTERRUPT_HANDLER(icu_8259_cpu_handler)
{
  uint_fast8_t			line = irq;
  struct icu_8259_handler_s	*h = icu_8259_table + line;

  /* reset interrupt line status on icu */
  pic_8259_irqend_master(0x20, line);

  /* call interrupt handler */
  h->hndl(h->data);
}

DEV_INIT(icu_8259_init)
{
#ifndef CONFIG_STATIC_DRIVERS
  dev->f_cleanup	= icu_8259_cleanup;
  dev->icu.f_enable	= icu_8259_enable;
  dev->icu.f_sethndl	= icu_8259_sethndl;
  dev->icu.f_delhndl	= icu_8259_delhndl;
#endif

  cpu_interrupt_hw_sethandler(icu_8259_cpu_handler);

  pic_8259_init(dev->addr[ICU_ADDR_MASTER], dev->addr[ICU_ADDR_SLAVE], CPU_HWINT_VECTOR);

  return 0;
}

