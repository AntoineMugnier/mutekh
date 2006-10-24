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

#include "icu-soclib.h"

#include "icu-soclib-private.h"

#include <string.h>

#include <hexo/types.h>
#include <hexo/device.h>
#include <hexo/driver.h>
#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/interrupt.h>

static CPU_LOCAL struct icu_soclib_private_s *icu_soclib_pv;

DEVICU_ENABLE(icu_soclib_enable)
{
  if (enable)
    cpu_mem_write_32(dev->addr[0] + ICU_SOCLIB_REG_IER_SET, 1 << irq);
  else
    cpu_mem_write_32(dev->addr[0] + ICU_SOCLIB_REG_IER_CLR, 1 << irq);
}

DEVICU_SETHNDL(icu_soclib_sethndl)
{
  struct icu_soclib_private_s	*pv = dev->drv_pv;
  struct icu_soclib_handler_s	*h = pv->table + irq;

  h->hndl = hndl;
  h->data = data;

  return 0;
}

DEVICU_DELHNDL(icu_soclib_delhndl)
{
  /* FIXME */
  return 0;
}

static CPU_INTERRUPT_HANDLER(icu_soclib_cpu_handler)
{
  struct icu_soclib_private_s	*pv = CPU_LOCAL_GET(icu_soclib_pv);
  struct icu_soclib_handler_s	*h;

  h = pv->table + cpu_mem_read_32(pv->dev->addr[0] + ICU_SOCLIB_REG_VECTOR);

  /* call interrupt handler */
  h->hndl(h->data);
}

DEV_CLEANUP(icu_soclib_cleanup)
{
  struct icu_soclib_private_s	*pv = dev->drv_pv;

  mem_free(pv);
}

#ifndef CONFIG_STATIC_DRIVERS
const struct driver_s	icu_soclib_drv =
{
  .f_init		= icu_soclib_init,
  .f_cleanup		= icu_soclib_cleanup,
  .f.icu = {
    .f_enable		= icu_soclib_enable,
    .f_sethndl		= icu_soclib_sethndl,
    .f_delhndl		= icu_soclib_delhndl,
  }
};

#endif

DEV_INIT(icu_soclib_init)
{
  struct icu_soclib_private_s	*pv;

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &icu_soclib_drv;
#endif

  if ((pv = mem_alloc(sizeof (*pv), MEM_SCOPE_SYS))) /* FIXME allocation scope ? */
    {
      CPU_LOCAL_SET(icu_soclib_pv, pv);
      dev->drv_pv = pv;
      pv->dev = dev;

      cpu_mem_write_32(dev->addr[0] + ICU_SOCLIB_REG_IER_CLR, -1);

      cpu_interrupt_hw_sethandler(icu_soclib_cpu_handler);

      return 0;
    }

  return -ENOMEM;
}

