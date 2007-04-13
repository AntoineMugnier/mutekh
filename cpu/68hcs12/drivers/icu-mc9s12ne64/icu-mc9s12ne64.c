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

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#include <hexo/types.h>

#include <hexo/device/icu.h>
#include <hexo/device.h>
#include <hexo/driver.h>

#include <hexo/alloc.h>
#include <hexo/interrupt.h>
#include <hexo/error.h>

#include "icu-mc9s12ne64.h"

#include "icu-mc9s12ne64-private.h"

static CPU_LOCAL struct icu_mc9s12ne64_private_s *icu_mc9s12ne64_pv;

DEVICU_ENABLE(icu_mc9s12ne64_enable)
{
  /* nothing to do */
}

DEVICU_SETHNDL(icu_mc9s12ne64_sethndl)
{
  struct icu_mc9s12ne64_private_s	*pv = dev->drv_pv;
  struct icu_mc9s12ne64_handler_s	*h = pv->table + irq;

  h->hndl = hndl;
  h->data = data;

  return 0;
}

DEVICU_DELHNDL(icu_mc9s12ne64_delhndl)
{
  return 0;
}

static CPU_INTERRUPT_HANDLER(icu_mc9s12ne64_cpu_handler)
{
  struct icu_mc9s12ne64_private_s	*pv = CPU_LOCAL_GET(icu_mc9s12ne64_pv);
  struct icu_mc9s12ne64_handler_s	*h = pv->table + irq;

  /* call interrupt handler */
  h->hndl(h->data);
}

DEV_CLEANUP(icu_mc9s12ne64_cleanup)
{
  struct icu_mc9s12ne64_private_s	*pv = dev->drv_pv;

  mem_free(pv);
}

#ifndef CONFIG_STATIC_DRIVERS
const struct driver_s	icu_mc9s12ne64_drv =
{
  .f_init		= icu_mc9s12ne64_init,
  .f_cleanup		= icu_mc9s12ne64_cleanup,
  .f.icu = {
    .f_enable		= icu_mc9s12ne64_enable,
    .f_sethndl		= icu_mc9s12ne64_sethndl,
    .f_delhndl		= icu_mc9s12ne64_delhndl,
  }
};
#endif

DEV_INIT(icu_mc9s12ne64_init)
{
  struct icu_mc9s12ne64_private_s	*pv;

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &icu_mc9s12ne64_drv;
#endif

  if ((pv = mem_alloc(sizeof (*pv), MEM_SCOPE_SYS)))
    {
      CPU_LOCAL_SET(icu_mc9s12ne64_pv, pv);
      dev->drv_pv = pv;
      pv->dev = dev;

      cpu_interrupt_sethandler(icu_mc9s12ne64_cpu_handler);

      return 0;
    }

  return -ENOMEM;
}

