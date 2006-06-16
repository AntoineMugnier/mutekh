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
#include <mutek/alloc.h>
#include <mutek/lock.h>
#include <mutek/interrupt.h>

//#include "enum-pci.h"
#include "enum-pci-private.h"

/**************************************************************/

DEVENUM_FIND(enum_pci_find)
{
  struct enum_pci_context_s	*pv = dev->drv_pv;

  return NULL;
}

/*
 * device close operation
 */

DEV_CLEANUP(enum_pci_cleanup)
{
  struct enum_pci_context_s	*pv = dev->drv_pv;

  //  lock_destroy(&pv->lock);

  mem_free(pv);
}

/*
 * device open operation
 */

DEV_INIT(enum_pci_init)
{
  struct enum_pci_context_s	*pv;

#ifndef CONFIG_STATIC_DRIVERS
  dev->f_cleanup	= enum_pci_cleanup;
  dev->f_irq		= 0;
  dev->denum.f_find	= enum_pci_find;
#endif

  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  lock_init(&pv->lock);

  dev->drv_pv = pv;

  return 0;
}

