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
#include <hexo/device.h>
#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include "enum-pci.h"
#include "enum-pci-private.h"

/**************************************************************/

#if 0
DEVENUM_FIND(enum_pci_find)
{
  struct enum_pci_context_s	*pv = dev->drv_pv;
  struct enum_id_pci_s		*ident = id;
  /*
  CONTAINER_FOREACH(device_list, DLIST, device_list, &dev->children,
  {
    if (ident->vendor == ENUM_ID_PCI_WILDCARD)
      ;
  });
  */
  return NULL;
}

#endif

/*
 * device close operation
 */

DEV_CLEANUP(enum_pci_cleanup)
{
  struct enum_pci_context_s	*pv = dev->drv_pv;

  //  lock_destroy(&pv->lock);

  mem_free(pv);
}

static error_t
pci_enum_dev_probe(struct device_s *dev, uint8_t bus,
		   uint8_t dv, uint8_t fn)
{
  error_t			res = -ENOMEM;
  struct device_s		*new;
  struct enum_pv_pci_s		*enum_pv;
  uint16_t			vendor;
  uint8_t			htype;

  vendor = pci_confreg_read(bus, dv, fn, PCI_CONFREG_VENDOR);

  if (!vendor || vendor == 0xffff)
    return -ENOENT;

  if ((new = device_obj_new(0)))
    {
      if ((enum_pv = mem_alloc(sizeof (*enum_pv), MEM_SCOPE_SYS)))
	{
	  enum_pv->vendor = vendor;
	  enum_pv->devid = pci_confreg_read(bus, dv, fn, PCI_CONFREG_DEVID);
	  enum_pv->class = pci_confreg_read(bus, dv, fn, PCI_CONFREG_CLASS);

	  printf("PCI device %04x:%04x class %x06x\n",
		 vendor, enum_pv->devid, enum_pv->class);

	  device_register(new, dev, enum_pv);

	  htype = pci_confreg_read(bus, dv, fn, PCI_CONFREG_HTYPE);

	  res = (htype & PCI_CONFREG_HTYPE_MULTI ? 1 : 0);
	}

      device_obj_refdrop(new);
    }

  return res;
}

static error_t
pci_enum_probe(struct device_s *dev)
{
  uint_fast8_t	bus, dv, fn;

  for (bus = 0; bus < PCI_CONF_MAXBUS; bus++)
    for (dv = 0; dv < PCI_CONF_MAXDEVICE; dv++)
      for (fn = 0; fn < PCI_CONF_MAXFCN; fn++)
	if (pci_enum_dev_probe(dev, bus, dv, fn) <= 0)
	  break;

  return 0;
}


/*
 * device open operation
 */

#ifndef CONFIG_STATIC_DRIVERS
const struct driver_s	enum_pci_drv =
{
  .f_init		= enum_pci_init,
  .f_cleanup		= enum_pci_cleanup,
  .f.denum = {
  }
};
#endif

DEV_INIT(enum_pci_init)
{
  struct enum_pci_context_s	*pv;

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &enum_pci_drv;
#endif

  /* allocate private driver data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  lock_init(&pv->lock);

  dev->drv_pv = pv;

  pci_enum_probe(dev);

  return 0;
}

