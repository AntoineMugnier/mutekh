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

#include <hexo/device.h>
#include <device/driver.h>
#include <hexo/error.h>
#include <hexo/alloc.h>

#ifdef CONFIG_HEXO_DEVICE_TREE

CONTAINER_FUNC(device_list, CLIST, inline, device_list);

OBJECT_CONSTRUCTOR(device_obj)
{
  device_list_init(&obj->children);
  lock_init(&obj->lock);

  return 0;
}

OBJECT_DESTRUCTOR(device_obj)
{
  dev_cleanup(obj);
  device_list_destroy(&obj->children);
  lock_destroy(&obj->lock);
}

error_t
device_register(struct device_s *dev,
		struct device_s *parent,
		void *enum_pv)
{
  dev->parent = device_obj_refnew(parent);
  dev->enum_pv = enum_pv;

  device_list_pushback(&parent->children, dev);

  return 0;
}

error_t device_unregister(struct device_s *dev)
{
  return -1; 			/* FIXME */
}

void
device_dump_list(struct device_s *root)
{
  CONTAINER_FOREACH(device_list, CLIST, &root->children,
  {
    printk("device %p\n", item);
  });
}

#endif

void
device_init(struct device_s *dev)
{
#ifdef CONFIG_HEXO_DEVICE_TREE
  va_list ap;
  device_obj_construct(dev, NULL, ap);
#endif
  lock_init(&dev->lock);
}

struct device_s *device_get_child(struct device_s *dev, uint_fast8_t i)
{
  struct device_s *res = NULL;

  CONTAINER_FOREACH(device_list, CLIST, &dev->children,
  {
    if (i-- == 0)
      {
	res = item;
	break;
      }
  });

  return res;
}

