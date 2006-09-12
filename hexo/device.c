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
#include <hexo/error.h>
#include <hexo/alloc.h>

#ifdef CONFIG_DEVICE_HIERARCHY

CONTAINER_OBJECT_FUNC(static inline, device_list, DLIST, device_list, HEXO_SPIN, device_obj, list_entry);

OBJECT_CONSTRUCTOR(device_obj)
{
  struct device_s	*obj;

  if ((obj = mem_alloc(sizeof (*obj), MEM_SCOPE_SYS)))
    {
      device_obj_init(obj);
      device_list_init(&obj->children);
    }

  return obj;
}

OBJECT_DESTRUCTOR(device_obj)
{
  mem_free(obj);
}

error_t
device_register(struct device_s *dev,
		struct device_s *parent,
		void *enum_pv)
{
  dev->parent = device_obj_refnew(parent);
  dev->enum_pv = enum_pv;

  device_list_push(&parent->children, dev);

  return 0;
}

void
device_dump_list(struct device_s *root)
{
  CONTAINER_FOREACH(device_list, DLIST, device_list, &root->children,
  {
    printf("device %p\n", item);
  });
}

void
device_init(struct device_s *dev)
{
  device_list_init(&dev->children);
  device_obj_init(dev);
}

#endif

