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

#ifdef CONFIG_DEVICE_HIERARCHY

static struct device_s dev_root = { } ;

CONTAINER_FUNC(static inline, device, DLIST, device_list, SPIN, siblings);

error_t
device_init(struct device_s *dev)
{
  device_list_init(&dev->children);

  return 0;
}

error_t
device_register(struct device_s *dev,
		struct device_s *parent,
		void *enum_pv)
{
  if (!parent)
    parent = &dev_root;

  dev->parent = parent;
  dev->enum_pv = enum_pv;

  device_list_push(&parent->children, dev);

  return 0;
}

void
device_dump_list(struct device_s *root)
{
  struct device_s	*d;

  if (!root)
    root = &dev_root;

  for (d = device_list_head(&root->children); d;
       d = device_list_next(&root->children, d))
    {
      printf("device %p\n", d);
    }
}

#endif

