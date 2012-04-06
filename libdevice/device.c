/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <hexo/error.h>

#include <stdio.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/enum.h>

#include <mutek/mem_alloc.h>

#include <mutek/printk.h>

#include <drivers/enum/root/enum-root.h>

# ifdef CONFIG_DRIVER_ENUM_ROOT
struct device_s enum_root;
# endif

void device_tree_init()
{
# ifdef CONFIG_DRIVER_ENUM_ROOT
  device_init(&enum_root);
  enum_root_init(&enum_root);
  enum_root.name = "root";
# endif
}

CONTAINER_FUNC(device_list, CLIST, inline, device_list);

void device_init(struct device_s *dev)
{
  lock_init(&dev->lock);
  dev->status = DEVICE_NO_DRIVER;
  dev->drv = NULL;
  dev->res_count = DEVICE_STATIC_RESOURCE_COUNT;
  dev->ref_count = 0;
  dev->allocated = 0;

#ifdef CONFIG_DEVICE_TREE
  dev->name = NULL;
  dev->enum_dev = &enum_root;
  dev->parent = NULL;
  device_list_init(&dev->children);
#endif

  memset(dev->res, 0, sizeof(dev->res));
}

struct device_s *device_alloc(size_t resources)
{
  struct device_s *dev = mem_alloc(sizeof(struct device_s) + sizeof(struct dev_resource_s)
                                   * ((ssize_t)resources - DEVICE_STATIC_RESOURCE_COUNT), (mem_scope_sys));

  if (dev != NULL)
    {
      lock_init(&dev->lock);
      dev->status = DEVICE_NO_DRIVER;
      dev->drv = NULL;
      dev->res_count = resources;
      dev->ref_count = 0;
      dev->allocated = 1;

#ifdef CONFIG_DEVICE_TREE
      dev->name = NULL;
      dev->enum_dev = &enum_root;
      dev->parent = NULL;
      device_list_init(&dev->children);
#endif

      memset(dev->res, 0, sizeof(struct dev_resource_s) * resources);
    }

  return dev;
}

void device_cleanup(struct device_s *dev)
{
  uint_fast8_t i;

  assert(!dev->parent);
  assert(!dev->ref_count);

  if (dev->status == DEVICE_DRIVER_INIT_DONE)
    dev->drv->f_cleanup(dev);

  for (i = 0; i < dev->res_count; i++)
    {
      struct dev_resource_s *r = dev->res + i;
      switch (r->type)
        {
#ifdef CONFIG_HEXO_IRQ
        case DEV_RES_IRQ:
          r->irq.icu--;
          break;
#endif
        case DEV_RES_VENDORID:
          if (r->vendor.name && dev->allocated)
            mem_free((void*)r->vendor.name);
          break;

        case DEV_RES_PRODUCTID:
          if (r->product.name && dev->allocated)
            mem_free((void*)r->product.name);
          break;

        case DEV_RES_STR_PARAM:
        case DEV_RES_UINT_ARRAY_PARAM:
          if (dev->allocated)
            mem_free((void*)r->str_param.value);
        case DEV_RES_UINT_PARAM:
          if (dev->allocated)
            mem_free((void*)r->str_param.name);
          break;

        default:
          break;
        }
    }

  device_list_destroy(&dev->children);
  lock_destroy(&dev->lock);

  if (dev->allocated)
    {
      if (dev->name)
        mem_free((void*)dev->name);

      mem_free(dev);
    }
}

void device_shrink(struct device_s *dev)
{
  uint_fast8_t i;

  for (i = dev->res_count; i > 0; i--)
    if (dev->res[i-1].type != DEV_RES_UNUSED)
      break;

  if (i < dev->res_count)
    {
      dev->res_count = i;

      mem_resize(dev, sizeof(struct device_s) + sizeof(struct dev_resource_s)
                 * ((ssize_t)i - DEVICE_STATIC_RESOURCE_COUNT));
    }
}

void device_attach(struct device_s *dev,
                   struct device_s *parent)
{
  static uint_fast16_t id;
  char name [16];

  assert(!dev->parent);

  if (!parent)
    parent = &enum_root;

  dev->parent = parent;

  if (!dev->name)
    {
      sprintf(name, "dev%u", id++);
      dev->name = strdup(name);
    }

  device_list_pushback(&parent->children, dev);
}

void device_detach(struct device_s *dev)
{
  assert(dev->parent);

  device_list_remove(&dev->parent->children, dev);
  dev->parent = 0;
}

struct device_s *device_get_child(struct device_s *dev, uint_fast8_t i)
{
  struct device_s *res = NULL;

#ifdef CONFIG_DEVICE_TREE
  CONTAINER_FOREACH(device_list, CLIST, &dev->children,
  {
    if (i-- == 0)
      {
	res = item;
	break;
      }
  });
#endif

  return res;
}

#if defined(CONFIG_DEVICE_TREE) && defined (CONFIG_DRIVER_ENUM_ROOT)
static void _device_tree_walk(struct device_s *dev, device_tree_walker_t *walker, void *priv)
{
    walker(dev, priv);
    CONTAINER_FOREACH(device_list, CLIST, &dev->children,
    {
        _device_tree_walk(item, walker, priv);
    });
}

void device_tree_walk(device_tree_walker_t *walker, void *priv)
{
    _device_tree_walk(&enum_root, walker, priv);
}
#endif

