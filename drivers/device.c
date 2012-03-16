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

#include <drivers/enum/root/enum-root.h>

#include <device/device.h>
#include <device/driver.h>
#include <hexo/error.h>
#include <mutek/mem_alloc.h>

#include <mutek/printk.h>

#ifdef CONFIG_DEVICE_TREE

# ifdef CONFIG_DRIVER_ENUM_ROOT
struct device_s enum_root;
# endif

void device_tree_init()
{
# ifdef CONFIG_DRIVER_ENUM_ROOT
	device_init(&enum_root);
	enum_root_init(&enum_root, NULL);
# endif
}

CONTAINER_FUNC(device_list, CLIST, inline, device_list);

void device_init(struct device_s *dev)
{
  dev->res_count = DEVICE_STATIC_RESOURCE_COUNT;

  device_list_init(&dev->children);
  lock_init(&dev->lock);
  dev->parent = NULL;
  dev->enum_type = DEVENUM_TYPE_INVALID;
  dev->allocated = 0;

  memset(dev->res, 0, sizeof(dev->res));
}

struct device_s *device_alloc(size_t resources)
{
  struct device_s *dev = mem_alloc(sizeof(struct device_s) + sizeof(struct dev_resource_s)
                                   * ((ssize_t)resources - DEVICE_STATIC_RESOURCE_COUNT), (mem_scope_sys));

  dev->res_count = resources;
  dev->enum_type = DEVENUM_TYPE_INVALID;
  dev->allocated = 1;

  return dev;
}

void device_cleanup(struct device_s *dev)
{
  assert(!dev->parent);
  assert(!dev->ref_count);

  if (dev->drv)
    dev->drv->f_cleanup(dev);

  device_list_destroy(&dev->children);
  lock_destroy(&dev->lock);

  if (dev->allocated)
    mem_free(dev);
}

void device_attach(struct device_s *dev,
                   struct device_s *parent)
{
  assert(!dev->parent);

  if (!parent)
    parent = &enum_root;

  dev->parent = parent;

  device_list_pushback(&parent->children, dev);
}

void device_detach(struct device_s *dev)
{
  assert(dev->parent);

  device_list_remove(&dev->parent->children, dev);
  dev->parent = 0;
}

void
device_dump_r(struct device_s *dev, uint_fast8_t indent)
{
  uint_fast8_t i, j;

  printk("\n");
  for (i = 0; i < indent; i++)
    printk("  ");
  printk("Device %p, status: %i, use: %i\n", dev, dev->status, dev->ref_count);

  if (dev->icu)
    {
      for (i = 0; i < indent; i++)
        printk("  ");
      printk("  Interrupts controller: %p", dev->icu);
    }

  if (dev->drv)
    {
      for (i = 0; i < indent; i++)
        printk("  ");
      printk("  Driver: %p \"%s\"\n", dev->drv, dev->drv->desc);
    }

  for (j = 0; j < dev->res_count; j++)
    {
      for (i = 0; i < indent; i++)
        printk("  ");
      switch (dev->res[j].type)
        {
        case DEV_RES_UNUSED:
          continue;
        case DEV_RES_MEM:
          printk("  %i: Memory range from %p to %p\n", j, dev->res[j].mem.start, dev->res[j].mem.end);
          break;
        case DEV_RES_IO:
          printk("  %i: I/O range from %p to %p\n", j, dev->res[j].io.start, dev->res[j].io.end);
          break;
        case DEV_RES_IRQ:
          printk("  %i: IRQ number %i, source end-point %p\n", j, dev->res[j].irq.id, dev->res[j].irq.ep);
          break;
        case DEV_RES_ID:
          printk("  %i: ID %x %x\n", j, dev->res[j].id.major, dev->res[j].id.minor);
          break;
        case DEV_RES_VENDORID:
          printk("  %i: Vendor %x \"%s\"\n", j, dev->res[j].vendorid.id, dev->res[j].vendorid.name);
          break;
        case DEV_RES_DEVICEID:
          printk("  %i: Device %x \"%s\"\n", j, dev->res[j].vendorid.id, dev->res[j].vendorid.name);
          break;
        default:
          printk("  %i: unknown resource type %i\n", j, dev->res[j].type);
        }
    }
}

void
device_dump(struct device_s *dev)
{
  device_dump_r(dev, 0);
}

static void
device_dump_tree_r(struct device_s *root, uint_fast8_t i)
{
  device_dump_r(root, i);

  CONTAINER_FOREACH(device_list, CLIST, &root->children,
  {
    device_dump_tree_r(item, i+1);
  });
}

void
device_dump_tree(struct device_s *root)
{
  if (!root)
    root = &enum_root;

  device_dump_tree_r(&enum_root, 0);
}

#endif

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


error_t device_res_id(const struct device_s *dev,
                      enum dev_resource_type_e type,
                      uint_fast8_t id, uint_fast8_t *res)
{
  uint_fast8_t i;

  for (i = 0; i < dev->res_count; i++)
    if (dev->res[i].type == type && !id--)
      {
        *res = i;
        return 0;
      }

  return -ENOENT;
}

error_t device_res_get_uint(const struct device_s *dev,
                            enum dev_resource_type_e type,
                            uint_fast8_t id, uintptr_t *res)
{
  uint_fast8_t i;

  for (i = 0; i < dev->res_count; i++)
    if (dev->res[i].type == type && !id--)
      {
        *res = dev->res[i].uint;
        return 0;
      }

  return -ENOENT;
}

struct dev_resource_s * device_res_add(struct device_s *dev)
{
  uint_fast8_t i;

  for (i = 0; i < dev->res_count; i++)
    {
      struct dev_resource_s *r = dev->res + i;

      if (r->type == DEV_RES_UNUSED)
        return r;
    }

  return NULL;
}

error_t device_res_add_io(struct device_s *dev, uintptr_t start, uintptr_t end)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_IO;
  r->io.start = start;
  r->io.end = end;

  return 0;
}

error_t device_res_add_mem(struct device_s *dev, uintptr_t start, uintptr_t end)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_MEM;
  r->mem.start = start;
  r->mem.end = end;

  return 0;
}

error_t device_res_add_irq(struct device_s *dev, uintptr_t irq)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_IRQ;
  r->irq.id = irq;

  return 0;
}

struct device_accessor_s {
  struct device_s *dev;
  const void *api;
  uint_fast8_t number;
};

struct driver_class_s {
  enum device_class_e class_;
  void *functions[];
};

error_t device_get_accessor(void *accessor, struct device_s *dev,
                            enum device_class_e cl, uint_fast8_t number)
{
  struct device_accessor_s *a = accessor;
  const struct driver_class_s *c;
  uint_fast8_t i, n = number;

  if (dev->drv == NULL)
    return -ENOENT;

  for (i = 0; (c = dev->drv->classes[i]) != NULL; i++)
    {
      if (c->class_ == cl && !n--)
        {
          a->dev = dev;
          a->api = c;
          a->number = number;
          dev->ref_count++;
          return 0;
        }
    }

  return -ENOENT;
}

void device_put_accessor(void *accessor)
{
  struct device_accessor_s *a = accessor;

  assert(a->dev && a->dev->ref_count);

  a->dev->ref_count--;
  a->dev = NULL;
  a->api = NULL;
}

