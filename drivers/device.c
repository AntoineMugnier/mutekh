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

#include <stdio.h>
#include <device/device.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/enum.h>

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
  enum_root.name = "root";
# endif
}

CONTAINER_FUNC(device_list, CLIST, inline, device_list);

void device_init(struct device_s *dev)
{
  device_list_init(&dev->children);
  lock_init(&dev->lock);
  dev->parent = NULL;
  dev->res_count = DEVICE_STATIC_RESOURCE_COUNT;
  dev->allocated = 0;
  dev->status = DEVICE_NO_DRIVER;
  dev->name = NULL;
  dev->enum_dev = &enum_root;

  memset(dev->res, 0, sizeof(dev->res));
}

struct device_s *device_alloc(size_t resources)
{
  struct device_s *dev = mem_alloc(sizeof(struct device_s) + sizeof(struct dev_resource_s)
                                   * ((ssize_t)resources - DEVICE_STATIC_RESOURCE_COUNT), (mem_scope_sys));

  device_list_init(&dev->children);
  lock_init(&dev->lock);
  dev->parent = NULL;
  dev->res_count = resources;
  dev->allocated = 1;
  dev->status = DEVICE_NO_DRIVER;
  dev->name = NULL;
  dev->enum_dev = &enum_root;

  return dev;
}

void device_cleanup(struct device_s *dev)
{
  uint_fast8_t i;

  assert(!dev->parent);
  assert(!dev->ref_count);

  if (dev->drv)
    dev->drv->f_cleanup(dev);

  for (i = 0; i < dev->res_count; i++)
    {
      struct dev_resource_s *r = dev->res + i;
      switch (r->type)
        {
#ifdef CONFIG_HEXO_IRQ
        case DEV_RES_IRQ:
          r->irq.icu--;
#endif
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

void
device_dump_r(struct device_s *dev, uint_fast8_t indent)
{
  uint_fast8_t i, j;
  const char *status[] = { DEVICE_STATUS_NAMES };

  printk("\n");
  for (i = 0; i < indent; i++)
    printk("  ");
  printk("Device %p `%s'\n", dev, dev->name);
  for (i = 0; i < indent; i++)
    printk("  ");
  printk("  status: %s, use: %i\n", status[dev->status], dev->ref_count);

  if (dev->drv)
    {
      for (i = 0; i < indent; i++)
        printk("  ");
      printk("  Driver: %p `%s'\n", dev->drv, dev->drv->desc);
    }

  uint_fast8_t count[DEV_RES_TYPES_COUNT] = { 0 };

  for (j = 0; j < dev->res_count; j++)
    {
      struct dev_resource_s *r = dev->res + j;

      uint16_t type = r->type;
      uint_fast8_t c = 0;

      if (type < DEV_RES_TYPES_COUNT)
        c = count[type]++;

      if (type == DEV_RES_UNUSED)
          continue;
      for (i = 0; i < indent; i++)
        printk("  ");
      switch (type)
        {
        case DEV_RES_MEM:
          printk("  Memory range %i from %p to %p\n", c, r->mem.start, r->mem.end);
          break;
        case DEV_RES_IO:
          printk("  I/O range %i from %p to %p\n", c, r->io.start, r->io.end);
          break;
#ifdef CONFIG_HEXO_IRQ
        case DEV_RES_IRQ:
          printk("  IRQ output %i bound to input %i of controller %p `%s'\n",
                 r->irq.dev_out_id, r->irq.icu_in_id, r->irq.icu, r->irq.icu->name);
          break;
#endif
        case DEV_RES_ID:
          printk("  Numerical identifier %x %x\n", r->id.major, r->id.minor);
          break;
        case DEV_RES_VENDORID:
          printk("  Vendor ID %x `%s'\n", r->vendor.id, r->vendor.name);
          break;
        case DEV_RES_PRODUCTID:
          printk("  Product ID %x `%s'\n", r->product.id, r->product.name);
          break;
        default:
          printk("  %i: unknown resource type %i\n", j, r->type);
        case DEV_RES_UNUSED:
          ;
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

struct dev_resource_s *device_res_get(struct device_s *dev,
                                      enum dev_resource_type_e type,
                                      uint_fast8_t id)
{
  uint_fast8_t i;

  for (i = 0; i < dev->res_count; i++)
    if (dev->res[i].type == type && !id--)
      return dev->res + i;

  return NULL;
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

error_t device_res_add_irq(struct device_s *dev, uint_fast16_t dev_out_id,
                           uint_fast16_t icu_in_id, struct device_s *icu)
{
#ifdef CONFIG_HEXO_IRQ
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_IRQ;
  r->irq.dev_out_id = dev_out_id;
  r->irq.icu_in_id = icu_in_id;
  r->irq.icu = icu;

  icu->ref_count++;

  return 0;
#else
  return -EINVAL;
#endif
}

error_t device_res_add_id(struct device_s *dev, uintptr_t major, uintptr_t minor)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_ID;
  r->id.major = major;
  r->id.minor = minor;

  return 0;
}

error_t device_res_add_vendorid(struct device_s *dev, uintptr_t id, const char *name)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_VENDORID;
  r->vendor.id = id;
  r->vendor.name = name;

  return 0;
}

error_t device_res_add_productid(struct device_s *dev, uintptr_t id, const char *name)
{
  struct dev_resource_s *r = device_res_add(dev);

  if (!r)
    return -ENOMEM;

  r->type = DEV_RES_PRODUCTID;
  r->product.id = id;
  r->product.name = name;

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

static bool_t device_bind_driver_r(struct device_s *dev)
{
  extern const struct driver_s * global_driver_registry[];
  extern const struct driver_s * global_driver_registry_end[];

  bool_t done = 0;

  switch (dev->status)
    {
    case DEVICE_NO_DRIVER: {
      const struct driver_s **drv = global_driver_registry;

      struct device_enum_s e;

      /* get associated enumerator device */
      if (!dev->enum_dev)
        break;
      if (device_get_accessor(&e, dev->enum_dev, DEVICE_CLASS_ENUM, 0))
        break;

      /* iterate over available drivers */
      for ( ; drv < global_driver_registry_end ; drv++ )
        {
          /* driver entry may be NULL on heterogeneous systems */
          if (!*drv)
            continue;

          /* use enumerator to decide if driver is appropriate for this device */
          if (DEVICE_OP(&e, match_driver, *drv, dev))
            {
              dev->drv = *drv;
              dev->status = DEVICE_DRIVER_INIT_PENDING;
              printk("device: %p `%s' device bound to %p `%s' driver\n",
                     dev, dev->name, *drv, (*drv)->desc);
              done = 1;
              break;
            }
        }

      device_put_accessor(&e);

      if (dev->status != DEVICE_DRIVER_INIT_PENDING)
        break;
    }

      /* try to intialize device using associated driver */
    case DEVICE_DRIVER_INIT_PENDING: {
      const struct driver_s *drv = dev->drv;
      uint_fast8_t i;

      /** check dependencies before try device initialization */
      for (i = 0; i < dev->res_count; i++)
        {
          struct dev_resource_s *r = dev->res + i;
          switch (r->type)
            {
#ifdef CONFIG_HEXO_IRQ
              /** check that interrupt controllers are initialized */
            case DEV_RES_IRQ:
              if (r->irq.icu->status != DEVICE_DRIVER_INIT_DONE)
                goto skip;
#endif
            default:
              break;
            }
        }

      /** device init */
      error_t err = drv->f_init(dev, NULL);

      if (err)
        printk("device: device %p `%s' initialization failed with return code %i\n",
               dev, dev->name, err);

      if (dev->status != DEVICE_DRIVER_INIT_PENDING)
        done = 1;

      goto skip;
      skip:;
    }

    default:
      break;
    }

  CONTAINER_FOREACH_NOLOCK(device_list, CLIST, &dev->children, {
      done |= device_bind_driver_r(item);
  });

  return done;
}

void device_bind_driver(struct device_s *dev)
{
  if (!dev)
    dev = &enum_root;

  /* try to bind and init as many devices as possible */
  while (device_bind_driver_r(dev))
    ;
}

