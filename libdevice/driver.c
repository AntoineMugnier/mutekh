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

    Copyright (c) 2009, Nicolas Pouillon, <nipo@ssji.net>
    Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2011 Institut Telecom / Telecom ParisTech
*/

#include <hexo/error.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/enum.h>

#include <assert.h>
#include <string.h>
#include <stdlib.h>

error_t device_get_accessor(void *accessor, struct device_s *dev,
                            enum driver_class_e cl, uint_fast8_t number)
{
  struct device_accessor_s *a = accessor;
  const struct driver_class_s *c;
  uint_fast8_t i;
  error_t err = -ENOTSUP;

  if (dev->status != DEVICE_DRIVER_INIT_DONE)
    return -EAGAIN;
  assert(dev->drv != NULL);

  for (i = 0; (c = dev->drv->classes[i]) != NULL; i++)
    {
      if (c->class_ == cl)
        {
          a->dev = dev;
          a->api = c;
          a->number = number;
          err = 0;
          if (dev->drv->f_use == NULL ||
            !(err = dev->drv->f_use(accessor, DEV_USE_GET_ACCESSOR)))
            dev->ref_count++;
          else
            a->dev = NULL;
          return err;
        }
    }

  return -ENOTSUP;
}

void device_put_accessor(void *accessor)
{
  struct device_accessor_s *a = accessor;
  struct device_s *dev = a->dev;

  assert(dev && dev->ref_count);

  dev->ref_count--;
  if (dev->drv->f_use != NULL)
    dev->drv->f_use(accessor, DEV_USE_PUT_ACCESSOR);
  a->dev = NULL;
  a->api = NULL;
}

error_t device_get_accessor_by_path(void *accessor, struct device_node_s *root,
                                    const char *path, enum driver_class_e cl)
{
  const char *num;

  error_t e = device_node_from_path(&root, path, 5, &num, &device_filter_init_done);
  if (e)
    return e;

  return device_get_accessor(accessor, device_from_node(root), cl, num ? atoi(num) : 0);
}

#ifdef CONFIG_DEVICE_TREE

static bool_t device_find_driver_r(struct device_node_s *node)
{
  extern const struct driver_s * dev_drivers_table[];
  extern const struct driver_s * dev_drivers_table_end[];

  bool_t done = 0;

  struct device_s *dev = device_from_node(node);

  if (dev && !(node->flags & DEVICE_FLAG_IGNORE))
    {
      switch (dev->status)
        {
        case DEVICE_NO_DRIVER: {
          const struct driver_s **drv = dev_drivers_table;

          struct device_enum_s e;

          /* get associated enumerator device */
          if (!dev->enum_dev)
            break;
          if (device_get_accessor(&e, dev->enum_dev, DRIVER_CLASS_ENUM, 0))
            break;

          /* iterate over available drivers */
          for ( ; drv < dev_drivers_table_end ; drv++ )
            {
              /* driver entry may be NULL on heterogeneous systems */
              if (!*drv)
                continue;

              /* use enumerator to decide if driver is appropriate for this device */
              if (DEVICE_OP(&e, match_driver, *drv, dev))
                {
                  device_bind_driver(dev, *drv);
                  done = 1;
                  break;
                }
            }

          device_put_accessor(&e);

          if (dev->status != DEVICE_DRIVER_INIT_PENDING)
            break;
        }

          /* try to intialize device using associated driver */
        case DEVICE_DRIVER_INIT_PENDING: 

          if (device_init_driver(dev) != -EAGAIN)
            if (dev->status != DEVICE_DRIVER_INIT_PENDING)
              done = 1;

        default:
          break;
        }
    }

  DEVICE_NODE_FOREACH(node, child, {
      done |= device_find_driver_r(child);
  });

  return done;
}

void device_find_driver(struct device_node_s *node)
{
  if (!node)
    node = device_tree_root();

  /* try to bind and init as many devices as possible */
  while (device_find_driver_r(node))
    ;
}

#endif

error_t device_bind_driver(struct device_s *dev, const struct driver_s *drv)
{
  if (dev->status != DEVICE_NO_DRIVER)
    return -EBUSY;

  dev->drv = drv;
  dev->status = DEVICE_DRIVER_INIT_PENDING;

  return 0;
}

error_t device_init_driver(struct device_s *dev)
{
  const struct driver_s *drv = dev->drv;

  if (dev->status != DEVICE_DRIVER_INIT_PENDING)
    return -EBUSY;

  /* check dependencies before trying device initialization */
  DEVICE_RES_FOREACH(dev, r, {
      if (r->flags & DEVICE_RES_FLAGS_DEPEND)
        {
          struct device_s *dep = dev;
          if (device_get_by_path(&dep, (const char*)r->u.uint[0], &device_filter_init_done))
            return -EAGAIN;
        }
  });

  printk("device: initialization of device %p `%s' using driver %p `%s'\n",
         dev, dev->node.name, drv, drv->desc);

  /* device init */
  error_t err = drv->f_init(dev);

  if (err)
    printk("device: device %p `%s' initialization failed with return code %i\n",
           dev, dev->node.name, err);

  return err;
}

error_t dev_driver_notsup_fcn()
{
  return -ENOTSUP;
}

