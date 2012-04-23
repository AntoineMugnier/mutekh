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
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/enum.h>

#include <assert.h>
#include <string.h>

# ifdef CONFIG_DRIVER_ENUM_ROOT
struct device_s device_enum_root;
# endif

error_t device_get_accessor(void *accessor, struct device_s *dev,
                            enum driver_class_e cl, uint_fast8_t number)
{
  struct device_accessor_s *a = accessor;
  const struct driver_class_s *c;
  uint_fast8_t i, n = number;

  if (dev->status != DEVICE_DRIVER_INIT_DONE)
    return -EAGAIN;
  assert(dev->drv != NULL);

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

  return -ENOTSUP;
}

void device_put_accessor(void *accessor)
{
  struct device_accessor_s *a = accessor;

  assert(a->dev && a->dev->ref_count);

  a->dev->ref_count--;
  a->dev = NULL;
  a->api = NULL;
}

#ifdef CONFIG_DEVICE_TREE

static bool_t device_find_driver_r(struct device_s *dev)
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
      if (device_get_accessor(&e, dev->enum_dev, DRIVER_CLASS_ENUM, 0))
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

  CONTAINER_FOREACH_NOLOCK(device_list, CLIST, &dev->children, {
      done |= device_find_driver_r(item);
  });

  return done;
}

extern struct device_s device_enum_root;

void device_find_driver(struct device_s *dev)
{
  if (!dev)
    dev = &device_enum_root;

  /* try to bind and init as many devices as possible */
  while (device_find_driver_r(dev))
    ;
}

#endif

error_t device_bind_driver(struct device_s *dev, const struct driver_s *drv)
{
  if (dev->status != DEVICE_NO_DRIVER)
    return -EBUSY;

  dev->drv = drv;
  dev->status = DEVICE_DRIVER_INIT_PENDING;      

  printk("device: %p `%s' uses %p `%s' driver\n",
         dev, dev->name, drv, drv->desc);

  return 0;
}

error_t device_init_driver(struct device_s *dev)
{
  const struct driver_s *drv = dev->drv;

  if (dev->status != DEVICE_DRIVER_INIT_PENDING)
    return -EBUSY;

  uint_fast8_t i;

  /* check dependencies before try device initialization */
  for (i = 0; i < dev->res_count; i++)
    {
      struct dev_resource_s *r = dev->res + i;
      switch (r->type)
        {
#ifdef CONFIG_DEVICE_IRQ
          /* check that interrupt controllers are initialized */
        case DEV_RES_IRQ: {
          struct device_s *icu = r->irq.icu;
          if (!icu)
            icu = device_get_default_icu(dev);
          if (!icu || icu->status != DEVICE_DRIVER_INIT_DONE)
            return -EAGAIN;
        }
#endif
        default:
          break;
        }
    }

  /* device init */
  error_t err = drv->f_init(dev);

  if (err)
    printk("device: device %p `%s' initialization failed with return code %i\n",
           dev, dev->name, err);

  return err;
}

