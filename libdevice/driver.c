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

#include <mutek/startup.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/enum.h>
#include <enums.h>

#include <assert.h>
#include <string.h>
#include <stdlib.h>

const char driver_class_e[] = ENUM_DESC_DRIVER_CLASS_E;

static error_t device_get_api(struct device_s *dev,
                              enum driver_class_e cl,
                              const struct driver_class_s **api)
{
  const struct driver_s *drv = dev->drv;
  if (drv == NULL)
    return -EBUSY;

  const struct driver_class_s *c = NULL;
  uint_fast8_t i;
  for (i = 0; (c = drv->classes[i]) != NULL; i++)
    if (c->class_ == cl)
      goto found;
  return -ENOENT;

 found:
  switch (dev->status)
    {
    case DEVICE_ENUM_ERROR:
    case DEVICE_NO_DRIVER:
      UNREACHABLE();
    case DEVICE_DRIVER_INIT_PARTIAL:
      if (dev->init_mask & (1 << i))
        {
        case DEVICE_DRIVER_INIT_DONE:
          if (api)
            *api = c;
          return 0;
        }
      return -EAGAIN;
    default:
      return -EBUSY;
    }
}

error_t device_get_accessor(void *accessor, struct device_s *dev,
                            enum driver_class_e cl, uint_fast8_t number)
{
  struct device_accessor_s *a = accessor;
  error_t err;

  LOCK_SPIN_IRQ(&dev->lock);

  const struct driver_class_s *c;
  if (!(err = device_get_api(dev, cl, &c)))
    {
      a->dev = dev;
      a->api = c;
      a->number = number;
      if (dev->drv->f_use == NULL ||
          !(err = dev->drv->f_use(accessor, DEV_USE_GET_ACCESSOR)))
        dev->ref_count++;
      else
        a->dev = NULL;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

void device_put_accessor(void *accessor)
{
  struct device_accessor_s *a = accessor;
  struct device_s *dev = a->dev;

  assert(dev != NULL);

  LOCK_SPIN_IRQ(&dev->lock);

  assert(dev->ref_count);

  dev->ref_count--;
  if (dev->drv->f_use != NULL)
    dev->drv->f_use(accessor, DEV_USE_PUT_ACCESSOR);
  a->dev = NULL;
  a->api = NULL;

  LOCK_RELEASE_IRQ(&dev->lock);
}

static bool_t device_filter_accessor(struct device_node_s *node)
{
  struct device_s *dev = device_from_node(node);
  if (dev == NULL || dev->drv == NULL)
    return 0;
  if (dev->status == DEVICE_DRIVER_INIT_DONE)
    return 1;
  return !(dev->node.flags & DEVICE_FLAG_IGNORE);
}

error_t device_get_accessor_by_path(void *accessor, struct device_node_s *root,
                                    const char *path, enum driver_class_e cl)
{
  const char *num;

  error_t e = device_node_from_path(&root, path, 5, &num, &device_filter_accessor);
  if (e)
    return e;

  return device_get_accessor(accessor, device_from_node(root), cl, num ? atoi(num) : 0);
}

#ifdef CONFIG_DEVICE_TREE

static bool_t device_find_driver_r(struct device_node_s *node, uint_fast8_t pass)
{
  bool_t done = 1;

  struct device_s *dev = device_from_node(node);

  if (dev && !(node->flags & DEVICE_FLAG_IGNORE))
    {
      switch (dev->status)
        {
        case DEVICE_NO_DRIVER: {
          const struct driver_registry_s *reg;
          struct device_enum_s e;

          /* get associated enumerator device */
          if (!dev->enum_dev)
            break;
          if (device_get_accessor(&e, dev->enum_dev, DRIVER_CLASS_ENUM, 0))
            break;

          /* iterate over available drivers */
          for (reg = driver_registry_table;
               reg < driver_registry_table_end;
               reg++)
            {
              /* driver entry may be NULL on heterogeneous systems */
              if (!reg->driver || !reg->id_table || !reg->id_count)
                continue;

              /* use enumerator to decide if driver is appropriate for this device */
              if (DEVICE_OP(&e, match_driver, reg->id_table, reg->id_count, dev))
                {
                  device_bind_driver(dev, reg->driver);
                  done = 0;
                  break;
                }
            }

          device_put_accessor(&e);

          if (dev->status != DEVICE_DRIVER_INIT_PENDING)
            break;
        }

          /* try to intialize device using associated driver */
        case DEVICE_DRIVER_INIT_PENDING: 

          if (!pass && !(dev->drv->flags & DRIVER_FLAGS_EARLY_INIT))
            break;

        case DEVICE_DRIVER_INIT_PARTIAL:

          if (device_init_driver(dev) == 0)
            done = 0;

        default:
          break;
        }
    }

  DEVICE_NODE_FOREACH(node, child, {
      done &= device_find_driver_r(child, pass);
  });

  return done;
}

void device_find_driver(struct device_node_s *node, uint_fast8_t pass)
{
  if (!node)
    node = device_tree_root();

  /* try to bind and init as many devices as possible */
  while (!device_find_driver_r(node, pass))
    ;
}

#endif

static void libdevice_drivers_init(uint_fast8_t pass)
{
#ifdef CONFIG_DEVICE_TREE
  device_find_driver(NULL, pass);
#else
  bool_t done;
  struct device_s *dev;

  do {
    done = 1;
    DEVICE_NODE_FOREACH(, node, {
        if (node->flags & DEVICE_FLAG_IGNORE)
          continue;
        if (!(dev = device_from_node(node)))
          continue;

        switch (dev->status)
          {
          default:
            continue;
          case DEVICE_DRIVER_INIT_PENDING:
            if (!pass && !(dev->drv->flags & DRIVER_FLAGS_EARLY_INIT))
              continue;
          case DEVICE_DRIVER_INIT_PARTIAL:
            if (device_init_driver(dev) == 0)
              done = 0;
          }
    });
  } while (!done);
#endif
}

void libdevice_drivers_init0(void)
{
  libdevice_drivers_init(0);
}

void libdevice_drivers_init1(void)
{
  if (cpu_isbootstrap())
    libdevice_drivers_init(1);
}

error_t device_bind_driver(struct device_s *dev, const struct driver_s *drv)
{
  error_t err = -EBUSY;

  LOCK_SPIN_IRQ(&dev->lock);

  if (dev->status == DEVICE_NO_DRIVER)
    {
      err = 0;
      dev->drv = drv;
      dev->status = DEVICE_DRIVER_INIT_PENDING;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

error_t device_unbind_driver(struct device_s *dev)
{
  error_t err;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (dev->status)
    {
    default:
      err = -EINVAL;
      break;
    case DEVICE_NO_DRIVER:
    case DEVICE_DRIVER_INIT_PENDING:
    case DEVICE_DRIVER_INIT_FAILED:
      dev->status = DEVICE_NO_DRIVER;
      dev->drv = NULL;
      err = 0;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

error_t device_release_driver(struct device_s *dev)
{
  error_t err = -EBUSY;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (dev->status)
    {
    default:
      break;
#ifdef CONFIG_DEVICE_DRIVER_CLEANUP
    case DEVICE_DRIVER_INIT_DONE:
    case DEVICE_DRIVER_INIT_PARTIAL:
      if (dev->ref_count)
        break;
      if (dev->drv->f_cleanup(dev))
        break;
#endif
    case DEVICE_DRIVER_INIT_PENDING:
    case DEVICE_DRIVER_INIT_FAILED:
      err = 0;
      dev->status = DEVICE_DRIVER_INIT_PENDING;
      break;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

error_t device_init_driver(struct device_s *dev)
{
  const struct driver_s *drv = dev->drv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (dev->status)
    {
    default:
      err = -EBUSY;
      goto end;
    case DEVICE_DRIVER_INIT_PENDING:
    case DEVICE_DRIVER_INIT_PARTIAL:
      break;
    }

  uint32_t cl_missing = 0;

  /* check dependencies before trying device initialization */
  DEVICE_RES_FOREACH(dev, r, {
      const char *path;
      enum driver_class_e cl;

      switch (r->type)
        {
#ifdef CONFIG_DEVICE_CLOCK
        case DEV_RES_CLOCK_SRC:
          path = r->u.ptr[0];
          cl = DRIVER_CLASS_CLOCK;
          break;
#endif
#ifdef CONFIG_DEVICE_DMA
        case DEV_RES_DMA:
          path = r->u.ptr[0];
          cl = DRIVER_CLASS_DMA;
          break;
#endif
        case DEV_RES_DEV_PARAM:
          path = r->u.ptr[1];
          cl = r->u.dev_param.class_;
          break;
        default:
          continue;
        }

      struct device_s *dep = dev;
      if (device_get_by_path(&dep, path, NULL))
        goto missing;

      switch (dep->status)
        {
        case DEVICE_DRIVER_INIT_DONE:
          continue;
        case DEVICE_DRIVER_INIT_PARTIAL:
          if (cl == DRIVER_CLASS_NONE)
          default:
            goto missing;
        }
      if (!device_get_api(dep, cl, NULL))
        continue;
    missing:
      if (cl >= sizeof(cl_missing) * 8)
        {
          err = -EAGAIN;
          goto end;
        }
      cl_missing |= 1 << cl;
    });

  if (!(drv->flags & DRIVER_FLAGS_NO_DEPEND) && cl_missing)
    {
#if 0
      printk("device: postponed initialization of device %p `%s'\n", dev, dev->node.name);
#endif
      err = -EAGAIN;
      goto end;
    }

  printk("device: initialization of device %p `%s' using driver %p"
#if defined(CONFIG_DEVICE_DRIVER_DESC)
         " `%s'"
#endif
         "\n",
         dev, dev->node.name, drv
#if defined(CONFIG_DEVICE_DRIVER_DESC)
         , drv->desc
#endif
         );

  /* device init */
  err = drv->f_init(dev, cl_missing);

  switch (err)
    {
    default:
      printk("device: device %p `%s' initialization failed with return code %i\n",
             dev, dev->node.name, err);
      break;
    case 0:
      assert(dev->status == DEVICE_DRIVER_INIT_DONE ||
             dev->status == DEVICE_DRIVER_INIT_PARTIAL);
      break;
    case -EAGAIN:
      assert(dev->status == DEVICE_DRIVER_INIT_PENDING ||
             dev->status == DEVICE_DRIVER_INIT_PARTIAL);
      break;
    }

 end:;
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

error_t dev_driver_notsup_fcn(void)
{
  return -ENOTSUP;
}

DEV_USE(dev_use_generic)
{
  return 0;
}

