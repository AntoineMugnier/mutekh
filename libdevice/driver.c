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
#include <hexo/ordering.h>

#include <mutek/startup.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/startup.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/irq.h>
#include <device/class/enum.h>
#include <enums.h>

#include <assert.h>
#include <string.h>
#include <stdlib.h>

static uint_fast8_t device_init_pass = 0;

const char driver_class_e[] = ENUM_DESC_DRIVER_CLASS_E;

enum drv_init_loop_e {
  DRV_INIT_LOOP_AGAIN = 1,
  DRV_INIT_LOOP_ONGOING = 2
};

static error_t device_init_driver_nolock(struct device_s *dev);

error_t device_get_api(struct device_s *dev,
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
#ifdef CONFIG_DEVICE_ENUM
    case DEVICE_INIT_ENUM_ERR:
    case DEVICE_INIT_ENUM_DRV:
#endif
    case DEVICE_INIT_NODRV:
      UNREACHABLE();
#if defined(CONFIG_DEVICE_INIT_PARTIAL) || defined(CONFIG_DEVICE_INIT_ASYNC)
    case DEVICE_INIT_ONGOING:
# ifdef CONFIG_DEVICE_INIT_PARTIAL
    case DEVICE_INIT_PARTIAL:
      if (dev->init_mask & (1 << i))
        goto done;
      if (dev->status == DEVICE_INIT_ONGOING)
# endif
        return -EAGAIN;
#endif
    default:
      return -EBUSY;
    case DEVICE_INIT_DONE:
    done:
      if (api)
        *api = c;
      return 0;
    }
}

error_t device_last_number(struct device_s *dev,
                           enum driver_class_e cl,
                           uint_fast8_t *num)
{
  struct device_accessor_s acc;
  error_t err;

  acc.dev = dev;
  LOCK_SPIN_IRQ(&dev->lock);
  err = device_get_api(dev, cl, &acc.api);
  if (!err)
    {
      dev_use_t *use = dev->drv->f_use;
      err = use(&acc, DEV_USE_LAST_NUMBER);
    }
  LOCK_RELEASE_IRQ(&dev->lock);
  return err;
}

error_t device_start(struct device_accessor_s *acc)
{
  struct device_s *dev = acc->dev;
  dev_use_t *use = dev->drv->f_use;
  error_t err = 0;
  LOCK_SPIN_IRQ(&dev->lock);
  assert((dev->start_count + DEVICE_START_COUNT_INC) >>
         (CONFIG_DEVICE_USE_BITS + CONFIG_DEVICE_START_LOG2INC) == 0);
  err = use(acc, DEV_USE_START);
  if (!err)
    dev->start_count += DEVICE_START_COUNT_INC;
  LOCK_RELEASE_IRQ(&dev->lock);
  return err;
}

error_t device_stop(struct device_accessor_s *acc)
{
  struct device_s *dev = acc->dev;
  dev_use_t *use = dev->drv->f_use;
  error_t err = -EINVAL;
  LOCK_SPIN_IRQ(&dev->lock);
  if (dev->start_count >= DEVICE_START_COUNT_INC)
    {
      dev->start_count -= DEVICE_START_COUNT_INC;
      err = use(acc, DEV_USE_STOP);
      if (err)
        dev->start_count += DEVICE_START_COUNT_INC;
    }
  LOCK_RELEASE_IRQ(&dev->lock);
  return err;
}

static bool_t device_filter_accessor(struct device_node_s *node)
{
  struct device_s *dev = device_from_node(node);
  if (dev == NULL || dev->drv == NULL)
    return 0;
  return (dev->status == DEVICE_INIT_DONE
#if defined(CONFIG_DEVICE_INIT_ASYNC) || defined(CONFIG_DEVICE_INIT_PARTIAL)
          || dev->status == DEVICE_INIT_ONGOING
#endif
#if defined(CONFIG_DEVICE_INIT_PARTIAL)
          || dev->status == DEVICE_INIT_PARTIAL
#endif
          );
}

#if defined(CONFIG_DEVICE_ENUM) && defined(CONFIG_MUTEK_CONTEXT_SCHED)
error_t device_wait_accessor(struct device_accessor_s *acc, struct device_s *dev,
                             enum driver_class_e cl, uint_fast8_t number)
{
  struct device_enum_s edev;

  if (device_get_accessor(&edev.base, (void*)dev->node.parent, DRIVER_CLASS_ENUM, 0))
    return -EINVAL;

  struct dev_enum_rq_s rq = {
    .type = DEV_ENUM_INIT_EVENT,
    .init.dev = dev,
    .init.class_ = cl,
  };

  error_t err = dev_enum_wait_request(&edev, &rq);

  device_put_accessor(&edev.base);

  if (!err)
    {
      acc->dev = dev;
      acc->api = rq.init.api;
      acc->number = number;
      LOCK_SPIN_IRQ(&dev->lock);
      err = dev->drv->f_use(acc, DEV_USE_GET_ACCESSOR);
      if (err)
        dev->ref_count--;
      LOCK_RELEASE_IRQ(&dev->lock);
      if (!err)
        return 0;
    }

  acc->dev = NULL;
  return err;
}

error_t device_wait_accessor_by_path(struct device_accessor_s *acc, struct device_node_s *root,
                                     const char *path, enum driver_class_e cl)
{
  struct device_s *dev;
  uint_fast8_t number;
  error_t e = device_get_by_path(&dev, &number, root, path, &device_filter_accessor);
  if (e)
    return e;

  return device_wait_accessor(acc, dev, cl, number);
}
#endif

error_t device_get_accessor(struct device_accessor_s *acc, struct device_s *dev,
                            enum driver_class_e cl, uint_fast8_t number)
{
  error_t err;

  LOCK_SPIN_IRQ(&dev->lock);

  const struct driver_class_s *c;
  if (!(err = device_get_api(dev, cl, &c)))
    {
      acc->dev = dev;
      acc->api = c;
      acc->number = number;
      if (!(err = dev->drv->f_use(acc, DEV_USE_GET_ACCESSOR)))
        dev->ref_count++;
      else
        acc->dev = NULL;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

error_t device_copy_accessor(struct device_accessor_s *a,
                             const struct device_accessor_s *b)
{
  struct device_s *dev = b->dev;
  error_t err = 0;

  assert(dev);

  a->dev = b->dev;
  a->api = b->api;
  a->number = b->number;

  if (dev->drv->f_use == NULL ||
      !(err = dev->drv->f_use(a, DEV_USE_GET_ACCESSOR)))
    dev->ref_count++;
  else
    a->dev = NULL;

  return err;
}

void device_put_accessor(struct device_accessor_s *acc)
{
  struct device_s *dev = acc->dev;

  assert(dev != NULL);

  LOCK_SPIN_IRQ(&dev->lock);

  assert(dev->ref_count);

  dev->ref_count--;
  dev->drv->f_use(acc, DEV_USE_PUT_ACCESSOR);
  acc->dev = NULL;
  acc->api = NULL;

  LOCK_RELEASE_IRQ(&dev->lock);
}

error_t device_get_accessor_by_path(struct device_accessor_s *acc, struct device_node_s *root,
                                    const char *path, enum driver_class_e cl)
{
  struct device_s *dev;
  uint_fast8_t number;
  error_t e = device_get_by_path(&dev, &number, root, path, &device_filter_accessor);
  if (e)
    return e;

  return device_get_accessor(acc, dev, cl, number);
}

#ifdef CONFIG_DEVICE_TREE

static enum drv_init_loop_e device_find_driver_r(struct device_node_s *node)
{
  enum drv_init_loop_e lp = 0;

  struct device_s *dev = device_from_node(node);

  if (dev)
    {
      LOCK_SPIN_IRQ(&dev->lock);

      switch (dev->status)
        {
# ifdef CONFIG_DEVICE_ENUM
        case DEVICE_INIT_ENUM_DRV: {
#  ifdef CONFIG_DEVICE_DRIVER_REGISTRY
          const struct driver_registry_s *reg;
          struct device_enum_s e;

          dev->status = DEVICE_INIT_NODRV;

          if (dev->node.flags & DEVICE_FLAG_NO_AUTOBIND)
            break;

          /* get associated enumerator device */
          struct device_s *enum_dev = (void*)dev->node.parent;
          if (device_get_accessor(&e.base, enum_dev, DRIVER_CLASS_ENUM, 0))
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
                  dev->drv = reg->driver;
                  dev->status = DEVICE_INIT_PENDING;
                  lp |= DRV_INIT_LOOP_AGAIN;
                }
            }

          device_put_accessor(&e.base);

          if (dev->status != DEVICE_INIT_PENDING)
            break;
#  else
          dev->status = DEVICE_INIT_NODRV;
          break;
#  endif
        }
# endif

          /* try to intialize the device using the associated driver */
        case DEVICE_INIT_PENDING:
          if (dev->node.flags & DEVICE_FLAG_NO_AUTOINIT)
            break;

          if (device_init_pass == 0 &&
              !(dev->drv->flags & DRIVER_FLAGS_EARLY_INIT))
            break;

        case DEVICE_INIT_ONGOING:
# ifdef CONFIG_DEVICE_INIT_ASYNC
          if (!(dev->node.flags & DEVICE_FLAG_NO_STARTUP_WAIT))
            lp |= DRV_INIT_LOOP_ONGOING;
# endif
          if (device_init_driver_nolock(dev) == 0)
            lp |= DRV_INIT_LOOP_AGAIN;
          break;

#if defined(CONFIG_DEVICE_INIT_ASYNC) && defined(CONFIG_DEVICE_CLEANUP)
        case DEVICE_INIT_DECLINE:
          if (!dev->drv->f_cleanup(dev))
            dev->status = DEVICE_INIT_RELEASED;
          break;
#endif

        default:
          break;
        }

      LOCK_RELEASE_IRQ(&dev->lock);
    }

  DEVICE_NODE_FOREACH(node, child, {
      lp |= device_find_driver_r(child);
  });

  return lp;
}

static enum drv_init_loop_e device_find_driver(struct device_node_s *node)
{
  if (!node)
    node = device_tree_root();

  /* try to bind and init as many devices as possible */
  enum drv_init_loop_e lp;
  do {
    lp = device_find_driver_r(node);
  } while (lp & DRV_INIT_LOOP_AGAIN);

  return lp;
}

#endif

static void libdevice_drivers_init()
{
  enum drv_init_loop_e lp;
#ifdef CONFIG_DEVICE_TREE
  lp = device_find_driver(NULL);
#else
  struct device_s *dev;

  do {
    lp = 0;
    DEVICE_NODE_FOREACH(, node, {

        if (!(dev = device_from_node(node)))
          continue;

        LOCK_SPIN_IRQ(&dev->lock);

        switch (dev->status)
          {
          case DEVICE_INIT_PENDING:
            if (dev->node.flags & DEVICE_FLAG_NO_AUTOINIT)
              break;

            order_smp_read();
            if (device_init_pass == 0 &&
                !(dev->drv->flags & DRIVER_FLAGS_EARLY_INIT))
              break;

          case DEVICE_INIT_ONGOING:
# ifdef CONFIG_DEVICE_INIT_ASYNC
            if (!(dev->node.flags & DEVICE_FLAG_NO_STARTUP_WAIT))
              lp |= DRV_INIT_LOOP_ONGOING;
# endif
            if (device_init_driver_nolock(dev) == 0)
              lp |= DRV_INIT_LOOP_AGAIN;
            break;

# if defined(CONFIG_DEVICE_INIT_ASYNC) && defined(CONFIG_DEVICE_CLEANUP)
          case DEVICE_INIT_DECLINE:
            if (!dev->drv->f_cleanup(dev))
              dev->status = DEVICE_INIT_RELEASED;
            break;
# endif

          default:
            break;
          }

        LOCK_RELEASE_IRQ(&dev->lock);
    });
  } while (lp & DRV_INIT_LOOP_AGAIN);
#endif

  order_smp_read();
  if (device_init_pass == 1
#ifdef CONFIG_DEVICE_INIT_ASYNC
      && !(lp & DRV_INIT_LOOP_ONGOING)
#endif
      )
    {
      INIT_DEVREADY_INIT();
      device_init_pass = 2;
      order_smp_write();
    }
}

#ifdef CONFIG_DEVICE_INIT_ASYNC

static struct kroutine_s device_deferred_init_kr;

static KROUTINE_EXEC(device_deferred_init)
{
  libdevice_drivers_init();
}

static void device_async_init_event(struct device_s *dev)
{
#ifdef CONFIG_DEVICE_ENUM
  struct device_s *enum_dev = (void*)dev->node.parent;
  const struct driver_s *enum_drv = enum_dev->drv;
  lock_spin(&enum_dev->lock);
  enum_drv->f_use(dev, DEV_USE_ENUM_CHILD_INIT);
  lock_release(&enum_dev->lock);
#endif

  kroutine_exec(&device_deferred_init_kr);
}

#endif

static void device_update_status(struct device_s *dev, error_t err)
{
 switch (err)
    {
    case 0:
      printk("done\n");
      dev->status = DEVICE_INIT_DONE;
      break;

    case -EAGAIN:
#if defined(CONFIG_DEVICE_INIT_PARTIAL) || defined(CONFIG_DEVICE_INIT_ASYNC)
      printk("ongoing\n");
      dev->status = DEVICE_INIT_ONGOING;
      break;
#else
      UNREACHABLE();
#endif

    default:
#ifdef CONFIG_DEVICE_INIT_PARTIAL
      if (dev->init_mask)
        {
          printk("partial (err=%i)\n", err);
          dev->status = DEVICE_INIT_PARTIAL;
        }
      else
#endif
        {
          printk("failed (err=%i)\n", err);
#ifdef CONFIG_DEVICE_TREE
          assert(device_list_isempty(&dev->node.children));
#endif
          dev->status = DEVICE_INIT_FAILED;
        }
      break;
    }

#ifdef CONFIG_DEVICE_INIT_ASYNC
  order_smp_read();
  if (device_init_pass != 0)
    device_async_init_event(dev);
#endif
}

#ifdef CONFIG_DEVICE_INIT_ASYNC

void device_async_init_done(struct device_s *dev, error_t err)
{
  assert(dev->status == DEVICE_INIT_ONGOING);

  printk("device: init %p %-24s ",
         dev, dev->node.name);

  device_update_status(dev, err);
}

# if defined(CONFIG_DEVICE_CLEANUP)
void device_async_cleanup_done(struct device_s *dev)
{
  assert(dev->status == DEVICE_INIT_DECLINE);
  dev->status = DEVICE_INIT_RELEASED;
  kroutine_exec(&device_deferred_init_kr);
}
# endif

#endif

void libdevice_drivers_init0(void)
{
  libdevice_drivers_init();
}

void libdevice_drivers_init1(void)
{
  if (cpu_isbootstrap())
    {
      device_init_pass = 1;
      order_smp_write();

#ifdef CONFIG_DEVICE_INIT_ASYNC
      kroutine_init_deferred(&device_deferred_init_kr, device_deferred_init);
#endif
      libdevice_drivers_init();
    }
}

error_t device_bind_driver(struct device_s *dev, const struct driver_s *drv)
{
  error_t err;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (dev->status)
    {
#ifdef CONFIG_DEVICE_ENUM
    case DEVICE_INIT_ENUM_DRV:
#endif
    case DEVICE_INIT_NODRV:
    case DEVICE_INIT_RELEASED:
    case DEVICE_INIT_PENDING:
      err = 0;
      dev->drv = drv;
      dev->status = DEVICE_INIT_RELEASED;
      break;
    default:
      err = -EBUSY;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

#if defined(CONFIG_DEVICE_CLEANUP)
error_t device_unbind_driver(struct device_s *dev)
{
  error_t err;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (dev->status)
    {
    default:
      err = -EBUSY;
      break;
    case DEVICE_INIT_PENDING:
    case DEVICE_INIT_RELEASED:
      dev->drv = NULL;
    case DEVICE_INIT_FAILED:
#ifdef CONFIG_DEVICE_ENUM
    case DEVICE_INIT_ENUM_DRV:
#endif
      dev->status = DEVICE_INIT_NODRV;
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
#ifdef CONFIG_DEVICE_CLEANUP
    case DEVICE_INIT_DONE:
# ifdef CONFIG_DEVICE_INIT_PARTIAL
    case DEVICE_INIT_PARTIAL:
# endif
      if (dev->ref_count || dev->start_count
# ifdef CONFIG_DEVICE_TREE
          || !device_list_isempty(&dev->node.children)
# endif
          )
        break;
# ifdef CONFIG_DEVICE_INIT_ASYNC
    case DEVICE_INIT_DECLINE:
# endif
      switch (dev->drv->f_cleanup(dev))
        {
        case 0:
          goto done;
# ifdef CONFIG_DEVICE_INIT_ASYNC
        case -EAGAIN:
          dev->status = DEVICE_INIT_DECLINE;
          err = 0;
# endif
        default:
          break;
        }
      break;
#endif
    case DEVICE_INIT_PENDING:
    case DEVICE_INIT_FAILED:
    case DEVICE_INIT_RELEASED:
    done:
      err = 0;
      dev->status = DEVICE_INIT_RELEASED;
    default:
      break;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}
#endif

error_t device_init_driver(struct device_s *dev)
{
  error_t err;
  LOCK_SPIN_IRQ(&dev->lock);
  err = device_init_driver_nolock(dev);
  LOCK_RELEASE_IRQ(&dev->lock);
  return err;
}

static error_t device_init_driver_nolock(struct device_s *dev)
{
  const struct driver_s *drv = dev->drv;
  error_t err = 0;
#ifdef CONFIG_DEVICE_SLEEP
  static typeof(dev->sleep_order) device_sleep_order = 0;
#endif

  switch (dev->status)
    {
#if defined(CONFIG_DEVICE_INIT_PARTIAL) || defined(CONFIG_DEVICE_INIT_ASYNC)
    case DEVICE_INIT_ONGOING:
      if (drv->flags & DRIVER_FLAGS_RETRY_INIT)
        break;
      return -EAGAIN;
#endif
    default:
      return -EBUSY;

    case DEVICE_INIT_FAILED:
      dev->status = DEVICE_INIT_PENDING;
    case DEVICE_INIT_PENDING:
    case DEVICE_INIT_RELEASED:
#ifdef CONFIG_DEVICE_SLEEP
      dev->sleep_order = device_sleep_order++;
      device_sleep_queue_orphan(dev);
      dev->sleep_policy = DEVICE_SLEEP_CPUIDLE;
#endif
#ifdef CONFIG_DEVICE_INIT_PARTIAL
      dev->init_mask = 0;
#endif
      break;
    }

  uint32_t cl_missing = 0;

  /* check dependencies before trying device initialization */
  DEVICE_RES_FOREACH(dev, r, {
      const char *path;
      __unused__ enum driver_class_e cl;

      switch (r->type)
        {
#ifdef CONFIG_DEVICE_CLOCK
        case DEV_RES_CLK_SRC:
          path = r->u.ptr[0];
          cl = DRIVER_CLASS_CMU;
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

      struct device_s *dep;
      if (!device_get_by_path(&dep, NULL, &dev->node, path, NULL))
        {
          switch (dep->status)
            {
            case DEVICE_INIT_DONE:
              continue;
#ifdef CONFIG_DEVICE_INIT_PARTIAL
            case DEVICE_INIT_ONGOING:
            case DEVICE_INIT_PARTIAL:
              if (cl != DRIVER_CLASS_NONE && !device_get_api(dep, cl, NULL))
                continue;
              if (dev->status == DEVICE_INIT_ONGOING)
                break;
#elif defined(CONFIG_DEVICE_INIT_ASYNC)
            case DEVICE_INIT_ONGOING:
              break;
#endif
            default:
            missing:
              printk("device: init %p %-24s dep error `%s'\n",
                     dev, dev->node.name, path);
              dev->status = DEVICE_INIT_FAILED;
              return -ENOENT;

            case DEVICE_INIT_ENUM_DRV:
            case DEVICE_INIT_PENDING:
              break;
            }
        }
#ifndef CONFIG_DEVICE_TREE
      else
        goto missing;
#endif

#ifdef CONFIG_DEVICE_INIT_PARTIAL
      if (cl >= sizeof(cl_missing) * 8)
#else
      if (!(drv->flags & DRIVER_FLAGS_NO_DEPEND))
#endif
        return -EAGAIN;

#ifdef CONFIG_DEVICE_INIT_PARTIAL
      cl_missing |= 1 << cl;
#endif
    });

#ifdef CONFIG_DEVICE_INIT_PARTIAL
  if (!(drv->flags & DRIVER_FLAGS_NO_DEPEND) && cl_missing)
    return -EAGAIN;
#endif

  printk("device: init %p %-24s ",
         dev, dev->node.name);

  /* device init */
  err = drv->f_init(dev, cl_missing);

  device_update_status(dev, err);

  return err;
}

error_t dev_driver_notsup_fcn(void)
{
  return -ENOTSUP;
}

DEV_USE(dev_use_generic)
{
  switch (op)
    {
    case DEV_USE_GET_ACCESSOR: {
      struct device_accessor_s *acc = param;
      if (acc->number)
        return -ENOTSUP;
    }
#ifdef CONFIG_DEVICE_CLOCK
    case DEV_USE_CLOCK_GATES:
#endif
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    case DEV_USE_CLOCK_NOTIFY:
#endif
    case DEV_USE_PUT_ACCESSOR:
    case DEV_USE_START:
    case DEV_USE_STOP:
      return 0;
    default:
      return -ENOTSUP;
    }

  return 0;
}

