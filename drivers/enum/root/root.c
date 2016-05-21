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

    Copyright (c) 2016 Alexandre Becoulet <alexandre.becoulet@free.fr>

*/

#include <hexo/types.h>

#include <device/class/enum.h>
#include <device/device.h>
#include <device/driver.h>
#include <mutek/mem_alloc.h>

#ifdef CONFIG_DEVICE_ENUM

DRIVER_PV(struct enum_root_pv_s
{
  dev_request_queue_root_t queue;
});

static DEV_ENUM_MATCH_DRIVER(enum_root_match_driver)
{
  return 0;
}

static DEV_ENUM_REQUEST(enum_root_request)
{
  struct device_s *dev = accessor->dev;
  struct enum_root_pv_s *pv = dev->drv_pv;

  return dev_drv_enum_request_generic(&pv->queue, dev, rq);
}

static DEV_ENUM_CANCEL(enum_root_cancel)
{
  struct device_s *dev = accessor->dev;
  struct enum_root_pv_s *pv = dev->drv_pv;

  return dev_drv_enum_cancel_generic(&pv->queue, dev, rq);
}

static DEV_USE(enum_root_use)
{
  switch (op)
    {
    case DEV_USE_ENUM_CHILD_INIT: {
      struct device_s *cdev = param;
      struct device_s *dev = (void*)cdev->node.parent;
      struct enum_root_pv_s *pv = dev->drv_pv;
      dev_drv_enum_child_init(&pv->queue, cdev);
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

#else

# define enum_root_use dev_use_generic

#endif

static DEV_INIT(enum_root_init)
{
#ifdef CONFIG_DEVICE_ENUM
  struct enum_root_pv_s *pv;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  dev->drv_pv = pv;

  dev_request_queue_init(&pv->queue);
#endif

  return 0;
}

static DEV_CLEANUP(enum_root_cleanup)
{
#if 0 && defined(CONFIG_DEVICE_ENUM)
  struct enum_root_pv_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->queue))
    return -EBUSY;

  mem_free(pv);

  return 0;
#endif

  return -EBUSY;
}

DRIVER_DECLARE(enum_root_drv, DRIVER_FLAGS_EARLY_INIT,
               "MutekH root enumerator", enum_root,
#ifdef CONFIG_DEVICE_ENUM
               DRIVER_ENUM_METHODS(enum_root)
#else
               NULL
#endif
               );

