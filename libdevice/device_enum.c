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

#include <device/class/enum.h>
#include <device/request.h>

extern error_t
dev_enum_wait_request(const struct device_enum_s *accessor,
                      struct dev_enum_rq_s *rq);

void
dev_drv_enum_child_init(dev_request_queue_root_t *q, struct device_s *cdev)
{
  GCT_FOREACH(dev_request_queue, q, rq_, {
      struct dev_enum_rq_s *rq = dev_enum_rq_s_cast(rq_);

      switch (rq->type)
        {
        case DEV_ENUM_INIT_EVENT:
          if (rq->init.dev == cdev)
            {
              error_t err = device_get_api(cdev, rq->init.class_, &rq->init.api);
              switch (err)
                {
#if defined(CONFIG_DEVICE_INIT_PARTIAL) || defined(CONFIG_DEVICE_INIT_ASYNC)
                case -EAGAIN:
                  break;
#endif
                default:
                  cdev->ref_count--;
                  rq->error = err;
                case 0:
                  dev_enum_rq_done(rq);
                  rq->base.drvdata = NULL;
                  GCT_FOREACH_DROP;
                }
            }
          break;

        case DEV_ENUM_LIST_EVENT:
          break;
        }
  });
}

void
dev_drv_enum_init_enqueue(dev_request_queue_root_t *q, struct dev_enum_rq_s *rq)
{
  struct device_s *cdev = rq->init.dev;

  lock_spin(&cdev->lock);

  switch (cdev->status)
    {
#if defined(CONFIG_DEVICE_INIT_PARTIAL) || defined(CONFIG_DEVICE_INIT_ASYNC)
    case DEVICE_INIT_ONGOING:
#endif
#ifdef CONFIG_DEVICE_INIT_PARTIAL
    case DEVICE_INIT_PARTIAL:
#endif
    case DEVICE_INIT_DONE: {
      error_t err = device_get_api(cdev, rq->init.class_, &rq->init.api);
      switch (err)
        {
#if defined(CONFIG_DEVICE_INIT_PARTIAL) || defined(CONFIG_DEVICE_INIT_ASYNC)
        case -EAGAIN:
          goto postpone;
#endif
        case 0:
          cdev->ref_count++;
        default:
          rq->error = err;
          dev_enum_rq_done(rq);
          break;
        }
      break;
    }
    case DEVICE_INIT_ENUM_DRV:
    case DEVICE_INIT_PENDING:
    postpone:
      cdev->ref_count++;
      rq->base.drvdata = q;
      dev_enum_rq_pushback(q, rq);
      break;
    default:
      rq->error = -EBUSY;
      dev_enum_rq_done(rq);
      break;
    }

  lock_release(&cdev->lock);
}

void dev_drv_enum_request_generic(dev_request_queue_root_t *q,
                                  struct device_s *dev,
                                  struct dev_enum_rq_s *rq)
{
  rq->base.drvdata = NULL;
  rq->error = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
    {
    case DEV_ENUM_LIST_EVENT:
      if (rq->list.rev < 1)
        {
          dev_enum_rq_done(rq);
          rq->list.rev = 1;
          break;
        }
      rq->base.drvdata = q;
      break;

    case DEV_ENUM_INIT_EVENT:
      if (rq->init.dev->node.parent != &dev->node)
        {
          rq->error = -ENOENT;
          dev_enum_rq_done(rq);
          break;
        }
      dev_drv_enum_init_enqueue(q, rq);
      break;
    }

  LOCK_RELEASE_IRQ(&dev->lock);
}

error_t dev_drv_enum_cancel_generic(dev_request_queue_root_t *q,
                                    struct device_s *dev,
                                    struct dev_enum_rq_s *rq)
{
  error_t err = -EBUSY;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->base.drvdata == q)
    {
      err = 0;
      switch (rq->type)
        {
        case DEV_ENUM_INIT_EVENT:
          dev_enum_rq_remove(q, rq);
        default:
          rq->base.drvdata = NULL;
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

