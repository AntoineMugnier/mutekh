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

    Copyright (c) 2012 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
    Copyright (c) 2012 Institut Telecom / Telecom ParisTech

*/

#include <string.h>
#include <stdio.h>

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/class/timer.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#define  TIMER_VALUE		0
#define  TIMER_MODE		4
# define TIMER_MODE_EN          0x01
# define TIMER_MODE_IRQEN	0x02
#define  TIMER_PERIOD           8
#define  TIMER_IRQ		12

#define TIMER_REG_ADDR(a, r, n)  ((a) + (r) + (n) * 0x10)

#define SOCLIB_TIMER_MIN_PERIOD     2000
#define SOCLIB_TIMER_DEFAULT_PERIOD 250000

/*

  When interrutons are available, the timer value is a 64 bits
  software counter and the hardware timer period is used as an input
  frequency divider configured from current resolution.

  When interrutons support is disabled in the configuration, the
  timer value is taken from the hardware timer register, the
  resolution is fixed to 1 and the max timer value is 2^32-1.

*/

struct soclib_timer_state_s
{
#ifdef CONFIG_DEVICE_IRQ
  dev_timer_queue_root_t queue;
  dev_timer_value_t value;
  dev_timer_res_t   period;
#endif
  uint_fast8_t start_count;
};

struct soclib_timer_private_s
{
  uintptr_t addr;
  uintptr_t t_count;
  struct dev_irq_ep_s *irq_eps;
  struct soclib_timer_state_s t[0];
};

#ifdef CONFIG_DEVICE_IRQ
static DEV_IRQ_EP_PROCESS(soclib_timer_irq)
{
  struct device_s *dev = ep->dev;
  struct soclib_timer_private_s *pv = dev->drv_pv;
  uint_fast8_t number = ep - pv->irq_eps;

  assert(number < pv->t_count);
  lock_spin(&dev->lock);

  while (1)
    {
      if (!endian_le32(cpu_mem_read_32(TIMER_REG_ADDR(pv->addr, TIMER_IRQ, number))))
        break;

      cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_IRQ, number), 0);

      struct soclib_timer_state_s *p = pv->t + number;

      p->value++;

      struct dev_timer_rq_s *rq = dev_timer_queue_head(&p->queue);

      while (rq)
        {
          if (rq->deadline > p->value)
            break;

          rq->drvdata = 0;
          dev_timer_queue_pop(&p->queue);

          if (rq->callback(rq, 0))
            {
              if (rq->delay)
                rq->deadline = p->value + rq->delay;

              rq->drvdata = p;
              dev_timer_queue_insert_ascend(&p->queue, rq);
            }
          else
            {
              // stop timer if not in use
              if (--p->start_count == 0)
                cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, number), endian_le32(TIMER_MODE_IRQEN));
            }

          rq = dev_timer_queue_head(&p->queue);
        }
    }
     
  lock_release(&dev->lock);
}
#endif

static DEVTIMER_REQUEST(soclib_timer_request)
{
# ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = tdev->dev;
  struct soclib_timer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  if (tdev->number >= pv->t_count)
    return -ENOENT;

  if (!rq)
    return 0;

  rq->tdev = tdev;

  struct soclib_timer_state_s *p = pv->t + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);

  uint64_t val = p->value;

  if (cancel)
    {
      if (rq->drvdata == p)
        {
          dev_timer_queue_remove(&p->queue, rq);
          rq->drvdata = 0;

          // stop timer if not in use
          if (--p->start_count == 0)
            cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, tdev->number), endian_le32(TIMER_MODE_IRQEN));
        }
      else
        {
          err = -ETIMEDOUT;
        }
    }
  else
    {
      do {
        if (rq->delay)
          rq->deadline = val + rq->delay;

        if (rq->deadline <= val)
          {
            if (rq->callback(rq, 1))
              continue;
            err = ETIMEDOUT;
            goto done;
          }
      } while (0);

      rq->drvdata = p;
      dev_timer_queue_insert_ascend(&p->queue, rq);

      /* start timer if needed */
      if (p->start_count++ == 0)
        {
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_VALUE, tdev->number), 0);
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_PERIOD, tdev->number), endian_le32(p->period));
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, tdev->number), endian_le32(TIMER_MODE_EN | TIMER_MODE_IRQEN));
        }
    }
 done:

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
# else
  return -ENOTSUP;
# endif
}

static DEVTIMER_START_STOP(soclib_timer_state_start_stop)
{
  struct device_s *dev = tdev->dev;
  struct soclib_timer_private_s *pv = dev->drv_pv;

  if (tdev->number >= pv->t_count)
    return -ENOENT;

  error_t err = 0;

  struct soclib_timer_state_s *p = pv->t + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);

# ifdef CONFIG_DEVICE_IRQ
  if (start)
    {
      if (p->start_count++ == 0)
        {
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_VALUE, tdev->number), 0);
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_PERIOD, tdev->number), endian_le32(p->period));
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, tdev->number), endian_le32(TIMER_MODE_EN | TIMER_MODE_IRQEN));
        }
    }
  else
    {
      if ((p->start_count == 0) || (p->start_count == 1 && !dev_timer_queue_isempty(&p->queue)))
        err = -EINVAL;
      else if (--p->start_count == 0)
        cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, tdev->number), endian_le32(TIMER_MODE_IRQEN));
    }
# else
  if (start)
    {
      if (p->start_count++ == 0)
        cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, tdev->number), endian_le32(TIMER_MODE_EN));
    }
  else
    {
      if (p->start_count == 0)
        err = -EINVAL;
      else if (--p->start_count == 0)
        cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, tdev->number), 0);
    }
# endif

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEVTIMER_GET_VALUE(soclib_timer_get_value)
{
  struct device_s *dev = tdev->dev;
  struct soclib_timer_private_s *pv = dev->drv_pv;

  if (tdev->number >= pv->t_count)
    return -ENOENT;
 
# ifdef CONFIG_DEVICE_IRQ
  struct soclib_timer_state_s *p = pv->t + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);
  *value = p->value;
  LOCK_RELEASE_IRQ(&dev->lock);
#else
  *value = endian_le32(cpu_mem_read_32(TIMER_REG_ADDR(pv->addr, TIMER_VALUE, tdev->number)));
#endif

  return 0;
}

static DEVTIMER_RESOLUTION(soclib_timer_resolution)
{
  struct device_s *dev = tdev->dev;
  struct soclib_timer_private_s *pv = dev->drv_pv;

  if (tdev->number >= pv->t_count)
    return -ENOENT;

  error_t err = 0;

# ifdef CONFIG_DEVICE_IRQ
  struct soclib_timer_state_s *p = pv->t + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (res)
    {
      if (*res)
        {
          if (p->start_count)
            {
              err = -EBUSY;
            }
          else if (*res < SOCLIB_TIMER_MIN_PERIOD)
            {
              p->period = SOCLIB_TIMER_MIN_PERIOD;
              err = -ERANGE;
            }
          else
            {
              p->period = *res;
            }
        }
      *res = p->period;
    }

  if (max)
    *max = 0xffffffffffffffffULL;

  LOCK_RELEASE_IRQ(&dev->lock);

#else
  if (res)
    {
      if (*res != 0)
        err = -ENOTSUP;
      *res = 1;
    }

  if (max)
    *max = 0xffffffff;
#endif

  return err;
}

const struct driver_timer_s  soclib_timer_timer_drv =
{
  .class_         = DRIVER_CLASS_TIMER,
  .f_request      = soclib_timer_request,
  .f_start_stop   = soclib_timer_state_start_stop,
  .f_get_value    = soclib_timer_get_value,
  .f_resolution   = soclib_timer_resolution,
};

/************************************************************************/

static const struct devenum_ident_s  soclib_timer_ids[] =
{
  DEVENUM_FDTNAME_ENTRY("soclib:vci_timer"),
  { 0 }
};

static DEV_INIT(soclib_timer_init);
static DEV_CLEANUP(soclib_timer_cleanup);

const struct driver_s  soclib_timer_drv =
{
  .desc           = "Soclib VciTimer",
  .id_table       = soclib_timer_ids,
  .f_init         = soclib_timer_init,
  .f_cleanup      = soclib_timer_cleanup,

  .classes        = {
    &soclib_timer_timer_drv,
    0
  }
};


static DEV_INIT(soclib_timer_init)
{
  struct soclib_timer_private_s  *pv;
  uint_fast8_t i;

  uintptr_t t_count = 1;

  if (device_get_param_uint(dev, "count", &t_count))
    printk("warning: vci_timer device `%p' has no `count' parameter, assuming only one timer is available.\n", dev);

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv) + t_count * (sizeof(struct soclib_timer_state_s)
#ifdef CONFIG_DEVICE_IRQ
                                           + sizeof(struct dev_irq_ep_s)
#endif
                                           ), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->t_count = t_count;
  dev->drv_pv = pv;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

#ifdef CONFIG_DEVICE_IRQ
  dev_timer_res_t resolution = SOCLIB_TIMER_DEFAULT_PERIOD;
  device_get_param_uint(dev, "period", &resolution);
  if (resolution < SOCLIB_TIMER_MIN_PERIOD)
    resolution = SOCLIB_TIMER_MIN_PERIOD;

  pv->irq_eps = (void*)(pv->t + t_count);

  device_irq_source_init(dev, pv->irq_eps, t_count, soclib_timer_irq);

  if (device_irq_source_link(dev, pv->irq_eps, t_count, 1))
    goto err_mem;
#endif

  for (i = 0; i < t_count; i++)
    {
      struct soclib_timer_state_s *p = pv->t + i;
      p->start_count = 0;

# ifdef CONFIG_DEVICE_IRQ
      p->value = 0;
      dev_timer_queue_init(&p->queue);
      p->period = resolution;
      cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, i), endian_le32(TIMER_MODE_IRQEN));
      cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_IRQ, i), 0);
# else
      cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_PERIOD, i), 0xffffffff);
      cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, i), 0);
# endif
    }

  dev->drv = &soclib_timer_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(soclib_timer_cleanup)
{
  struct soclib_timer_private_s *pv = dev->drv_pv;

  uint_fast8_t i;
  for (i = 0; i < pv->t_count; i++)
    {
      cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, i), 0);
#ifdef CONFIG_DEVICE_IRQ
      struct soclib_timer_state_s *p = pv->t + i;
      dev_timer_queue_destroy(&p->queue);
#endif
    }

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_unlink(dev, pv->irq_eps, pv->t_count);
#endif

  mem_free(pv);
}

REGISTER_DRIVER(soclib_timer_drv);

