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
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/timer.h>
#include <device/irq.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/kroutine.h>

#define TIMER_REG_SCALER         0x00
#define TIMER_REG_SC_RELOAD      0x04
#define TIMER_REG_CFG            0x08
#define  TIMER_CFG_NTIMERS       0x00000007
#define  TIMER_CFG_SIRQ          0x00000100
#define  TIMER_CFG_DFREEZE       0x00000200

#define TIMER_REG_COUNTER        0x00
#define TIMER_REG_RELOAD         0x04
#define TIMER_REG_CTRL           0x08
#define  TIMER_CTRL_ENABLED      0x00000001
#define  TIMER_CTRL_RESTART      0x00000002
#define  TIMER_CTRL_LOAD         0x00000004
#define  TIMER_CTRL_IE           0x00000008
#define  TIMER_CTRL_IP           0x00000010
#define  TIMER_CTRL_CHAIN        0x00000020
#define  TIMER_CTRL_DEBUGHLT     0x00000040

#define TIMER_REG_ADDR(a, r, n)  ((a) + 0x10 + (r) + (n) * 0x10)

/*

  When interrutons are available, the timer value is a 64 bits
  software counter and the hardware timer period is used as an input
  frequency divider configured from current resolution.

  When interrutons support is disabled in the configuration, the
  timer value is taken from the hardware timer register, the
  resolution is fixed to 1 and the max timer value is 2^32-1.

*/

struct gptimer_state_s
{
#ifdef CONFIG_DEVICE_IRQ
  dev_timer_queue_root_t queue;
  dev_timer_value_t value;
  dev_timer_res_t   period;
#endif
  uint_fast8_t start_count;
};

struct gptimer_private_s
{
  uintptr_t addr;
  uintptr_t t_count;
  struct dev_irq_ep_s *irq_eps;
  struct gptimer_state_s t[0];
};

#ifdef CONFIG_DEVICE_IRQ
static inline bool_t gptimer_irq_process(struct device_s *dev, uint_fast8_t number)
{
  struct gptimer_private_s *pv = dev->drv_pv;

  assert(number < pv->t_count);

  if ((endian_be32(TIMER_CTRL_IP) & cpu_mem_read_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, number))) == 0)
    return 0;

  cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, number),
                   endian_be32(TIMER_CTRL_IP | TIMER_CTRL_ENABLED |
                               TIMER_CTRL_RESTART | TIMER_CTRL_IE));

  struct gptimer_state_s *p = pv->t + number;

  p->value++;

  while (1)
    {
      struct dev_timer_rq_s *rq = dev_timer_queue_head(&p->queue);

      if (rq == NULL)
        {
          /* stop timer if not in use */
          p->start_count &= ~1;
          if (p->start_count == 0)
            cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, number), endian_be32(TIMER_CTRL_IE));
        }

      if (rq->deadline > p->value)
        break;

      rq->drvdata = 0;
      dev_timer_queue_pop(&p->queue);

      lock_release(&dev->lock);
      kroutine_exec(&rq->kr, 0);
      lock_spin(&dev->lock);
    }

  return 1;
}

static DEV_IRQ_EP_PROCESS(gptimer_irq_single)
{
  struct device_s *dev = ep->dev;
  struct gptimer_private_s *pv = dev->drv_pv;

  lock_spin(&dev->lock);

  bool_t done;

  do {
    uint_fast8_t i;
    done = 1;

    for (i = 0; i < pv->t_count; i++)
      {
        if (gptimer_irq_process(pv, i))
          done = 0;
      }
  } while (!done);

  lock_release(&dev->lock);
}

static DEV_IRQ_EP_PROCESS(gptimer_irq_separate)
{
  struct device_s *dev = ep->dev;
  struct gptimer_private_s *pv = dev->drv_pv;
  uint_fast8_t number = ep - pv->irq_eps;

  lock_spin(&dev->lock);

  while (gptimer_irq_process(pv, number))
    ;

  lock_release(&dev->lock);
}

#endif

static DEVTIMER_CANCEL(gptimer_cancel)
{
# ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = tdev->dev;
  struct gptimer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  if (tdev->number >= pv->t_count)
    return -ENOENT;

  assert(rq->tdev == tdev);

  struct gptimer_state_s *p = pv->t + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->drvdata == p)
    {
      dev_timer_queue_remove(&p->queue, rq);
      rq->drvdata = 0;

      // stop timer if not in use
      if (dev_timer_queue_isempty(&p->queue))
        {
          p->start_count &= ~1;
          if (p->start_count == 0)
            cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, tdev->number), endian_be32(TIMER_CTRL_IE));
        }
    }
  else
    {
      err = -ETIMEDOUT;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
# else
  return -ENOTSUP;
# endif
}

static DEVTIMER_REQUEST(gptimer_request)
{
# ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = tdev->dev;
  struct gptimer_private_s *pv = dev->drv_pv;
  error_t err = 0;

  if (tdev->number >= pv->t_count)
    return -ENOENT;

  rq->tdev = tdev;

  struct gptimer_state_s *p = pv->t + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);

  uint64_t val = p->value;

  if (rq->delay)
    rq->deadline = val + rq->delay;

  if (rq->deadline <= val)
    err = ETIMEDOUT;
  else
    {
      rq->drvdata = p;
      dev_timer_queue_insert(&p->queue, rq);

      /* start timer if needed */
      if (p->start_count == 0)
        {
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_COUNTER, tdev->number), endian_be32(p->period));
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_RELOAD, tdev->number), endian_be32(p->period));
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, tdev->number), endian_be32(TIMER_CTRL_ENABLED | TIMER_CTRL_RESTART | TIMER_CTRL_IE));
        }
      p->start_count |= 1;
    }

 done:;
  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
# else
  return -ENOTSUP;
# endif
}

static DEVTIMER_START_STOP(gptimer_state_start_stop)
{
  struct device_s *dev = tdev->dev;
  struct gptimer_private_s *pv = dev->drv_pv;

  if (tdev->number >= pv->t_count)
    return -ENOENT;

  error_t err = 0;

  struct gptimer_state_s *p = pv->t + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);

# ifdef CONFIG_DEVICE_IRQ
  if (start)
    {
      if (p->start_count == 0)
        {
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_COUNTER, tdev->number), endian_be32(p->period));
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_RELOAD, tdev->number), endian_be32(p->period));
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, tdev->number), endian_be32(TIMER_CTRL_ENABLED | TIMER_CTRL_RESTART | TIMER_CTRL_IE));
        }
      p->start_count += 2;
    }
  else
    {
      if (p->start_count < 2)
        err = -EINVAL;
      else
        {
          p->start_count -= 2;
          if (p->start_count == 0)
            cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, tdev->number), endian_be32(TIMER_CTRL_IE));
        }
    }
# else
  if (start)
    {
      if (p->start_count++ == 0)
        cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, tdev->number), endian_be32(TIMER_CTRL_ENABLED | TIMER_CTRL_RESTART));
    }
  else
    {
      if (p->start_count == 0)
        err = -EINVAL;
      else if (--p->start_count == 0)
        cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, tdev->number), 0);
    }
# endif

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEVTIMER_GET_VALUE(gptimer_get_value)
{
  struct device_s *dev = tdev->dev;
  struct gptimer_private_s *pv = dev->drv_pv;

  if (tdev->number >= pv->t_count)
    return -ENOENT;
 
# ifdef CONFIG_DEVICE_IRQ
  struct gptimer_state_s *p = pv->t + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);
  *value = p->value;
  LOCK_RELEASE_IRQ(&dev->lock);
#else
  *value = ~endian_be32(cpu_mem_read_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_COUNTER, tdev->number)));
#endif

  return 0;
}

static DEVTIMER_RESOLUTION(gptimer_resolution)
{
  struct device_s *dev = tdev->dev;
  struct gptimer_private_s *pv = dev->drv_pv;

  if (tdev->number >= pv->t_count)
    return -ENOENT;

  error_t err = 0;
  uint32_t sc = endian_be32(cpu_mem_read_32(pv->addr + TIMER_REG_SC_RELOAD)) + 1;

# ifdef CONFIG_DEVICE_IRQ
  struct gptimer_state_s *p = pv->t + tdev->number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (res)
    {
      if (*res)
        {
          if (p->start_count)
            {
              err = -EBUSY;
            }
          else
            {
              dev_timer_res_t r = *res / sc;
              if (r < 1)
                r = 1;
              p->period = r;
              r *= sc;
              if (r != *res)
                err = -ERANGE;
              *res = r;
            }
        }
      else
        {
          *res = p->period * sc;
        }
    }

  if (max)
    *max = 0xffffffffffffffffULL;

  LOCK_RELEASE_IRQ(&dev->lock);

#else
  if (res)
    {
      if (*res != 0)
        err = -ENOTSUP;
      *res = sc;
    }

  if (max)
    *max = 0xffffffff;
#endif

  return err;
}

const struct driver_timer_s  gptimer_timer_drv =
{
  .class_         = DRIVER_CLASS_TIMER,
  .f_cancel       = gptimer_cancel,
  .f_request      = gptimer_request,
  .f_start_stop   = gptimer_state_start_stop,
  .f_get_value    = gptimer_get_value,
  .f_resolution   = gptimer_resolution,
};

/************************************************************************/

static const struct devenum_ident_s  gptimer_ids[] =
{
  DEVENUM_GAISLER_ENTRY(0x1, 0x011),
  { 0 }
};

static DEV_INIT(gptimer_init);
static DEV_CLEANUP(gptimer_cleanup);

const struct driver_s  gptimer_drv =
{
  .desc           = "Gaisler GPTIMER",
  .id_table       = gptimer_ids,
  .f_init         = gptimer_init,
  .f_cleanup      = gptimer_cleanup,

  .classes        = {
    &gptimer_timer_drv,
    0
  }
};


static DEV_INIT(gptimer_init)
{
  struct gptimer_private_s  *pv;
  uint_fast8_t i;
  uintptr_t addr;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -ENOENT;

  uint32_t cfg = endian_be32(cpu_mem_read_32(addr + TIMER_REG_CFG));
  uint_fast8_t t_count = cfg & TIMER_CFG_NTIMERS;
#ifdef CONFIG_DEVICE_IRQ
  uint_fast8_t irq_count = cfg & TIMER_CFG_SIRQ ? t_count : 1;
#endif

  pv = mem_alloc(sizeof (*pv) + t_count * (sizeof(struct gptimer_state_s)
#ifdef CONFIG_DEVICE_IRQ
                              + irq_count * sizeof(struct dev_irq_ep_s)
#endif
                                           ), (mem_scope_sys));
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));
  pv->t_count = t_count;
  dev->drv_pv = pv;
  pv->addr = addr;

  cpu_mem_write_32(addr + TIMER_REG_SC_RELOAD, endian_be32(CONFIG_DRIVER_GAISLER_GPTIMER_PRESCALE - 1));

#ifdef CONFIG_DEVICE_IRQ
  uint32_t sc = endian_be32(cpu_mem_read_32(pv->addr + TIMER_REG_SC_RELOAD)) + 1;
  dev_timer_res_t resolution = 100000;
  device_get_param_uint(dev, "period", &resolution);
  resolution /= sc;
  if (resolution < 1)
    resolution = 1;

  pv->irq_eps = (void*)(pv->t + t_count);

  if (cfg & TIMER_CFG_SIRQ)
    device_irq_source_init(dev, pv->irq_eps, irq_count,
                           gptimer_irq_separate, DEV_IRQ_SENSE_RISING_EDGE);
  else
    device_irq_source_init(dev, pv->irq_eps, irq_count,
                           gptimer_irq_single, DEV_IRQ_SENSE_RISING_EDGE);

  if (device_irq_source_link(dev, pv->irq_eps, irq_count, -1))
    goto err_mem;
#endif

  for (i = 0; i < t_count; i++)
    {
      struct gptimer_state_s *p = pv->t + i;
      p->start_count = 0;

# ifdef CONFIG_DEVICE_IRQ
      p->value = 0;
      dev_timer_queue_init(&p->queue);
      p->period = resolution;
      cpu_mem_write_32(TIMER_REG_ADDR(addr, TIMER_REG_CTRL, i),
                       endian_be32(TIMER_CTRL_IP | TIMER_CTRL_IE));
# else
      cpu_mem_write_32(TIMER_REG_ADDR(addr, TIMER_REG_COUNTER, i), 0xffffffff);
      cpu_mem_write_32(TIMER_REG_ADDR(addr, TIMER_REG_RELOAD, i), 0xffffffff);
      cpu_mem_write_32(TIMER_REG_ADDR(addr, TIMER_REG_CTRL, i),
                       endian_be32(TIMER_CTRL_ENABLED | TIMER_CTRL_RESTART));
# endif
    }

  dev->drv = &gptimer_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;
  return 0;

# ifdef CONFIG_DEVICE_IRQ
 err_mem:
  mem_free(pv);
  return -1;
#endif
}

static DEV_CLEANUP(gptimer_cleanup)
{
  struct gptimer_private_s *pv = dev->drv_pv;

  uint_fast8_t i;
  for (i = 0; i < pv->t_count; i++)
    {
      cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_REG_CTRL, i), 0);
#ifdef CONFIG_DEVICE_IRQ
      struct gptimer_state_s *p = pv->t + i;
      dev_timer_queue_destroy(&p->queue);
#endif
    }

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_unlink(dev, pv->irq_eps, pv->t_count);
#endif

  mem_free(pv);
}

REGISTER_DRIVER(gptimer_drv);

