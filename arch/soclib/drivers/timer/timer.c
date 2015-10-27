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

  Each hardware timer have two different timer interfaces with
  different modes. Hardware timer 0 has interface 0 and 1, timer 1 has
  interface 2 and 3, and so on. The two modes work this way:

  - Timer driver interfaces with an even number provide exposes the
    value of the free running counter directly. The value is 32 bits
    if irq support is disabled and 64 bits if irq support is
    enabled. This interface is not able to handle requests.

  - Timer driver interfaces with an odd number provide a 64 bits
    software timer which use the 32 bits hardware counter as a
    prescaler. This interface is able to handle requests.

  Timer start can be performed on a single interface of a pair at the
  same time.

*/

struct soclib_timer_state_s
{
  /* bit 0 indicates if some requests are using the timer, other bits
     are start count. The start count is positive if the timer has
     been started in mode 1 and negative if the timer has been started
     in mode 0. */
  int_fast8_t start_count;

#ifdef CONFIG_DEVICE_IRQ
  dev_request_pqueue_root_t queue;
  dev_timer_value_t value;
  dev_timer_res_t   period;
  dev_timer_cfgrev_t rev;
#endif
};

struct soclib_timer_private_s
{
  uintptr_t addr;
  uintptr_t t_count;
#ifdef CONFIG_DEVICE_IRQ
  struct dev_irq_src_s *irq_eps;
#endif
  struct soclib_timer_state_s t[0];
};

#ifdef CONFIG_DEVICE_IRQ
static DEV_IRQ_SRC_PROCESS(soclib_timer_irq)
{
  struct device_s *dev = ep->base.dev;
  struct soclib_timer_private_s *pv = dev->drv_pv;
  uint_fast8_t number = ep - pv->irq_eps;

  assert(number < pv->t_count);
  lock_spin(&dev->lock);

  while (1)
    {
      if (!cpu_mem_read_32(TIMER_REG_ADDR(pv->addr, TIMER_IRQ, number)))
        break;

      cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_IRQ, number), 0);

      struct soclib_timer_state_s *p = pv->t + number;

      p->value++;

      if (p->start_count <= 0)
        break;

      while (1)
        {
          struct dev_timer_rq_s *rq;
          rq = dev_timer_rq_s_cast(dev_request_pqueue_head(&p->queue));

          if (!rq)
            {
              /* stop timer */
              p->start_count &= ~1;
              if (p->start_count == 0)
                cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, number), 0);
              break;
            }

          assert(p->start_count & 1);

          if (rq->deadline > p->value)
            break;

          rq->rq.drvdata = NULL;
          dev_timer_pqueue_remove(&p->queue, dev_timer_rq_s_base(rq));

          lock_release(&dev->lock);
          kroutine_exec(&rq->rq.kr);
          lock_spin(&dev->lock);
        }
    }

  lock_release(&dev->lock);
}
#endif

static DEV_TIMER_REQUEST(soclib_timer_request)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct soclib_timer_private_s *pv = dev->drv_pv;
  error_t err = 0;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;

  if (mode == 0)
    return -ENOTSUP;

  if (number >= pv->t_count)
    return -ENOTSUP;

  struct soclib_timer_state_s *p = pv->t + number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (p->start_count < 0)  /* hardware timer already used in mode 0 */
    err = -EBUSY;
  else if (rq->rev && rq->rev != p->rev)
    err = -EAGAIN;
  else
    {
      uint64_t val = p->value;

      if (rq->delay)
        rq->deadline = val + rq->delay;

      if (rq->deadline <= val)
        err = -ETIMEDOUT;
      else
        {
          rq->rq.drvdata = p;
          dev_timer_pqueue_insert(&p->queue, dev_timer_rq_s_base(rq));

          /* start timer if needed */
          if (p->start_count == 0)
            {
              cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_VALUE, number), 0);
              cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_PERIOD, number), endian_le32(p->period));
              cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, number), endian_le32(TIMER_MODE_EN | TIMER_MODE_IRQEN));
            }
          p->start_count |= 1;
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEV_TIMER_CANCEL(soclib_timer_cancel)
{
#ifdef CONFIG_DEVICE_IRQ
  struct device_s *dev = accessor->dev;
  struct soclib_timer_private_s *pv = dev->drv_pv;
  error_t err = 0;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;

  if (mode == 0)
    return -ENOTSUP;

  if (number >= pv->t_count)
    return -ENOTSUP;

  struct soclib_timer_state_s *p = pv->t + number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rq.drvdata == p)
    {
      assert(p->start_count & 1);

      dev_timer_pqueue_remove(&p->queue, dev_timer_rq_s_base(rq));
      rq->rq.drvdata = NULL;

      /* stop timer if not in use */
      if (dev_request_pqueue_isempty(&p->queue))
        {
          p->start_count &= ~1;
          if (p->start_count == 0)
            cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, number), 0);
        }
    }
  else
    {
      err = -ETIMEDOUT;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
#else
  return -ENOTSUP;
#endif
}

static DEV_USE(soclib_timer_use)
{
  struct device_s *dev = accessor->dev;
  struct soclib_timer_private_s *pv = dev->drv_pv;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;
  bool_t start = 0;

  if (number >= pv->t_count)
    return -ENOTSUP;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR:
    case DEV_USE_PUT_ACCESSOR:
      return 0;
    case DEV_USE_START:
      start = 1;
    case DEV_USE_STOP:
      break;
    }

#ifndef CONFIG_DEVICE_IRQ
  if (mode != 0)
    return -ENOTSUP;
#endif

  error_t err = 0;

  struct soclib_timer_state_s *p = pv->t + number;
  int_fast8_t st = mode ? 2 : -2;

  LOCK_SPIN_IRQ(&dev->lock);

  if (p->start_count && ((p->start_count > 0) ^ mode))
    {
      /* timer already used in the other mode */
      err = -EBUSY;
    }
  else if (start)
    {
      if (p->start_count == 0)
        {
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_VALUE, number), 0);
#ifdef CONFIG_DEVICE_IRQ
          p->value = 0;
          if (mode)
            cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_PERIOD, number), endian_le32(p->period));
          else
#endif
            cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_PERIOD, number), 0xffffffff);
#ifdef CONFIG_DEVICE_IRQ
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, number), endian_le32(TIMER_MODE_EN | TIMER_MODE_IRQEN));
#else
          cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, number), endian_le32(TIMER_MODE_EN));
#endif
        }
      p->start_count += st;
    }
  else
    {
      if ((p->start_count & ~1) == 0)
        err = -EINVAL;
      else
        {
          p->start_count -= st;
          if (p->start_count == 0)
            cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, number), 0);
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_GET_VALUE(soclib_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct soclib_timer_private_s *pv = dev->drv_pv;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;
  error_t err = 0;

  if (number >= pv->t_count)
    return -ENOTSUP;

  LOCK_SPIN_IRQ(&dev->lock);

  struct soclib_timer_state_s *p = pv->t + number;

  if (!p->start_count || ((p->start_count > 0) ^ mode))
    {
      /* timer not started in this mode */
      err = -EBUSY;
    }
  else if (mode)
    {
#ifdef CONFIG_DEVICE_IRQ
      if (rev && rev != p->rev)
        err = -EAGAIN;
      else
        *value = p->value;
#else
      err = -ENOTSUP;
#endif
    }
  else
    {
      uint64_t v = endian_le32(cpu_mem_read_32(TIMER_REG_ADDR(pv->addr, TIMER_VALUE, number)));

#ifdef CONFIG_DEVICE_IRQ
      if (v < 0x80000000 && cpu_mem_read_32(TIMER_REG_ADDR(pv->addr, TIMER_IRQ, number)))
        v += 0x100000000ULL;
      v += p->value << 32;
#endif

      if (rev && rev != 1)
        err = -EAGAIN;
      else
        *value = v;
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEV_TIMER_CONFIG(soclib_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct soclib_timer_private_s *pv = dev->drv_pv;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;

  if (number >= pv->t_count)
    return -ENOTSUP;

  error_t err = 0;

  if (cfg && device_get_res_freq(accessor->dev, &cfg->freq, number))
    cfg->freq = DEV_FREQ_INVALID;

  LOCK_SPIN_IRQ(&dev->lock);

  if (mode)
    {
#ifdef CONFIG_DEVICE_IRQ
      struct soclib_timer_state_s *p = pv->t + number;

      if (res)
        {
          if (p->start_count)
            {
              err = -EBUSY;
            }
          else
            {
              if (res < SOCLIB_TIMER_MIN_PERIOD)
                {
                  res = SOCLIB_TIMER_MIN_PERIOD;
                  err = -ERANGE;
                }
              p->rev += 2;
              p->period = res;
            }
        }

      if (cfg)
        {
          cfg->acc = DEV_FREQ_ACC_INVALID;
          cfg->max = 0xffffffffffffffffULL;
          cfg->rev = p->rev;
          cfg->res = p->period;
          cfg->cap = DEV_TIMER_CAP_REQUEST | DEV_TIMER_CAP_STOPPABLE;
        }

#else
      err = -ENOTSUP;
#endif
    }
  else
    {
      if (res > 1)
        err = -ERANGE;

      if (cfg)
        {
          cfg->acc = DEV_FREQ_ACC_INVALID;
          cfg->cap = DEV_TIMER_CAP_STOPPABLE | DEV_TIMER_CAP_HIGHRES;
#ifdef CONFIG_DEVICE_IRQ
          cfg->max = 0xffffffffffffffffULL;
#else
          cfg->cap |= DEV_TIMER_CAP_TICKLESS;
          cfg->max = 0xffffffff;
#endif
          cfg->rev = 1;
          cfg->res = 1;
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

/************************************************************************/

static DEV_INIT(soclib_timer_init);
static DEV_CLEANUP(soclib_timer_cleanup);

DRIVER_DECLARE(soclib_timer_drv, 0, "Soclib Timer", soclib_timer,
               DRIVER_TIMER_METHODS(soclib_timer));

DRIVER_REGISTER(soclib_timer_drv,
                DEV_ENUM_FDTNAME_ENTRY("soclib:timer"));

static DEV_INIT(soclib_timer_init)
{
  struct soclib_timer_private_s  *pv;
  uint_fast8_t i;

  uintptr_t t_count = 1;

  if (device_get_param_uint(dev, "count", &t_count))
    printk("warning: timer device `%p' has no `count' parameter, assuming only one timer is available.\n", dev);

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof (*pv) + t_count * (sizeof(struct soclib_timer_state_s)
#ifdef CONFIG_DEVICE_IRQ
                                           + sizeof(struct dev_irq_src_s)
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
  device_get_param_uint(dev, "resolution", &resolution);
  if (resolution < SOCLIB_TIMER_MIN_PERIOD)
    resolution = SOCLIB_TIMER_MIN_PERIOD;

  pv->irq_eps = (void*)(pv->t + t_count);

  device_irq_source_init(dev, pv->irq_eps, t_count,
                         soclib_timer_irq);

  if (device_irq_source_link(dev, pv->irq_eps, t_count, -1))
    goto err_mem;
#endif

  for (i = 0; i < t_count; i++)
    {
      struct soclib_timer_state_s *p = pv->t + i;
      p->start_count = 0;

      cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_MODE, i), 0);

# ifdef CONFIG_DEVICE_IRQ
      p->rev = 1;
      p->value = 0;
      dev_request_pqueue_init(&p->queue);
      p->period = resolution;
      cpu_mem_write_32(TIMER_REG_ADDR(pv->addr, TIMER_IRQ, i), 0);
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
      dev_request_pqueue_destroy(&p->queue);
#endif
    }

#ifdef CONFIG_DEVICE_IRQ
  device_irq_source_unlink(dev, pv->irq_eps, pv->t_count);
#endif

  mem_free(pv);
}

