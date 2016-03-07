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

#include "xicu_private.h"

#include <string.h>
#include <stdio.h>

#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <device/class/timer.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>
#include <mutek/kroutine.h>

/*
  Each hardware timer have two different timer interfaces with
  different modes. Hardware timer 0 has interface 0 and 1, timer 1 has
  interface 2 and 3, and so on. The two modes work this way:

  - Timer driver interfaces with an even number provide exposes the 32
    bit value of the free running counter directly. This interface is
    not able to handle requests.

  - Timer driver interfaces with an odd number provide a 64 bits
    software timer which use the 32 bits hardware counter as a
    prescaler. This interface is able to handle requests.

  Timer start can be performed on a single interface of a pair at the
  same time.
*/

#ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
void soclib_xicu_pti_irq_process(struct device_s *dev, uint_fast8_t number)
{
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  struct soclib_xicu_pti_s *p = pv->pti + number;

  assert (number < pv->pti_count);
  lock_spin(&dev->lock);

  p->value++;

  while (1)
    {
      struct dev_timer_rq_s *rq;
      rq = dev_timer_rq_s_cast(dev_request_pqueue_head(&p->queue));

      if (!rq)
        {
          /* stop timer if not in use */
          p->start_count &= ~1;
          if (p->start_count == 0)
            cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, number), 0);
          break;
        }

      assert(p->start_count & 1);

      if (rq->deadline > p->value)
        break;

      rq->rq.drvdata = 0;
      dev_timer_pqueue_remove(&p->queue, dev_timer_rq_s_base(rq));

      lock_release(&dev->lock);
      kroutine_exec(&rq->rq.kr);
      lock_spin(&dev->lock);
    }

  lock_release(&dev->lock);
}
#endif

DEV_TIMER_REQUEST(soclib_xicu_timer_request)
{
#ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
  struct device_s *dev = accessor->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  error_t err = 0;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;

  if (mode == 0)
    return -ENOTSUP;

  if (number >= pv->pti_count)
    return -ENOTSUP;

  struct soclib_xicu_pti_s *p = pv->pti + number;

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
              cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_VAL, number), endian_le32(p->period));
              cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, number), endian_le32(p->period));
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

DEV_TIMER_CANCEL(soclib_xicu_timer_cancel)
{
#ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
  struct device_s *dev = accessor->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  error_t err = 0;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;

  if (mode == 0)
    return -ENOTSUP;

  if (number >= pv->pti_count)
    return -ENOTSUP;

  if (!rq)
    return 0;

  struct soclib_xicu_pti_s *p = pv->pti + number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->rq.drvdata == p)
    {
      assert(p->start_count & 1);

      dev_timer_pqueue_remove(&p->queue, dev_timer_rq_s_base(rq));
      rq->rq.drvdata = 0;

      /* stop timer if not in use */
      if (dev_request_pqueue_isempty(&p->queue))
        {
          p->start_count &= ~1;
          if (p->start_count == 0)
            cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, number), 0);
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

error_t soclib_xicu_timer_use(struct device_accessor_s *accessor, enum dev_use_op_e op)
{
  struct device_s *dev = accessor->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;

#ifndef CONFIG_DRIVER_SOCLIB_XICU_ICU
  if (mode != 0)
    return -ENOTSUP;
#endif

  error_t err = 0;
  struct soclib_xicu_pti_s *p = pv->pti + number;

  if (p->start_count && ((p->start_count > 0) ^ mode))
    {
      /* timer already used in the other mode */
      err = -EBUSY;
    }
  else if (op == DEV_USE_START)
    {
      if (p->start_count == 0)
        {
# ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
          if (mode)
            {
              cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_VAL, number), endian_le32(p->period));
              cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, number), endian_le32(p->period));
            }
          else
#endif
            {
              cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_VAL, number), 0xffffffff);
              cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, number), 0xffffffff);
            }
        }
      p->start_count += mode ? 2 : -2;
    }
  else        /* DEV_USE_STOP */
    {
      if ((p->start_count & ~1) == 0)
        err = -EINVAL;
      else
        {
          p->start_count -= mode ? 2 : -2;
          if (p->start_count == 0)
            cpu_mem_write_32(XICU_REG_ADDR(pv->addr, XICU_PTI_PER, number), 0);
        }
    }

  return err;
}

DEV_TIMER_GET_VALUE(soclib_xicu_timer_get_value)
{
  struct device_s *dev = accessor->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;
  error_t err = 0;

  if (number >= pv->pti_count)
    return -ENOTSUP;

  LOCK_SPIN_IRQ(&dev->lock);

  struct soclib_xicu_pti_s *p = pv->pti + number;

  if (mode)
    {
#ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
      if (rev && rev != p->rev)
        err = -EAGAIN;
      else
        *value = p->value;
#else
      err = -ENOTSUP;
#endif
    }
  else if (p->start_count < 0)
    {
      if (rev && rev != 1)
        err = -EAGAIN;
      else
        *value = ~endian_le32(cpu_mem_read_32(XICU_REG_ADDR(pv->addr, XICU_PTI_VAL, number)));
    }
  else
    err = -EBUSY; /* timer not started in right mode */

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

DEV_TIMER_CONFIG(soclib_xicu_timer_config)
{
  struct device_s *dev = accessor->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  uint_fast8_t number = accessor->number / 2;
  uint_fast8_t mode = accessor->number % 2;

  if (number >= pv->pti_count)
    return -ENOTSUP;

  error_t err = 0;

  if (cfg && device_get_res_freq(accessor->dev, &cfg->freq, number))
    cfg->freq = DEV_FREQ_INVALID;

  LOCK_SPIN_IRQ(&dev->lock);

  if (mode)
    {
#ifdef CONFIG_DRIVER_SOCLIB_XICU_ICU
      struct soclib_xicu_pti_s *p = pv->pti + number;

      if (res)
        {
          if (p->start_count)
            {
              err = -EBUSY;
            }
          else
            {
              if (res < SOCLIB_XICU_PTI_MIN_PERIOD)
                {
                  res = SOCLIB_XICU_PTI_MIN_PERIOD;
                  err = -ERANGE;
                }
              p->rev += 2;
              p->period = res;
            }
        }

      if (cfg)
        {
          cfg->max = 0xffffffffffffffffULL;
          cfg->rev = p->rev;
          cfg->res = p->period;
          cfg->cap = DEV_TIMER_CAP_REQUEST | DEV_TIMER_CAP_STOPPABLE
            | DEV_TIMER_CAP_KEEPVALUE;
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
          cfg->max = 0xffffffff;
          cfg->rev = 1;
          cfg->res = 1;
          cfg->cap = DEV_TIMER_CAP_STOPPABLE
            | DEV_TIMER_CAP_HIGHRES | DEV_TIMER_CAP_TICKLESS;
        }
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

