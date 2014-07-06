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

#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
void soclib_xicu_pti_irq_process(struct device_s *dev, uint_fast8_t number)
{
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  struct soclib_xicu_pti_s *p = pv->pti + number;

  assert (number < pv->pti_count);
  lock_spin(&dev->lock);

  p->value++;

  while (1)
    {
      struct dev_timer_rq_s *rq = dev_timer_queue_head(&p->queue);

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

      rq->drvdata = 0;
      dev_timer_queue_pop(&p->queue);

      lock_release(&dev->lock);
      kroutine_exec(&rq->kr, 0);
      lock_spin(&dev->lock);
    }

  lock_release(&dev->lock);
}
#endif

static DEVTIMER_REQUEST(soclib_xicu_timer_request)
{
#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  struct device_s *dev = tdev->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  error_t err = 0;
  uint_fast8_t number = tdev->number / 2;
  uint_fast8_t mode = tdev->number % 2;

  if (mode == 0)
    return -ENOTSUP;

  if (number >= pv->pti_count)
    return -ENOENT;

  rq->tdev = tdev;

  struct soclib_xicu_pti_s *p = pv->pti + number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (p->start_count < 0)  /* hardware timer already used in mode 0 */
    err = -EBUSY;
  else
    {
      uint64_t val = p->value;

      if (rq->delay)
        rq->deadline = val + rq->delay;

      if (rq->deadline <= val)
        err = ETIMEDOUT;
      else
        {
          rq->drvdata = p;
          dev_timer_queue_insert_ascend(&p->queue, rq);

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

static DEVTIMER_CANCEL(soclib_xicu_timer_cancel)
{
#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  struct device_s *dev = tdev->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  error_t err = 0;
  uint_fast8_t number = tdev->number / 2;
  uint_fast8_t mode = tdev->number % 2;

  if (mode == 0)
    return -ENOTSUP;

  if (number >= pv->pti_count)
    return -ENOENT;

  if (!rq)
    return 0;

  assert(rq->tdev->dev == dev && rq->tdev->number == tdev->number);

  struct soclib_xicu_pti_s *p = pv->pti + number;

  LOCK_SPIN_IRQ(&dev->lock);

  if (rq->drvdata == p)
    {
      assert(p->start_count & 1);

      dev_timer_queue_remove(&p->queue, rq);
      rq->drvdata = 0;

      /* stop timer if not in use */
      if (dev_timer_queue_isempty(&p->queue))
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

static DEVTIMER_START_STOP(soclib_xicu_timer_start_stop)
{
  struct device_s *dev = tdev->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  uint_fast8_t number = tdev->number / 2;
  uint_fast8_t mode = tdev->number % 2;

  if (number >= pv->pti_count)
    return -ENOENT;

#ifndef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
  if (mode != 0)
    return -ENOTSUP;
#endif

  error_t err = 0;

  struct soclib_xicu_pti_s *p = pv->pti + number;

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
# ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
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
  else
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

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEVTIMER_GET_VALUE(soclib_xicu_timer_get_value)
{
  struct device_s *dev = tdev->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  uint_fast8_t number = tdev->number / 2;
  uint_fast8_t mode = tdev->number % 2;
  error_t err = 0;

  if (number >= pv->pti_count)
    return -ENOENT;

  LOCK_SPIN_IRQ(&dev->lock);

  struct soclib_xicu_pti_s *p = pv->pti + number;

  if (!p->start_count || ((p->start_count > 0) ^ mode))
    {
      /* timer not started in this mode */
      err = -EBUSY;
    }
  else if (mode)
    {
#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
      *value = p->value;
#else
      err = -ENOTSUP;
#endif
    }
  else
    {
      *value = ~endian_le32(cpu_mem_read_32(XICU_REG_ADDR(pv->addr, XICU_PTI_VAL, number)));
    }

  LOCK_RELEASE_IRQ(&dev->lock);

  return err;
}

static DEVTIMER_RESOLUTION(soclib_xicu_timer_resolution)
{
  struct device_s *dev = tdev->dev;
  struct soclib_xicu_private_s *pv = dev->drv_pv;
  uint_fast8_t number = tdev->number / 2;
  uint_fast8_t mode = tdev->number % 2;

  if (number >= pv->pti_count)
    return -ENOENT;

  error_t err = 0;

  if (mode)
    {
#ifdef CONFIG_DRIVER_SOCLIB_VCI_XICU_ICU
      struct soclib_xicu_pti_s *p = pv->pti + number;

      LOCK_SPIN_IRQ(&dev->lock);

      if (res)
        {
          if (*res)
            {
              if (p->start_count)
                {
                  err = -EBUSY;
                }
              else if (*res < SOCLIB_XICU_PTI_MIN_PERIOD)
                {
                  p->period = SOCLIB_XICU_PTI_MIN_PERIOD;
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
      err = -ENOTSUP;
#endif
    }
  else
    {
      if (res)
        {
          if (*res != 0)
            err = -ENOTSUP;
          *res = 1;
        }
      if (max)
        *max = 0xffffffff;
    }

  return err;
}

const struct driver_timer_s  soclib_xicu_timer_drv =
{
  .class_         = DRIVER_CLASS_TIMER,
  .f_request      = soclib_xicu_timer_request, 
  .f_cancel       = soclib_xicu_timer_cancel,
  .f_start_stop   = soclib_xicu_timer_start_stop,
  .f_get_value    = soclib_xicu_timer_get_value,
  .f_get_freq     = dev_timer_drv_get_freq,
  .f_resolution   = soclib_xicu_timer_resolution,
};

