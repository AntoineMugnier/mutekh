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

    Copyright Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr> (c) 2012

*/

#include <hexo/types.h>
#include <device/driver.h>
#include <device/device.h>
#include <device/resources.h>
#include <device/class/timer.h>

#ifdef CONFIG_MUTEK_SCHEDULER
# include <mutek/scheduler.h>
# include <hexo/lock.h>
#endif

#include <mutek/kroutine.h>
#include <stdlib.h> /* abs */

GCT_CONTAINER_KEY_PROTOTYPES(dev_timer_queue, extern inline, dev_timer_queue, deadline,
        init, destroy, isempty, pop, head, remove, insert);

error_t dev_timer_init_sec(struct device_timer_s *accessor, dev_timer_delay_t *delay,
                           dev_timer_delay_t s_delay, uint32_t r_unit)
{
  dev_timer_res_t r = 0;
  if (DEVICE_OP(accessor, resolution, &r, NULL))
    return -EIO;

  struct dev_freq_s f;
  if (DEVICE_OP(accessor, get_freq, &f))
    return -EIO;

  uint64_t d = f.num * s_delay / ((uint64_t)r_unit * r * f.denom);
  if (d > (uint64_t)(dev_timer_delay_t)-1)
    return -ERANGE;

  *delay = d ? d : 1;

  return 0;
}

error_t dev_timer_shift_sec(struct device_timer_s *accessor,
                            int8_t *shift_a, int8_t *shift_b,
                            dev_timer_delay_t s_delay, uint32_t r_unit)
{
  dev_timer_res_t r = 0;
  if (DEVICE_OP(accessor, resolution, &r, NULL))
    return -EIO;

  struct dev_freq_s f;
  if (DEVICE_OP(accessor, get_freq, &f))
    return -EIO;

  uint64_t a = f.num * s_delay / f.denom;
  uint64_t b = (uint64_t)r_unit * r;

  if (a == 0 || b == 0)
    return -ERANGE;

  uint_fast8_t as = __builtin_clzll(a);
  uint_fast8_t bs = __builtin_clzll(b);
  a <<= as;
  b <<= bs;

  int_fast8_t d = bs - as;
  if (shift_a != NULL)
    {
      *shift_a = d + (a > b);
      if (abs(*shift_a) >= sizeof(dev_timer_delay_t) * 8)
        return -ERANGE;
    }
  if (shift_b != NULL)
    {
      *shift_b = d - (a < b);
      if (abs(*shift_b) >= sizeof(dev_timer_delay_t) * 8)
        return -ERANGE;
    }

  return 0;
}

error_t dev_timer_check_timeout(struct device_timer_s *accessor,
                                dev_timer_delay_t delay,
                                const dev_timer_value_t *start)
{
  // get max timer value (power of 2 minus 1)
  dev_timer_value_t max;
  if (DEVICE_OP(accessor, resolution, NULL, &max))
    return -EIO;

  // compute wrapping mask
  dev_timer_value_t b = (max >> 1) + 1;
  if (delay >= b)
    return -ERANGE;

  dev_timer_value_t v;
  if (DEVICE_OP(accessor, get_value, &v))
    return -EIO;

  return ((((*start + delay) & max) - v) & b) != 0;
}

error_t dev_timer_busy_wait(struct device_timer_s *accessor, struct dev_timer_rq_s *rq)
{
  // get max timer value (power of 2 minus 1)
  dev_timer_value_t max;
  if (DEVICE_OP(accessor, resolution, NULL, &max))
    return -EIO;

  // compute wrapping mask
  dev_timer_value_t b = (max >> 1) + 1;
  if (rq->delay >= b)
    return -ERANGE;

  // compute wrapped deadline
  dev_timer_value_t d;
  if (DEVICE_OP(accessor, get_value, &d))
    return -EIO;

  d = (d + rq->delay) & max;

  DEVICE_OP(accessor, start_stop, 1);

  while (1)
    {
      dev_timer_value_t v;
      DEVICE_OP(accessor, get_value, &v);

      // x will wrap when the deadline is reached
      if ((d - v) & b)
        break;
    }

  DEVICE_OP(accessor, start_stop, 0);

  return 0;
}

struct dev_timer_wait_rq_s
{
#ifdef CONFIG_MUTEK_SCHEDULER
  lock_t lock;
  struct sched_context_s *ctx;
#endif
  bool_t done;
};

#ifdef CONFIG_MUTEK_SCHEDULER
static KROUTINE_EXEC(dev_timer_wait_request_cb)
{
  struct dev_timer_rq_s *rq = KROUTINE_CONTAINER(kr, *rq, kr);
  struct dev_timer_wait_rq_s *status = rq->pvdata;

  LOCK_SPIN_IRQ(&status->lock);
  if (status->ctx != NULL)
    sched_context_start(status->ctx);
  status->done = 1;
  LOCK_RELEASE_IRQ(&status->lock);
}
#endif

error_t dev_timer_sleep(struct device_timer_s *accessor, struct dev_timer_rq_s *rq)
{
#ifndef CONFIG_MUTEK_SCHEDULER
  return dev_timer_busy_wait(accessor, rq);
#else
  struct dev_timer_wait_rq_s status;

  if (rq->delay == 0)
    return 0;

  lock_init(&status.lock);
  status.ctx = NULL;
  status.done = 0;
  rq->pvdata = &status;
  kroutine_init(&rq->kr, dev_timer_wait_request_cb, KROUTINE_IMMEDIATE);

  error_t e = DEVICE_OP(accessor, request, rq);

  if (e == -ETIMEDOUT)
    {
    }
  else if (e < 0)
    {
      e = dev_timer_busy_wait(accessor, rq);
    }
  else
    {
      CPU_INTERRUPT_SAVESTATE_DISABLE;
      lock_spin(&status.lock);

      if (!status.done)
        {
          status.ctx = sched_get_current();
          sched_stop_unlock(&status.lock);
        }
      else
        lock_release(&status.lock);

      assert(!rq->drvdata);

      CPU_INTERRUPT_RESTORESTATE;
      lock_destroy(&status.lock);
    }

  lock_destroy(&status.lock);

  return e;
#endif
}

DEV_TIMER_GET_FREQ(dev_timer_drv_get_freq)
{
  if (device_get_res_freq(accessor->dev, freq, accessor->number))
    return -ENOTSUP;
  return 0;
}

