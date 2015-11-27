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
#include <enums.h>

const char dev_timer_capabilities_e[] = ENUM_DESC_DEV_TIMER_CAPABILITIES_E;

GCT_CONTAINER_KEY_PROTOTYPES(dev_request_pqueue, extern inline, dev_timer_pqueue, dev_timer_pqueue,
                             remove, insert);

extern inline error_t dev_timer_frac(struct device_timer_s *accessor,
                                     uint64_t *num, uint64_t *denom,
                                     dev_timer_cfgrev_t *rev, bool_t reduce)
{
  struct dev_timer_config_s cfg;
  error_t err;

  err = DEVICE_OP(accessor, config, &cfg, 0);
  if (err)
    return err;
  if (!DEV_FREQ_IS_VALID(cfg.freq))
    return -EIO;

  uint64_t n = *num * cfg.freq.num;
  uint64_t d = *denom * cfg.res * cfg.freq.denom;

  if (reduce)
    {
      uint64_t gcd = gcd64(n, d);
      n /= gcd;
      d /= gcd;
    }

  *num = n;
  *denom = d;

  if (rev)
    *rev = cfg.rev;

  return 0;
}

error_t dev_timer_init_sec(struct device_timer_s *accessor, dev_timer_delay_t *delay,
                           dev_timer_cfgrev_t *rev, dev_timer_delay_t s_delay, uint32_t r_unit)
{
  uint64_t num = s_delay, denom = r_unit;
  error_t err = dev_timer_frac(accessor, &num, &denom, rev, 0);
  if (err)
    return err;

  uint64_t d = num / denom;
  if (d != (dev_timer_delay_t)d)
    return -ERANGE;

  *delay = d ? d : 1;

  return 0;
}

error_t dev_timer_init_sec_round(struct device_timer_s *accessor, dev_timer_delay_t *delay,
                                 dev_timer_cfgrev_t *rev, dev_timer_delay_t s_delay, uint32_t r_unit)
{
  uint64_t num = s_delay, denom = r_unit;
  error_t err = dev_timer_frac(accessor, &num, &denom, rev, 0);
  if (err)
    return err;

  num += denom / 2;

  uint64_t d = num / denom;
  if (d != (dev_timer_delay_t)d)
    return -ERANGE;

  *delay = d ? d : 1;

  return 0;
}

error_t dev_timer_init_sec_ceil(struct device_timer_s *accessor, dev_timer_delay_t *delay,
                                dev_timer_cfgrev_t *rev, dev_timer_delay_t s_delay, uint32_t r_unit)
{
  uint64_t num = s_delay, denom = r_unit;
  error_t err = dev_timer_frac(accessor, &num, &denom, rev, 0);
  if (err)
    return err;

  num += denom - 1;

  uint64_t d = num / denom;
  if (d != (dev_timer_delay_t)d)
    return -ERANGE;

  *delay = d ? d : 1;

  return 0;
}

error_t dev_timer_get_sec(struct device_timer_s *accessor, dev_timer_delay_t *delay,
                          dev_timer_cfgrev_t *rev, dev_timer_delay_t s_delay, uint32_t r_unit)
{
  struct dev_timer_config_s cfg;
  error_t err;

  err = DEVICE_OP(accessor, config, &cfg, 0);
  if (err)
    return err;
  if (!DEV_FREQ_IS_VALID(cfg.freq))
    return -EIO;

  uint64_t d = ((uint64_t)r_unit * cfg.res * cfg.freq.denom * s_delay)
    / ((uint64_t)cfg.freq.num);

  if (d > (uint64_t)(dev_timer_delay_t)-1)
    return -ERANGE;

  *delay = d ? d : 1;
  if (rev)
    *rev = cfg.rev;

  return 0;
}

error_t dev_timer_shift_sec(struct device_timer_s *accessor,
                            int8_t *shift_a, int8_t *shift_b,
                            dev_timer_cfgrev_t *rev,
                            dev_timer_delay_t s_delay, uint32_t r_unit)
{
  struct dev_timer_config_s cfg;
  error_t err;

  err = DEVICE_OP(accessor, config, &cfg, 0);
  if (err)
    return err;
  if (!DEV_FREQ_IS_VALID(cfg.freq))
    return -EIO;

  uint64_t a = ((uint64_t)cfg.freq.num * s_delay) / cfg.freq.denom;
  uint64_t b = (uint64_t)r_unit * cfg.res;

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

  if (rev)
    *rev = cfg.rev;

  return 0;
}

error_t dev_timer_check_timeout(struct device_timer_s *accessor,
                                dev_timer_delay_t delay,
                                const dev_timer_value_t *start)
{
  struct dev_timer_config_s cfg;
  error_t err;

  err = DEVICE_OP(accessor, config, &cfg, 0);
  if (err)
    return err;

  // compute wrapping mask
  dev_timer_value_t b = (cfg.max >> 1) + 1;
  if (delay >= b)
    return -ERANGE;

  dev_timer_value_t v;
  if (DEVICE_OP(accessor, get_value, &v, cfg.rev))
    return -EIO;

  return ((((*start + delay) & cfg.max) - v) & b) != 0;
}

error_t dev_timer_busy_wait(struct device_timer_s *accessor, dev_timer_delay_t delay)
{
  // get max timer value (power of 2 minus 1)
  struct dev_timer_config_s cfg;
  error_t err;

  err = DEVICE_OP(accessor, config, &cfg, 0);
  if (err)
    return err;

  // compute wrapping mask
  dev_timer_value_t b = (cfg.max >> 1) + 1;
  if (delay >= b)
    return -ERANGE;

  if (cfg.cap & DEV_TIMER_CAP_STOPPABLE)
    if (device_start(accessor))
      return -EBUSY;

  // compute wrapped deadline
  dev_timer_value_t d;
  if (DEVICE_OP(accessor, get_value, &d, cfg.rev))
    {
      err = -EIO;
      goto stop;
    }

  d = (d + delay) & cfg.max;

  while (1)
    {
      dev_timer_value_t v;
      if (DEVICE_OP(accessor, get_value, &v, cfg.rev))
        {
          err = -EIO;
          goto stop;
        }

      // x will wrap when the deadline is reached
      if ((d - v) & b)
        break;
    }

  err = 0;
 stop:
  if (cfg.cap & DEV_TIMER_CAP_STOPPABLE)
    device_stop(accessor);

  return err;
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
  struct dev_timer_rq_s *rq = dev_timer_rq_s_cast(dev_request_s_from_kr(kr));
  struct dev_timer_wait_rq_s *status = rq->rq.pvdata;

  LOCK_SPIN_IRQ(&status->lock);
  if (status->ctx != NULL)
    sched_context_start(status->ctx);
  status->done = 1;
  LOCK_RELEASE_IRQ(&status->lock);
}
#endif

#ifdef CONFIG_MUTEK_SCHEDULER
error_t dev_timer_sleep(struct device_timer_s *accessor, struct dev_timer_rq_s *rq)
{
  struct dev_timer_wait_rq_s status;

  if (rq->delay == 0)
    return 0;

  lock_init(&status.lock);
  status.ctx = NULL;
  status.done = 0;
  rq->rq.pvdata = &status;
  kroutine_init(&rq->rq.kr, dev_timer_wait_request_cb, KROUTINE_IMMEDIATE);

  error_t e = DEVICE_OP(accessor, request, rq);

  switch (e)
    {
    case -ETIMEDOUT:
      e = 0;
      break;

    case 0:
      CPU_INTERRUPT_SAVESTATE_DISABLE;
      lock_spin(&status.lock);

      if (!status.done)
        {
          status.ctx = sched_get_current();
          sched_stop_unlock(&status.lock);
        }
      else
        lock_release(&status.lock);

      assert(!rq->rq.drvdata);

      CPU_INTERRUPT_RESTORESTATE;
      lock_destroy(&status.lock);

    default:
      break;
    }

  lock_destroy(&status.lock);

  return e;
}
#endif

extern inline
error_t dev_timer_wait_deadline(struct device_timer_s *accessor, dev_timer_value_t deadline);
