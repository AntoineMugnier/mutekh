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

    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014
    Copyright Sébastien Cerdan <scerdan@gmail.com> (c) 2014

*/

#include <assert.h>
#include <string.h>

#include <hexo/lock.h>

#include <device/class/pwm.h>

const char dev_pwm_polarity_e[] = ENUM_DESC_DEV_PWM_POLARITY_E;
const char dev_pwm_mode_e[] = ENUM_DESC_DEV_PWM_MODE_E;

GCT_CONTAINER_PROTOTYPES(dev_pwm_queue, extern inline, dev_pwm_queue,
                   init, destroy, isempty, head, remove, push, pushback);

error_t dev_pwm_rq_queue_init(struct dev_pwm_rq_queue_s *q)
{
    dev_pwm_queue_init(&q->queue);
    lock_init_irq(&q->lock);
    return 0;
}

void dev_pwm_rq_queue_cleanup(struct dev_pwm_rq_queue_s *q)
{
    lock_destroy_irq(&q->lock);
    dev_pwm_queue_destroy(&q->queue);
}

error_t dev_pwm_request_init(struct device_pwm_s      *pdev,
                             struct dev_pwm_request_s *rq)
{
    memset(rq, 0, sizeof(*rq));
    rq->pdev  = pdev;
    rq->queue = DEVICE_OP(pdev, queue);
    return 0;
}


static
KROUTINE_EXEC(dev_pwm_config_end)
{
  struct dev_pwm_config_s *cfg = KROUTINE_CONTAINER(kr, *cfg, kr);
  struct dev_pwm_request_s *rq = cfg->pvdata;
  struct dev_pwm_rq_queue_s *q = rq->queue;

  lock_spin_irq(&q->lock);

  dev_pwm_queue_remove(&q->queue, rq);

  /* if the request cannot be applied now, but possibly later, then push
     the request again for a new try. */
  if (cfg->error == -EBUSY)
    {
      dev_pwm_queue_pushback(&q->queue, rq);
      lock_release_irq(&q->lock);
    }

  /* otherwise, call the kroutine and rearm. */
  else
    {
      /* update the marker so that infinite look-up is prevented. */
      if (rq == q->marker)
          q->marker = dev_pwm_queue_isempty(&q->queue) ?
            NULL : dev_pwm_queue_head(&q->queue);

      rq->error = cfg->error;

      lock_release_irq(&q->lock);
      kroutine_exec(&rq->kr, cpu_is_interruptible());

      /* if other requests are pending, then execute them. */
      if (kroutine_triggered_1st(kr))
        {
          lock_spin_irq(&q->lock);
          dev_pwm_execute(q);
        }
    }
}

void dev_pwm_execute(struct dev_pwm_rq_queue_s *q)
{
    while (!dev_pwm_queue_isempty(&q->queue) &&
           q->marker != dev_pwm_queue_head(&q->queue));
      {
        /* apply the next configuration from the queue. */
        struct dev_pwm_request_s *rq  = dev_pwm_queue_head(&q->queue);
        struct dev_pwm_config_s  *cfg = rq->cfg;
        assert(cfg != NULL);

        kroutine_init(&cfg->kr, &dev_pwm_config_end, KROUTINE_TRIGGER);
        cfg->pvdata = rq;

        /* call the driver. */
        lock_release_irq(&q->lock);
        DEVICE_OP(rq->pdev, config, cfg);

        /* if the kroutine_exec is not already called, return. */
        if (!kroutine_trigger(&cfg->kr, 0))
          return;
        lock_spin_irq(&q->lock);
      }
}

error_t dev_pwm_request_start(struct dev_pwm_request_s *rq)
{
  struct dev_pwm_rq_queue_s *q = rq->queue;

  rq->cfg->pdev = rq->pdev;

  lock_spin_irq(&q->lock);

  rq->error = 0;
  bool_t empty = dev_pwm_queue_isempty(&q->queue);

  if (rq->priority)
    dev_pwm_queue_push(&q->queue, rq);
  else
    dev_pwm_queue_pushback(&q->queue, rq);

  if (empty)
    {
      q->marker = rq;
      dev_pwm_execute(q);
    }

  lock_release_irq(&q->lock);

  return 0;
}

#if defined(CONFIG_MUTEK_SCHEDULER)

#include <mutek/scheduler.h>
#include <hexo/lock.h>

struct dev_pwm_wait_rq_s
{
  lock_t                 lock;
  struct sched_context_s *ctx;
  bool_t                 done:1;
};

static
KROUTINE_EXEC(dev_pwm_wait_request)
{
  struct dev_pwm_request_s *rq     = KROUTINE_CONTAINER(kr, *rq, kr);
  struct dev_pwm_wait_rq_s *status = rq->pvdata;

  lock_spin(&status->lock);
  if (status->ctx != NULL)
    sched_context_start(status->ctx);
  status->done = 1;
  lock_release(&status->lock);
}

error_t dev_pwm_config(struct device_pwm_s     *pdev,
                       struct dev_pwm_config_s *cfg)
{
  struct dev_pwm_request_s rq;
  struct dev_pwm_wait_rq_s status;


  lock_init(&status.lock);
  status.ctx  = NULL;
  status.done = 0;

  dev_pwm_request_init(pdev, &rq);
  kroutine_init(&rq.kr, &dev_pwm_wait_request, KROUTINE_IMMEDIATE);
  rq.pvdata = &status;
  rq.cfg    = cfg;

  dev_pwm_request_start(&rq);

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&status.lock);

  if (!status.done)
    {
      status.ctx = sched_get_current();
      sched_stop_unlock(&status.lock);
    }
  else
    lock_release(&status.lock);

  CPU_INTERRUPT_RESTORESTATE;
  lock_destroy(&status.lock);

  return rq.error;
}

#endif

