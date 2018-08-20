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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <mutek/scheduler.h>
#include <hexo/error.h>
#include <pthread.h>

error_t
pthread_cond_init(pthread_cond_t *cond,
		  const pthread_condattr_t *attr)
{
  return sched_queue_init(&cond->wait);
}

error_t
pthread_cond_destroy(pthread_cond_t *cond)
{
  sched_queue_destroy(&cond->wait);

  return 0;
}

error_t
pthread_cond_signal(pthread_cond_t *cond)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&cond->wait);
  __unused__ struct sched_context_s *sched_ctx;

  sched_ctx = sched_wake(&cond->wait);

#ifdef CONFIG_PTHREAD_COND_TIME
  if (sched_ctx)
  {
    struct pthread_s *thread = sched_ctx->priv;
    lock_spin(&thread->lock);
    thread->state &= ~_PTHREAD_STATE_TIMEDWAIT;
    lock_release(&thread->lock);
  }
#endif

  sched_queue_unlock(&cond->wait);
  CPU_INTERRUPT_RESTORESTATE;

  return 0;
}

error_t
pthread_cond_broadcast(pthread_cond_t *cond)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&cond->wait);
  struct sched_context_s *sched_ctx;

  while ((sched_ctx = sched_wake(&cond->wait)))
    {
#ifdef CONFIG_PTHREAD_COND_TIME
      struct pthread_s *thread = sched_ctx->priv;
      lock_spin(&thread->lock);
      thread->state &= ~_PTHREAD_STATE_TIMEDWAIT;
      lock_release(&thread->lock);
#endif
    }

  sched_queue_unlock(&cond->wait);
  CPU_INTERRUPT_RESTORESTATE;

  return 0;
}

#include <device/class/timer.h>
#include <device/driver.h>

error_t
pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex)
{
  error_t	res = 0;

#ifdef CONFIG_PTHREAD_CANCEL
  pthread_testcancel();
#endif

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&cond->wait);

  if (!pthread_mutex_unlock(mutex))
    {
      sched_wait_unlock(&cond->wait);

      pthread_mutex_lock(mutex);
    }
  else
    {
      sched_queue_unlock(&cond->wait);
      res = EINVAL;
    }

  CPU_INTERRUPT_RESTORESTATE;

  return res;
}

#ifdef CONFIG_PTHREAD_COND_TIME

#include <time.h>
#include <device/class/timer.h>
#include <device/driver.h>

struct pthread_cond_timedwait_ctx_s
{
  sched_queue_root_t *wait;
  struct sched_context_s *sched_ctx;
};

static KROUTINE_EXEC(pthread_cond_timer)
{
  struct dev_timer_rq_s *rq = dev_timer_rq_from_kr(kr);
  struct pthread_cond_timedwait_ctx_s *ev_ctx = rq->pvdata;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(ev_ctx->wait);

  struct sched_context_s *sched_ctx = ev_ctx->sched_ctx;
  struct pthread_s *thread = sched_ctx->priv;

  if (thread->state & _PTHREAD_STATE_TIMEDWAIT)
    {
      lock_spin(&thread->lock);
      thread->state &= ~_PTHREAD_STATE_TIMEDWAIT;
      thread->state |= _PTHREAD_STATE_TIMEOUT;
      lock_release(&thread->lock);
      sched_context_wake(ev_ctx->wait, sched_ctx);
    }

  sched_queue_unlock(ev_ctx->wait);
  CPU_INTERRUPT_RESTORESTATE;
}

error_t
pthread_cond_timedwait(pthread_cond_t *cond, 
		       pthread_mutex_t *mutex,
		       const struct timespec *abstime)
{
#ifdef CONFIG_PTHREAD_CANCEL
  pthread_testcancel();
#endif

  struct dev_timer_rq_s rq;

  if (libc_time_to_timer_rq(abstime, &rq))
    return EINVAL;

  error_t	res = 0;
  struct pthread_cond_timedwait_ctx_s ev_ctx;

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  if (!pthread_mutex_unlock(mutex))
    {
      sched_queue_wrlock(&cond->wait);

      ev_ctx.wait = &cond->wait;
      ev_ctx.sched_ctx = sched_get_current();
      struct pthread_s *this = ev_ctx.sched_ctx->priv;

      lock_spin(&this->lock);
      this->state |= _PTHREAD_STATE_TIMEDWAIT;
      this->state &= ~_PTHREAD_STATE_TIMEOUT;
      lock_release(&this->lock);

      dev_timer_rq_init_immediate(&rq, pthread_cond_timer);
      rq.pvdata = &ev_ctx;

      switch (DEVICE_OP(libc_timer(), request, &rq))
        {
        case 0:
          sched_wait_unlock(&cond->wait);

          lock_spin(&this->lock);
          if (this->state & _PTHREAD_STATE_TIMEOUT)
            res = ETIMEDOUT;
          else
            DEVICE_OP(libc_timer(), cancel, &rq);
          lock_release(&this->lock);

          break;
        case -ETIMEDOUT:
          res = ETIMEDOUT;
          sched_queue_unlock(&cond->wait);
          break;
        default:
          res = EINVAL;
          sched_queue_unlock(&cond->wait);
          break;
        }

      pthread_mutex_lock(mutex);
    }
  else
    {
      res = EINVAL;
    }

  CPU_INTERRUPT_RESTORESTATE;

  return res;
}

#endif
