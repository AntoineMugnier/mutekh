/*
  This file is part of MutekH.
  
  MutekH is free software; you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as published
  by the Free Software Foundation; version 2.1 of the License.
  
  MutekH is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.
  
  You should have received a copy of the GNU Lesser General Public
  License along with MutekH; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
  02110-1301 USA.

  Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <hexo/types.h>
#include <hexo/error.h>

#include <mutek/scheduler.h>
#include <mutek/semaphore.h>

#include <assert.h>

GCT_CONTAINER_FCNS   (semaphore_wait, static inline, semaphore_wait,,
                      init, destroy, pushback, remove, head, isempty, clear);

error_t semaphore_init(struct semaphore_s *semaphore, semaphore_value_t value)
{
  assert(value >= 0);
  semaphore->value = value;
  semaphore_wait_init(&semaphore->wait);
  return lock_init(&semaphore->lock);
}

void semaphore_destroy(struct semaphore_s *semaphore)
{
  assert(semaphore_wait_isempty(&semaphore->wait));
  semaphore_wait_destroy(&semaphore->wait);
  lock_destroy(&semaphore->lock);
}

static inline semaphore_value_t
semaphore_wait(struct semaphore_s *semaphore, semaphore_value_t n)
{
  struct semaphore_wait_s w;
  w.value = n;
  w.sched_ctx = sched_get_current();
  semaphore_wait_pushback(&semaphore->wait, &w);
  sched_stop_unlock(&semaphore->lock);
  return w.value;
}

void semaphore_take(struct semaphore_s *semaphore, semaphore_value_t n)
{
  assert(n >= 0);

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&semaphore->lock);

  if (semaphore->value < n)
    {
      n -= semaphore->value;
      semaphore->value = 0;
      semaphore_wait(semaphore, n);
    }
  else
    {
      semaphore->value -= n;
      lock_release(&semaphore->lock);
    }

  CPU_INTERRUPT_RESTORESTATE;
}

error_t semaphore_try_take(struct semaphore_s *semaphore, semaphore_value_t n)
{
  error_t r = -EBUSY;
  assert(n >= 0);

  LOCK_SPIN_IRQ(&semaphore->lock);

  if (semaphore->value >= n)
    {
      semaphore->value -= n;
      r = 0;
    }

  LOCK_RELEASE_IRQ(&semaphore->lock);

  return r;
}

semaphore_value_t semaphore_take_any(struct semaphore_s *semaphore)
{
  semaphore_value_t r;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&semaphore->lock);

  if (semaphore->value == 0)
    {
      r = semaphore_wait(semaphore, 0);
    }
  else
    {
      r = semaphore->value;
      semaphore->value = 0;
      lock_release(&semaphore->lock);
    }

  CPU_INTERRUPT_RESTORESTATE;

  return r;
}

semaphore_value_t semaphore_try_take_any(struct semaphore_s *semaphore)
{
  semaphore_value_t r;

  LOCK_SPIN_IRQ(&semaphore->lock);

  r = semaphore->value;
  semaphore->value = 0;

  LOCK_RELEASE_IRQ(&semaphore->lock);

  return r;
}

void semaphore_give(struct semaphore_s *semaphore, semaphore_value_t n)
{
  assert(n >= 0);

  LOCK_SPIN_IRQ(&semaphore->lock);
  struct semaphore_wait_s *w;

  n += semaphore->value;

  while (n > 0 && (w = semaphore_wait_head(&semaphore->wait)))
    {
      if (w->value)             /* waiter used take() */
        {
          if (w->value > n)
            {
              /* not large enough, end */
              w->value -= n;
              n = 0;
              break;
            }
          else
            {
              /* resume this one and continue */
              n -= w->value;
            }
        }
      else                      /* waiter used take_all() */
        {
          /* resume this one and return */
          w->value = n;
          n = 0;
        }

      /* resume this waiter */
      semaphore_wait_remove(&semaphore->wait, w);
      sched_context_start(w->sched_ctx);
    }

  semaphore->value = n;

  LOCK_RELEASE_IRQ(&semaphore->lock);
}

void semaphore_give_any(struct semaphore_s *semaphore, semaphore_value_t n)
{
  assert(n >= 0);

  LOCK_SPIN_IRQ(&semaphore->lock);

  n += semaphore->value;

  GCT_FOREACH(semaphore_wait, &semaphore->wait, w, {
      if (n == 0)
	GCT_FOREACH_BREAK;

      if (w->value)             /* waiter used take() */
        {
          if (w->value > n)
            {
              /* not large enough, continue */
              GCT_FOREACH_CONTINUE;
            }
          else
            {
              /* resume this one and continue */
              n -= w->value;
            }
        }
      else                      /* waiter used take_all() */
        {
          /* resume this one and return */
          w->value = n;
          n = 0;
        }

      sched_context_start(w->sched_ctx);
      GCT_FOREACH_DROP;
  });

  semaphore->value = n;

  LOCK_RELEASE_IRQ(&semaphore->lock);
}

void semaphore_barrier(struct semaphore_s *semaphore, semaphore_value_t n)
{
  assert(n >= 0);

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&semaphore->lock);

  if (semaphore->value <= n)
    {
      /* resume all */
      GCT_FOREACH(semaphore_wait, &semaphore->wait, w, {
          sched_context_start(w->sched_ctx);
          n += w->value;
      });
      semaphore_wait_clear(&semaphore->wait);
      semaphore->value = n;
      lock_release(&semaphore->lock);
    }
  else
    {
      semaphore->value -= n;
      semaphore_wait(semaphore, n);
    }

  CPU_INTERRUPT_RESTORESTATE;
}

semaphore_value_t semaphore_value(struct semaphore_s *semaphore)
{
  semaphore_value_t r;

  LOCK_SPIN_IRQ(&semaphore->lock);

  r = semaphore->value;
  if (r == 0)
    {
      GCT_FOREACH(semaphore_wait, &semaphore->wait, w, {
          r -= w->value;
      });
    }

  LOCK_RELEASE_IRQ(&semaphore->lock);

  return r;
}

extern inline void
semaphore_poll_init(struct semaphore_poll_s poll[], size_t count,
                    struct semaphore_s *sem);

