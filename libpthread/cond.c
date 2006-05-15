/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include "pthread-private.h"

error_t
pthread_cond_init(pthread_cond_t *cond,
		  const pthread_condattr_t *attr)
{
  __pthread_list_init(&cond->wait);
  return lock_init(&cond->lock);
}

error_t
pthread_cond_destroy(pthread_cond_t *cond)
{
  lock_destroy(&cond->lock);

  return 0;
}

error_t
pthread_cond_signal(pthread_cond_t *cond)
{
  struct pthread_s	*thread;

  lock_spin_irq(&cond->lock);

  if ((thread = __pthread_list_pop(&cond->wait)))
    __pthread_wake(thread);

  lock_release_irq(&cond->lock);

  return 0;
}

error_t
pthread_cond_broadcast(pthread_cond_t *cond)
{
  struct pthread_s	*thread;

  lock_spin_irq(&cond->lock);

  while ((thread = __pthread_list_pop(&cond->wait)))
    __pthread_wake(thread);

  lock_release_irq(&cond->lock);

  return 0;
}

error_t
pthread_cond_wait(pthread_cond_t *cond, pthread_mutex_t *mutex)
{
  lock_spin_irq(&cond->lock);

  if (pthread_mutex_unlock(mutex))
#ifdef CONFIG_PTHREAD_CHECK
    {
      lock_release_irq(&cond->lock);
      return EINVAL;
    }
#endif

  __pthread_wait(&cond->wait);

  lock_release_irq(&cond->lock);

  __pthread_switch();

  pthread_mutex_lock(mutex);

  return 0;
}

error_t
pthread_cond_timedwait(pthread_cond_t *cond, 
		       pthread_mutex_t *mutex,
		       const struct timespec *delay)
{
  /* FIXME */
  return pthread_cond_wait(cond, mutex);
}

