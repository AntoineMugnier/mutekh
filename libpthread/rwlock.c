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
pthread_rwlock_destroy(pthread_rwlock_t *rwlock)
{
  lock_destroy(&rwlock->lock);

  return 0;
}

error_t
pthread_rwlock_init(pthread_rwlock_t *rwlock,
		    const pthread_rwlockattr_t *attr)
{
  __pthread_list_init(&rwlock->wait_rd);
  __pthread_list_init(&rwlock->wait_wr);
  rwlock->count = 0;
  return lock_init(&rwlock->lock);
}

error_t
pthread_rwlock_rdlock(pthread_rwlock_t *rwlock)
{
  lock_spin_irq(&rwlock->lock);

  /* check write locked or write lock pending */
  if (rwlock->count < 0 || __pthread_list_head(&rwlock->wait_wr))
    {
      /* add current thread in read wait queue */
      __pthread_wait(&rwlock->wait_rd);

      /* switch to next thread */
      lock_release_irq(&rwlock->lock);
      __pthread_switch();
    }
  else
    {
      /* mark rwlock as used */
      rwlock->count++;
      lock_release_irq(&rwlock->lock);
    }

  return 0;
}

error_t
pthread_rwlock_tryrdlock(pthread_rwlock_t *rwlock)
{
  error_t	res = 0;

  lock_spin_irq(&rwlock->lock);

  /* check write locked or write lock pending */
  if (rwlock->count < 0 || __pthread_list_head(&rwlock->wait_wr))
    res = -EBUSY;
  else
    rwlock->count++;

  lock_release_irq(&rwlock->lock);

  return res;
}

error_t
pthread_rwlock_wrlock(pthread_rwlock_t *rwlock)
{
  lock_spin_irq(&rwlock->lock);

  /* check locked */
  if (rwlock->count != 0)
    {
      /* add current thread in write wait queue */
      __pthread_wait(&rwlock->wait_wr);

      /* switch to next thread */
      lock_release_irq(&rwlock->lock);
      __pthread_switch();
    }
  else
    {
      /* mark rwlock as write locked */
      rwlock->count--;
      lock_release_irq(&rwlock->lock);
    }

  return 0;
}

error_t
pthread_rwlock_trywrlock(pthread_rwlock_t *rwlock)
{
  error_t	res = 0;

  lock_spin_irq(&rwlock->lock);

  /* check locked */
  if (rwlock->count != 0)
    res = -EBUSY;
  else
    rwlock->count--;

  lock_release_irq(&rwlock->lock);

  return 0;
}

error_t
pthread_rwlock_unlock(pthread_rwlock_t *rwlock)
{
  error_t	res = 0;

  lock_spin_irq(&rwlock->lock);

  switch (rwlock->count)
    {
      struct pthread_s	*next;

      /* read locked once */
    case 1:
      /* write locked */
    case -1:

      rwlock->count = 0;
      /* we are unlocked here */

      /* try to wake 1 pending write thread */
      if ((next = __pthread_list_pop(&rwlock->wait_wr)))
	{
	  __pthread_wake(next);
	  rwlock->count--;
	  break;
	}

      /* wake all pending read threads */
      while ((next = __pthread_list_pop(&rwlock->wait_rd)))
	{
	  __pthread_wake(next);
	  rwlock->count++;
	}

      break;

#ifdef CONFIG_PTHREAD_CHECK
      /* already unlocked */
    case 0:
      res = -EPERM;
      break;
#endif

      /* read locked mutiple times */
    default:
      rwlock->count--;
      break;
    }

  lock_release_irq(&rwlock->lock);

  return res;
}

