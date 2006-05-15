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


/** unlock and relock the mutex if other threads are waiting */
static inline void
__pthread_mutex_unlock_relock(pthread_mutex_t *mutex)
{
  struct pthread_s	*thread = __pthread_list_pop(&mutex->wait);

  mutex->count--;

  if (thread)
    {
      __pthread_wake(thread);
      mutex->count++;
    }
}



/************************************************************************
		PTHREAD_MUTEX_NORMAL
************************************************************************/

error_t
__pthread_mutex_normal_lock(pthread_mutex_t *mutex)
{
  lock_spin_irq(&mutex->lock);

  /* check current mutex state */
  if (mutex->count)
    {
      /* add current thread in mutex wait queue */
      __pthread_wait(&mutex->wait);

      /* switch to next thread */
      lock_release_irq(&mutex->lock);
      __pthread_switch();
    }
  else
    {
      /* mark mutex as used */
      mutex->count++;
      lock_release_irq(&mutex->lock);
    }

  return 0;
}

error_t
__pthread_mutex_normal_trylock(pthread_mutex_t *mutex)
{
  error_t	res = EBUSY;

  lock_spin_irq(&mutex->lock);

  /* check current mutex state */
  if (!mutex->count)
    {
      mutex->count++;
      res = 0;
    }

  lock_release_irq(&mutex->lock);

  return res;
}

error_t
__pthread_mutex_normal_unlock(pthread_mutex_t *mutex)
{
  lock_spin_irq(&mutex->lock);

  __pthread_mutex_unlock_relock(mutex);

  lock_release_irq(&mutex->lock);

  return 0;
}

#ifdef CONFIG_PTHREAD_MUTEX_ATTR

CPUARCH_LOCAL pthread_mutexattr_t __pthread_mutex_attr_normal =
  {
    .type =
    {
      .mutex_lock	= __pthread_mutex_normal_lock,
      .mutex_trylock	= __pthread_mutex_normal_trylock,
      .mutex_unlock	= __pthread_mutex_normal_unlock,
    }
  };

#endif



/************************************************************************
		PTHREAD_MUTEX_ERRORCHECK
************************************************************************/

#ifdef CONFIG_PTHREAD_MUTEX_ATTR

static error_t
__pthread_mutex_errorcheck_lock(pthread_mutex_t *mutex)
{
  error_t		res = 0;

  lock_spin_irq(&mutex->lock);

  /* check current mutex state */
  if (mutex->count)
    {
      if (mutex->owner == pthread_self())
	{
	  /* dead lock condition detected */
	  res = EDEADLK;
	  lock_release_irq(&mutex->lock);
	}
      else
	{
	  /* add current thread in mutex wait queue */
	  __pthread_wait(&mutex->wait);

	  /* switch to next thread */
	  lock_release_irq(&mutex->lock);
	  __pthread_switch();
	}
    }
  else
    {
      /* mark mutex as used */
      mutex->owner = pthread_self();
      mutex->count++;
      lock_release_irq(&mutex->lock);
    }

  return res;
}

static error_t
__pthread_mutex_errorcheck_trylock(pthread_mutex_t *mutex)
{
  error_t	res = 0;

  lock_spin_irq(&mutex->lock);

  /* check current mutex state */
  if (mutex->count)
    {
      if (mutex->owner == pthread_self())
	/* dead lock condition detected */
	res = EDEADLK;
      else
	res = EBUSY;
    }
  else
    mutex->count++;

  lock_release_irq(&mutex->lock);

  return res;
}

static error_t
__pthread_mutex_errorcheck_unlock(pthread_mutex_t *mutex)
{
  error_t	res = 0;

  lock_spin_irq(&mutex->lock);

  if (mutex->count)
    {
      if (mutex->owner == pthread_self())
	__pthread_mutex_unlock_relock(mutex);
      else
	res = EPERM;
    }
  else
    res = EBUSY;

  lock_release_irq(&mutex->lock);

  return 0;
}

CPUARCH_LOCAL pthread_mutexattr_t __pthread_mutex_attr_errorcheck =
  {
    .type =
    {
      .mutex_lock	= __pthread_mutex_errorcheck_lock,
      .mutex_trylock	= __pthread_mutex_errorcheck_trylock,
      .mutex_unlock	= __pthread_mutex_errorcheck_unlock,
    }
  };

#endif



/************************************************************************
		PTHREAD_MUTEX_RECURSIVE
************************************************************************/

#ifdef CONFIG_PTHREAD_MUTEX_ATTR

static error_t
__pthread_mutex_recursive_lock(pthread_mutex_t *mutex)
{
  lock_spin_irq(&mutex->lock);

  /* check current mutex state */
  if (mutex->count && (mutex->owner != pthread_self()))
    {
      /* add current thread in mutex wait queue */
      __pthread_wait(&mutex->wait);

      /* switch to next thread */
      lock_release_irq(&mutex->lock);
      __pthread_switch();
    }
  else
    {
      /* mark mutex as used */
      mutex->owner = pthread_self();
      mutex->count++;
      lock_release_irq(&mutex->lock);
    }

  return 0;
}

static error_t
__pthread_mutex_recursive_trylock(pthread_mutex_t *mutex)
{
  error_t	res = 0;

  lock_spin_irq(&mutex->lock);

  if (mutex->count && (mutex->owner != pthread_self()))
    res = EBUSY;
  else
    mutex->count++;

  lock_release_irq(&mutex->lock);

  return res;
}

static error_t
__pthread_mutex_recursive_unlock(pthread_mutex_t *mutex)
{
  lock_spin_irq(&mutex->lock);

  if (mutex->count == 1)
    __pthread_mutex_unlock_relock(mutex);
  else
    mutex->count--;

  lock_release_irq(&mutex->lock);

  return 0;
}

CPUARCH_LOCAL pthread_mutexattr_t __pthread_mutex_attr_recursive =
  {
    .type =
    {
      .mutex_lock	= __pthread_mutex_recursive_lock,
      .mutex_trylock	= __pthread_mutex_recursive_trylock,
      .mutex_unlock	= __pthread_mutex_recursive_unlock,
    }
  };

#endif



/************************************************************************/



error_t
pthread_mutex_init(pthread_mutex_t *mutex,
		   const pthread_mutexattr_t *attr)
{
  mutex->count = 0;
  __pthread_list_init(&mutex->wait);

#ifdef CONFIG_PTHREAD_MUTEX_ATTR
  /* default mutex attribute */
  if (!attr)
    attr = CPUARCH_LOCAL_ADDR(__pthread_mutex_attr_normal);

  mutex->attr = attr;
#endif

  return lock_init(&mutex->lock);
}


error_t
pthread_mutex_destroy(pthread_mutex_t *mutex)
{
  lock_destroy(&mutex->lock);
  return 0;
}

#ifdef CONFIG_PTHREAD_MUTEX_ATTR

error_t
pthread_mutexattr_settype(pthread_mutexattr_t *attr, int_fast8_t type)
{
  switch (type)
    {
    case PTHREAD_MUTEX_DEFAULT:
    case PTHREAD_MUTEX_NORMAL:
      attr->type = CPUARCH_LOCAL_ADDR(__pthread_mutex_attr_normal)->type;
      return 0;

    case PTHREAD_MUTEX_ERRORCHECK:
      attr->type = CPUARCH_LOCAL_ADDR(__pthread_mutex_attr_errorcheck)->type;
      return 0;

    case PTHREAD_MUTEX_RECURSIVE:
      attr->type = CPUARCH_LOCAL_ADDR(__pthread_mutex_attr_recursive)->type;
      return 0;
    }

  return EINVAL;
}

#endif

