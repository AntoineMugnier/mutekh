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

error_t sem_init(sem_t *sem, __bool_t pshared, __pthread_sem_count_t value)
{
  sem->count = value;
  __pthread_list_init(&sem->wait);
  return lock_init(&sem->lock);
}

error_t sem_wait(sem_t *sem)
{
  lock_spin_irq(&sem->lock);

  if (sem->count <= 0)
    {
      /* add current thread in sem wait queue */
      __pthread_wait(&sem->wait);

      /* switch to next thread */
      lock_release_irq(&sem->lock);
      __pthread_switch();
    }
  else
    {
      sem->count--;
      lock_release_irq(&sem->lock);
    }

  return 0;
}

error_t sem_trywait(sem_t *sem)
{
  error_t	res = 0;

  lock_spin_irq(&sem->lock);

  if (sem->count <= 0)
    res = EBUSY;
  else
    sem->count--;

  lock_release_irq(&sem->lock);

  return res;
}

error_t sem_post(sem_t *sem)
{
  struct pthread_s	*next;

  lock_spin_irq(&sem->lock);

  if ((next = __pthread_list_pop(&sem->wait)))
    __pthread_wake(next);
  else
    sem->count++;

  lock_release_irq(&sem->lock);

  return 0;
}

error_t sem_getvalue(sem_t *sem, __pthread_sem_count_t *sval)
{
  lock_spin_irq(&sem->lock);

  *sval = sem->count;

  lock_release_irq(&sem->lock);

  return 0;
}

error_t sem_destroy(sem_t *sem)
{
  lock_destroy(&sem->lock);

  return 0;
}

