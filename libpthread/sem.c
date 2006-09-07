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

#include <hexo/scheduler.h>
#include <hexo/error.h>
#include <pthread.h>
#include <semaphore.h>

error_t sem_init(sem_t *sem, bool_t pshared, __pthread_sem_count_t value)
{
  sem->count = value;
  return sched_queue_init(&sem->wait);
}

error_t sem_destroy(sem_t *sem)
{
  sched_queue_destroy(&sem->wait);

  return 0;
}

error_t sem_wait(sem_t *sem)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&sem->wait);

  if (sem->count <= 0)
    {
      /* add current thread in sem wait queue */
      sched_wait_unlock(&sem->wait);
    }
  else
    {
      sem->count--;
      sched_queue_unlock(&sem->wait);
    }

  CPU_INTERRUPT_RESTORESTATE;

  return 0;
}

error_t sem_trywait(sem_t *sem)
{
  error_t	res = 0;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&sem->wait);

  if (sem->count <= 0)
    res = EBUSY;
  else
    sem->count--;

  sched_queue_unlock(&sem->wait);
  CPU_INTERRUPT_RESTORESTATE;

  return res;
}

error_t sem_post(sem_t *sem)
{
  struct pthread_s	*next;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&sem->wait);

  if (!sched_wake(&sem->wait))
    sem->count++;

  sched_queue_unlock(&sem->wait);
  CPU_INTERRUPT_RESTORESTATE;

  return 0;
}

error_t sem_getvalue(sem_t *sem, __pthread_sem_count_t *sval)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_queue_wrlock(&sem->wait);

  *sval = sem->count;

  sched_queue_unlock(&sem->wait);
  CPU_INTERRUPT_RESTORESTATE;

  return 0;
}

