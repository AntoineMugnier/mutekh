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

#include <hexo/types.h>
#include <hexo/error.h>

#include <mutek/scheduler.h>
#include <mutek/semaphore.h>

error_t semaphore_init(struct semaphore_s *semaphore, uint_fast8_t value)
{
	semaphore->count = value;
	return sched_queue_init(&semaphore->wait);
}

void semaphore_destroy(struct semaphore_s *semaphore)
{
	sched_queue_destroy(&semaphore->wait);
}

void semaphore_wait(struct semaphore_s *semaphore)
{
	CPU_INTERRUPT_SAVESTATE_DISABLE;
	sched_queue_wrlock(&semaphore->wait);

	if (semaphore->count <= 0)
    {
		/* add current thread in semaphore wait queue */
		sched_wait_unlock(&semaphore->wait);
    }
	else
    {
		semaphore->count--;
		sched_queue_unlock(&semaphore->wait);
    }

	CPU_INTERRUPT_RESTORESTATE;
}

error_t semaphore_trywait(struct semaphore_s *semaphore)
{
	error_t	res = 0;

	CPU_INTERRUPT_SAVESTATE_DISABLE;
	sched_queue_wrlock(&semaphore->wait);

	if (semaphore->count <= 0)
		res = EBUSY;
	else
		semaphore->count--;

	sched_queue_unlock(&semaphore->wait);
	CPU_INTERRUPT_RESTORESTATE;

	return res;
}

void semaphore_post(struct semaphore_s *semaphore)
{
	CPU_INTERRUPT_SAVESTATE_DISABLE;
	sched_queue_wrlock(&semaphore->wait);

	if (!sched_wake(&semaphore->wait))
		semaphore->count++;

	sched_queue_unlock(&semaphore->wait);
	CPU_INTERRUPT_RESTORESTATE;
}

void semaphore_getvalue(struct semaphore_s *semaphore, semaphore_count_t *sval)
{
	CPU_INTERRUPT_SAVESTATE_DISABLE;
	sched_queue_wrlock(&semaphore->wait);

	*sval = semaphore->count;

	sched_queue_unlock(&semaphore->wait);
	CPU_INTERRUPT_RESTORESTATE;
}

