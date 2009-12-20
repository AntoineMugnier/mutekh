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

#ifndef MUTEK_SEMAPHORE_H_
#define MUTEK_SEMAPHORE_H_

/**
 * @file
 * @module{Mutek}
 * @short Kernel semaphore service
 */

#include <mutek/scheduler.h>

/** Type for the semaphore counting */
typedef uint_fast8_t semaphore_count_t;

/** Semaphore object structure */
struct semaphore_s
{
	/** semaphore counter */
	semaphore_count_t count;

	/** blocked contexts wait queue */
	sched_queue_root_t wait;
};

#define SEMAPHORE_INITIALIZER {1, SCHED_QUEUE_INITIALIZER}

/**
   @this initializes a semaphore structure

   @param semaphore Semaphore structure to initialize
   @param value Initial count of semaphore
   @returns a standart error code
 */
error_t
semaphore_init(struct semaphore_s *semaphore, semaphore_count_t value);

/**
   @this waits for the semaphore count to be at least 1. @this
   decrements the count and returns.

   @param semaphore Semaphore structure
 */
void
semaphore_wait(struct semaphore_s *semaphore);

/**
   @this tries to take the semaphore, but dont wait if it cant take
   it. It will return EBUSY instead.

   @param semaphore Semaphore structure
   @returns 0 if taken else EBUSY
 */
error_t
semaphore_trywait(struct semaphore_s *semaphore);

/**
   @this increments the semaphore count, and wakes 1 frozen task.

   @param semaphore Semaphore structure
 */
void
semaphore_post(struct semaphore_s *semaphore);

/**
   @this gets the current value of a semaphore.

   @param semaphore Semaphore structure
   @param sval Return value: current semaphore count
 */
void
semaphore_getvalue(struct semaphore_s *semaphore, semaphore_count_t *sval);

/**
   @this destroys a semaphore. If some tasks are still waiting on the
   semaphore, behaviour is unpredictible.

   @param semaphore Semaphore structure
 */
void
semaphore_destroy(struct semaphore_s *semaphore);

#endif

