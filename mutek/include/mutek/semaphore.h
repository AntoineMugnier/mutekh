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

#ifndef MUTEK_SEMAPHORE_H_
#define MUTEK_SEMAPHORE_H_

/**
 * @file
 * @module{Mutek}
 * @short Kernel semaphore service
 *
 * The semaphore functions are empty and always succeed when the
 * scheduler is disabled in the configuration.
 */

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <mutek/scheduler.h>
#include <hexo/types.h>
#include <hexo/error.h>

/** Type for the semaphore counting */
typedef int_fast8_t semaphore_count_t;

/** Semaphore object structure */
struct semaphore_s
{
#ifdef CONFIG_MUTEK_CONTEXT_SCHED
	/** semaphore counter */
	semaphore_count_t count;

	/** blocked contexts wait queue */
	sched_queue_root_t wait;
# define SEMAPHORE_INITIALIZER {1, SCHED_QUEUE_INITIALIZER}
#else
# define SEMAPHORE_INITIALIZER { }
#endif
};

#ifndef CONFIG_MUTEK_CONTEXT_SCHED
ALWAYS_INLINE error_t
semaphore_init(struct semaphore_s *semaphore, semaphore_count_t value)
{
  return 0;
}

ALWAYS_INLINE void
semaphore_take(struct semaphore_s *semaphore, semaphore_count_t n)
{
}

ALWAYS_INLINE error_t
semaphore_try_take(struct semaphore_s *semaphore, semaphore_count_t n)
{
  return 0;
}

ALWAYS_INLINE void
semaphore_give(struct semaphore_s *semaphore, semaphore_count_t n)
{
}

ALWAYS_INLINE semaphore_count_t
semaphore_value(struct semaphore_s *semaphore)
{
  return 1;
}

ALWAYS_INLINE void
semaphore_destroy(struct semaphore_s *semaphore)
{
}
#endif

/**
   @this initializes a semaphore structure

   @param semaphore Semaphore structure to initialize
   @param value Initial count of semaphore
   @returns a standart error code
 */
config_depend(CONFIG_MUTEK_SEMAPHORE)
error_t semaphore_init(struct semaphore_s *semaphore, semaphore_count_t value);

/**
   @this waits for the semaphore count to be at least n. @this
   decrements the count and returns.

   @param semaphore Semaphore structure
   @param n Semaphore count to change
 */
config_depend(CONFIG_MUTEK_SEMAPHORE)
void semaphore_take(struct semaphore_s *semaphore, semaphore_count_t n);

/**
   @this tries to take the semaphore, but dont wait if it cant take
   it. It will return EBUSY instead.

   @param semaphore Semaphore structure
   @param n Semaphore count to change
   @returns 0 if taken else EBUSY
 */
config_depend(CONFIG_MUTEK_SEMAPHORE)
error_t semaphore_try_take(struct semaphore_s *semaphore, semaphore_count_t n);

/**
   @this changes the count of semaphore in a way it may not block,
   even if count does become negative.

   @param semaphore Semaphore structure
 */
config_depend(CONFIG_MUTEK_SEMAPHORE)
void semaphore_give(struct semaphore_s *semaphore, semaphore_count_t n);

/**
   @this gets the current value of a semaphore. This function always
   returns 1 if the scheduler is disabled.

   @param semaphore Semaphore structure
   @return current semaphore count.
 */
config_depend(CONFIG_MUTEK_SEMAPHORE)
semaphore_count_t semaphore_value(struct semaphore_s *semaphore);

/**
   @this destroys a semaphore. If some tasks are still waiting on the
   semaphore, behaviour is unpredictible.

   @param semaphore Semaphore structure
 */
config_depend(CONFIG_MUTEK_SEMAPHORE)
void semaphore_destroy(struct semaphore_s *semaphore);

C_HEADER_END

#endif

