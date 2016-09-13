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
 * @module {Core::Kernel services}
 * @short Kernel semaphore service
 */

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <gct_platform.h>
#include <gct/container_clist.h>

#include <mutek/scheduler.h>
#include <hexo/types.h>
#include <hexo/error.h>

/** Type for the semaphore valueing */
config_depend(CONFIG_MUTEK_SEMAPHORE)
typedef intptr_t semaphore_value_t;

#define GCT_CONTAINER_ALGO_semaphore_wait CLIST

#ifdef CONFIG_MUTEK_SEMAPHORE
/** @internal Semaphore wait object, allocated on waiter stack */
struct semaphore_wait_s
{
  GCT_CONTAINER_ENTRY    (semaphore_wait, list_entry);
  struct sched_context_s *sched_ctx;
  semaphore_value_t value;
};

GCT_CONTAINER_TYPES      (semaphore_wait, struct semaphore_wait_s *, list_entry);
#endif

/** Semaphore object structure */
config_depend(CONFIG_MUTEK_SEMAPHORE)
struct semaphore_s
{
#ifdef CONFIG_MUTEK_SEMAPHORE
  semaphore_wait_root_t wait;
  lock_t lock;
  semaphore_value_t value;
#endif
};

/** @This initializes a semaphore object and set its internal counter
    to the specified value. */
config_depend(CONFIG_MUTEK_SEMAPHORE)
error_t semaphore_init(struct semaphore_s *semaphore, semaphore_value_t value);

/** @This releases the resources used by the semaphore */
config_depend(CONFIG_MUTEK_SEMAPHORE)
void semaphore_destroy(struct semaphore_s *semaphore);

/** @This consumes the specified amount from the semaphore internal
    counter. If the current value of the counter is not large enough,
    the execution of the calling thread is suspended until the remaining
    amount becomes available. */
config_depend(CONFIG_MUTEK_SEMAPHORE)
void semaphore_take(struct semaphore_s *semaphore, semaphore_value_t n);

/** @This successes only if the semaphore internal counter is at least @em {n}.
    In this case it subtracts @em {n} from the semaphore counter an returns 0. */
config_depend(CONFIG_MUTEK_SEMAPHORE)
error_t semaphore_try_take(struct semaphore_s *semaphore, semaphore_value_t n);

/** @This suspends the execution of the caller until the
    semaphore internal counter is greater than 0. It then resets the
    counter to 0 and returns the previously observed value. */
config_depend(CONFIG_MUTEK_SEMAPHORE)
semaphore_value_t semaphore_take_any(struct semaphore_s *semaphore);

/** @This resets the semaphore internal counter to 0 and returns the
    previously observed value. */
config_depend(CONFIG_MUTEK_SEMAPHORE)
semaphore_value_t semaphore_try_take_any(struct semaphore_s *semaphore);

/** @This increases the internal counter by the specified amount then
    serves suspended threads in blocking order. */
config_depend(CONFIG_MUTEK_SEMAPHORE)
void semaphore_give(struct semaphore_s *semaphore, semaphore_value_t n);

/** @This increases the internal counter by the specified amount then
    serves any suspended threads with a low enough remaining amount. */
config_depend(CONFIG_MUTEK_SEMAPHORE)
void semaphore_give_any(struct semaphore_s *semaphore, semaphore_value_t n);

/** @This consumes the specified amount from the semaphore internal
    counter and suspends the execution of the caller only if the
    counter is currently larger than specified. In the other case, all
    waiter are resumed and the internal counter is restored to its
    initial value. */
config_depend(CONFIG_MUTEK_SEMAPHORE)
void semaphore_barrier(struct semaphore_s *semaphore, semaphore_value_t n);

/** @This returns the current value of the semaphore counter if
    greater than 0. In the other case, this function returns a
    null or negative value which indicates the counter amount needed to
    unblock all waiters which have called the @ref semaphore_take
    function. */
config_depend(CONFIG_MUTEK_SEMAPHORE)
semaphore_value_t semaphore_value(struct semaphore_s *semaphore);

/** @internalmembers @see semaphore_poll_init */
struct semaphore_poll_s
{
  struct semaphore_s *sem;
  semaphore_value_t value;
};

/** @This initializes an array of @ref semaphore_poll_s objects. This
    can be used to implement a primitive similar to unix @tt poll
    which waits for multiple asynchronous operations to terminate.

    Some helpers are provided to wait for termination of device
    requests. More helpers can be implemented in order to handle any
    kind of asynchronous event.

    See @sourcelink examples/poll_sem */
config_depend_inline(CONFIG_MUTEK_SEMAPHORE,
void semaphore_poll_init(struct semaphore_poll_s poll[], size_t count,
                         struct semaphore_s *sem),
{
  semaphore_value_t v = 1;
  uint_fast8_t i;

  for (i = 0; i < count; i++)
    {
      poll[i].sem = sem;
      poll[i].value = v;
      v <<= 1;
    }
});

C_HEADER_END

#endif

