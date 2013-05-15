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

#ifndef RW_LOCK_H_
#define RW_LOCK_H_

/**
 * @file
 * @module{Mutek}
 * @short General purpose read/write lock primitives
 *
 * The rwlock functions are empty and always succeed when the
 * scheduler is disabled in the configuration.
 */

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <mutek/scheduler.h>
#include <hexo/types.h>
#include <hexo/error.h>

struct rwlock_s
{
#ifdef CONFIG_MUTEK_SCHEDULER
  /** lock counter:
      count < 0 is write locked,
      count > 0 is read locked. */
  int_fast8_t                           count;
  /** blocked threads waiting for read */
  sched_queue_root_t                    wait_rd;
  /** blocked threads waiting for write */
  sched_queue_root_t                    wait_wr;

/** normal rwlock object static initializer */
# define RWLOCK_INITIALIZER                                     \
  {                                                             \
     .count = 0,                                                \
     .wait_rd = CONTAINER_ROOT_INITIALIZER(sched_queue, DLIST), \
     .wait_wr = CONTAINER_ROOT_INITIALIZER(sched_queue, DLIST),	\
  }

#else
# define RWLOCK_INITIALIZER { }
#endif
};

#ifndef CONFIG_MUTEK_SCHEDULER
static inline error_t rwlock_destroy(struct rwlock_s *rwlock)
{
  return 0;
}

static inline error_t rwlock_init(struct rwlock_s *rwlock)
{
  return 0;
}

static inline error_t rwlock_rdlock(struct rwlock_s *rwlock)
{
  return 0;
}

static inline error_t rwlock_wrlock(struct rwlock_s *rwlock);
{
  return 0;
}

static inline error_t rwlock_tryrdlock(struct rwlock_s *rwlock);
{
  return 0;
}

static inline error_t rwlock_trywrlock(struct rwlock_s *rwlock);
{
  return 0;
}

static inline error_t rwlock_unlock(struct rwlock_s *rwlock);
{
  return 0;
}
#endif

config_depend(CONFIG_MUTEK_RWLOCK)
error_t rwlock_destroy(struct rwlock_s *rwlock);

config_depend(CONFIG_MUTEK_RWLOCK)
error_t rwlock_init(struct rwlock_s *rwlock);

config_depend(CONFIG_MUTEK_RWLOCK)
error_t rwlock_rdlock(struct rwlock_s *rwlock);

config_depend(CONFIG_MUTEK_RWLOCK)
error_t rwlock_wrlock(struct rwlock_s *rwlock);

config_depend(CONFIG_MUTEK_RWLOCK)
error_t rwlock_tryrdlock(struct rwlock_s *rwlock);

config_depend(CONFIG_MUTEK_RWLOCK)
error_t rwlock_trywrlock(struct rwlock_s *rwlock);

config_depend(CONFIG_MUTEK_RWLOCK)
error_t rwlock_unlock(struct rwlock_s *rwlock);

C_HEADER_END

#endif
