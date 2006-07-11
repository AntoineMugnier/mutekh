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

/**
   @file ARCH lock
*/

#if !defined(LOCK_H_) || defined(ARCH_LOCK_H_)
#error This file can not be included directly
#else

#define ARCH_LOCK_H_

#include "hexo/iospace.h"

//#define ARCH_HAS_ATOMIC

struct		arch_lock_s
{
  uintptr_t	ramlock;
};

//#define ARCH_LOCK_INITIALIZER	{ .a = 0 }

extern uintptr_t __ramlock_base;

static inline error_t arch_lock_init(struct arch_lock_s *lock)
{
  lock->ramlock = __ramlock_base;

  __ramlock_base += 4;

  return 0;
}

static inline void arch_lock_destroy(struct arch_lock_s *lock)
{
}

static inline __bool_t arch_lock_try(struct arch_lock_s *lock)
{
  return cpu_mem_read_32(lock->ramlock);
}

static inline void arch_lock_spin(struct arch_lock_s *lock)
{
  while (arch_lock_try(lock))
    ;
}

static inline void arch_lock_release(struct arch_lock_s *lock)
{
  cpu_mem_write_32(lock->ramlock, 0);
}

static inline __bool_t arch_lock_state(struct arch_lock_s *lock)
{
  __bool_t	state = arch_lock_try(lock);

  if (!state)
    arch_lock_release(lock);

  return state;
}

#endif

