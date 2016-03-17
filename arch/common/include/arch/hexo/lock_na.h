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

/**
   @file ARCH lock
*/

#if !defined(LOCK_H_) || defined(ARCH_LOCK_H_)
#error This file can not be included directly
#else

#define ARCH_LOCK_H_

#include <assert.h>
#include <hexo/cpu.h>

struct		__arch_lock_s
{
#if defined(CONFIG_HEXO_LOCK_DEBUG) && !defined(CONFIG_ARCH_SMP)
  void *pc;
# define __arch_lock_unlock pc  /* field unlocked from context switch asm */
#endif
};

#define ARCH_LOCK_INITIALIZER	{ }

ALWAYS_INLINE error_t __arch_lock_init(struct __arch_lock_s *lock)
{
#if defined(CONFIG_ARCH_SMP)
  cpu_trap();
#elif defined(CONFIG_HEXO_LOCK_DEBUG)
  lock->pc = NULL;
  order_compiler_mem();
#endif
  return 0;
}

ALWAYS_INLINE void __arch_lock_destroy(struct __arch_lock_s *lock)
{
#if defined(CONFIG_ARCH_SMP)
  cpu_trap();
#endif
}

ALWAYS_INLINE bool_t __arch_lock_try(struct __arch_lock_s *lock)
{
#if defined(CONFIG_ARCH_SMP)
  cpu_trap();
#elif defined(CONFIG_HEXO_LOCK_DEBUG)
  order_compiler_mem();
  assert(lock->pc == NULL);
  lock->pc = __builtin_return_address(0);
  order_compiler_mem();
#endif
  return 0;
}

ALWAYS_INLINE void __arch_lock_spin(struct __arch_lock_s *lock)
{
#if defined(CONFIG_ARCH_SMP)
  cpu_trap();
#elif defined(CONFIG_HEXO_LOCK_DEBUG)
  order_compiler_mem();
  assert(lock->pc == NULL);
  lock->pc = __builtin_return_address(0);
  order_compiler_mem();
#endif
}

ALWAYS_INLINE void __arch_lock_release(struct __arch_lock_s *lock)
{
#if defined(CONFIG_ARCH_SMP)
  cpu_trap();
#elif defined(CONFIG_HEXO_LOCK_DEBUG)
  order_compiler_mem();
  assert(lock->pc != NULL);
  lock->pc = NULL;
  order_compiler_mem();
#endif
}

#endif

