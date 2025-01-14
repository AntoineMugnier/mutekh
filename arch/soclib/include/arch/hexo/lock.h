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

#if !defined(LOCK_H_) || defined(ARCH_SOCLIB_LOCK_H_)
#error This file can not be included directly
#else

#include <hexo/ordering.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
# include <arch/soclib/mem_checker.h>
#endif

# ifdef CONFIG_ARCH_SOCLIB_RAMLOCK

/************************************************************** 
	USE RAMLOCKS
 **************************************************************/

#include <hexo/iospace.h>

//#define ARCH_HAS_ATOMIC

struct		__arch_lock_s
{
  uintptr_t	ramlock;
};

//#define ARCH_LOCK_INITIALIZER	{ .a = 0 }

extern uintptr_t __ramlock_base;

ALWAYS_INLINE error_t __arch_lock_init(struct __arch_lock_s *lock)
{
  /* FIXME add allocation algorithm */
  lock->ramlock = __ramlock_base;

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_declare_lock((void*)lock->ramlock, 1);
#endif

  __ramlock_base += 4;

  return 0;
}

ALWAYS_INLINE void __arch_lock_destroy(struct __arch_lock_s *lock)
{
#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_declare_lock((void*)lock->ramlock, 0);
#endif
}

ALWAYS_INLINE bool_t __arch_lock_try(struct __arch_lock_s *lock)
{
  uint32_t result;
  result = cpu_mem_read_32(lock->ramlock);
  order_smp_mem();
  return result;
}

ALWAYS_INLINE void __arch_lock_spin(struct __arch_lock_s *lock)
{
  while (__arch_lock_try(lock))
    ;
}

ALWAYS_INLINE void __arch_lock_release(struct __arch_lock_s *lock)
{
  order_smp_mem();
  cpu_mem_write_32(lock->ramlock, 0);
}

ALWAYS_INLINE bool_t __arch_lock_state(struct __arch_lock_s *lock)
{
  bool_t	state = __arch_lock_try(lock);

  if (!state)
    __arch_lock_release(lock);

  return state;
}

# else  /* CONFIG_ARCH_SOCLIB_RAMLOCK */

/************************************************************** 
	USE CPU ATOMIC OPS
 **************************************************************/

#include "arch/common/include/arch/hexo/lock_cpuatomic.h"

#endif

# define ARCH_SOCLIB_LOCK_H_

#endif

