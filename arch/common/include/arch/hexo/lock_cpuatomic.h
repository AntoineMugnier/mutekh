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
#include <hexo/atomic.h>
#include <hexo/ordering.h>

#define ARCH_HAS_ATOMIC

struct		__arch_lock_s
{
#if defined(CONFIG_ARCH_SMP)
  atomic_int_t a;
# define __arch_lock_unlock a /* field unlocked from context switch asm */
#endif
#if defined(CONFIG_HEXO_LOCK_DEBUG)
  void *pc;
# if defined(CONFIG_ARCH_SMP)
  cpu_id_t cpu_id;
# else
#  define __arch_lock_unlock pc
# endif
#endif
};

#if defined(CONFIG_ARCH_SMP)
FIRST_FIELD_ASSERT(__arch_lock_s, a);
#elif defined(CONFIG_HEXO_LOCK_DEBUG)
FIRST_FIELD_ASSERT(__arch_lock_s, pc);
#endif

#define ARCH_LOCK_INITIALIZER	{ }

ALWAYS_INLINE error_t __arch_lock_init(struct __arch_lock_s *lock)
{
#if defined(CONFIG_ARCH_SMP)
  lock->a = 0;
#endif

#if defined(CONFIG_HEXO_LOCK_DEBUG)
  lock->pc = NULL;
# if defined(CONFIG_ARCH_SMP)
  lock->cpu_id = 0;
# endif
#endif

#if defined(CONFIG_SOCLIB_MEMCHECK)
# if defined(CONFIG_HEXO_LOCK_DEBUG) || defined(CONFIG_ARCH_SMP)
  soclib_mem_check_declare_lock((void*)lock, 1);
# endif
#endif

  order_smp_write();
  return 0;
}

ALWAYS_INLINE void __arch_lock_destroy(struct __arch_lock_s *lock)
{
#if defined(CONFIG_SOCLIB_MEMCHECK)
# if defined(CONFIG_HEXO_LOCK_DEBUG) || defined(CONFIG_ARCH_SMP)
  soclib_mem_check_declare_lock((void*)lock, 0);
# endif
#endif
}

ALWAYS_INLINE bool_t __arch_lock_try(struct __arch_lock_s *lock)
{
  bool_t res = 0;
#if defined(CONFIG_HEXO_LOCK_DEBUG)
# if defined(CONFIG_ARCH_SMP)
  assert(!lock->a || lock->cpu_id != cpu_id() + 1);
# else
  assert(lock->pc == NULL);
# endif
#endif

#if defined(CONFIG_ARCH_SMP)
  res = __cpu_atomic_bit_testset(&lock->a, 0);
#endif

#if defined(CONFIG_HEXO_LOCK_DEBUG)
  if (!res)
    {
      lock->pc = ({ __label__ _pc; _pc: &&_pc; });
# if defined(CONFIG_ARCH_SMP)
      lock->cpu_id = cpu_id() + 1;
# endif
    }
#endif
  order_smp_mem();
  return res;
}

ALWAYS_INLINE void __arch_lock_spin(struct __arch_lock_s *lock)
{
#if defined(CONFIG_HEXO_LOCK_DEBUG)
# if defined(CONFIG_ARCH_SMP)
  assert(!lock->a || lock->cpu_id != cpu_id() + 1);
# else
  assert(lock->pc == NULL);
# endif
#endif

#if defined(CONFIG_ARCH_SMP)
# ifdef CONFIG_DEBUG_SPINLOCK_LIMIT
  uint32_t deadline = CONFIG_DEBUG_SPINLOCK_LIMIT;

  while (__cpu_atomic_bit_testset(&lock->a, 0))
    {
      asm volatile("nop");
      assert(deadline-- > 0);
    }
# else
  __cpu_atomic_bit_waitset(&lock->a, 0);
# endif
#endif

#if defined(CONFIG_HEXO_LOCK_DEBUG)
  lock->pc = ({ __label__ _pc; _pc: &&_pc; });
# if defined(CONFIG_ARCH_SMP)
  lock->cpu_id = cpu_id() + 1;
# endif
#endif

  order_smp_mem();
}

ALWAYS_INLINE void __arch_lock_release(struct __arch_lock_s *lock)
{
  order_smp_mem();
#if defined(CONFIG_HEXO_LOCK_DEBUG)
# if defined(CONFIG_ARCH_SMP)
  assert(lock->cpu_id == cpu_id() + 1);
  lock->cpu_id = 0;
# else
  assert(lock->pc != NULL);
# endif
  lock->pc = NULL;
#endif

#if defined(CONFIG_ARCH_SMP)
  lock->a = 0;
#endif

  order_smp_write();
}

#endif

