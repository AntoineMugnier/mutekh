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
  @file
  @module {Core::Hardware abstraction layer}
  @short Spinlock stuff
  
  Spinlock operations include memory barriers to ensure the consistency of
  memory accesses on weakly-ordered memory architectures.
 */

#ifndef LOCK_H_
#define LOCK_H_

#include <hexo/decls.h>
#include <hexo/ordering.h>

C_HEADER_BEGIN

#include "types.h"
#include "error.h"
#include "interrupt.h"

struct __arch_lock_s;

#include <arch/hexo/lock.h>

struct			lock_s
{
#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
  /** architecture specific lock data */
  struct __arch_lock_s	arch;
#endif
};

typedef struct lock_s	lock_t;

struct			lock_irq_s
{
#ifdef CONFIG_HEXO_IRQ
  cpu_irq_state_t	__interrupt_state;
#endif
#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
  /** architecture specific lock data */
  struct __arch_lock_s	arch;
#endif
};

typedef struct lock_irq_s lock_irq_t;

#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
# define LOCK_INITIALIZER	{ .arch = ARCH_LOCK_INITIALIZER }
#else
# define LOCK_INITIALIZER	{ }
#endif

/** allocate a new lock */
ALWAYS_INLINE error_t lock_init(lock_t *lock)
{
#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
  return __arch_lock_init(&lock->arch);
#else
  return 0;
#endif
}

/** allocate a new lock with interrupt state */
ALWAYS_INLINE error_t lock_init_irq(lock_irq_t *lock)
{
#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
  return __arch_lock_init(&lock->arch);
#else
  return 0;
#endif
}


/** @this frees lock ressources */
ALWAYS_INLINE void lock_destroy(lock_t *lock)
{
#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
  return __arch_lock_destroy(&lock->arch);
#endif
}

/** @this frees lock ressources */
ALWAYS_INLINE void lock_destroy_irq(lock_irq_t *lock)
{
#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
  return __arch_lock_destroy(&lock->arch);
#endif
}


/** @this tries to take lock */
ALWAYS_INLINE bool_t lock_try(lock_t *lock)
{
#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
  order_smp_mem();
  return __arch_lock_try(&lock->arch);
#else
  return 0;
#endif
}


/** @this spins to take the lock */
ALWAYS_INLINE void lock_spin(lock_t *lock)
{
#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
  order_smp_mem();
  __arch_lock_spin(&lock->arch);
#endif
}

/** @This disables interrupts, spins to take the lock and store the
    previous irq state in @tt *lock */
ALWAYS_INLINE void lock_spin_irq(lock_irq_t *lock)
{
#ifdef CONFIG_HEXO_IRQ
  cpu_irq_state_t state;
  cpu_interrupt_savestate_disable(&state);
#endif
#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
  order_smp_mem();
  __arch_lock_spin(&lock->arch);
#endif
#ifdef CONFIG_HEXO_IRQ
  lock->__interrupt_state = state;
#endif
}

/** @This disables interrupts, spins to take the lock and store the
    previous irq state in @tt *irq_state */
ALWAYS_INLINE void lock_spin_irq2(lock_t *lock, cpu_irq_state_t *irq_state)
{
#ifdef CONFIG_HEXO_IRQ
  cpu_irq_state_t state;
  cpu_interrupt_savestate_disable(&state);
#endif
#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
  order_smp_mem();
  __arch_lock_spin(&lock->arch);
#endif
#ifdef CONFIG_HEXO_IRQ
  *irq_state = state;
#endif
}


/** @this saves interrupts state, disables interrupts, and spins to take
    lock. This macro must be matched with the LOCK_RELEASE_IRQ macro. */
#ifdef CONFIG_HEXO_IRQ
# define LOCK_SPIN_IRQ(lock)					\
  HEXO_ATOMIC_SCOPE_BEGIN                                       \
  cpu_irq_state_t	__interrupt_state;					\
  cpu_interrupt_savestate_disable(&__interrupt_state);		\
  lock_spin(lock);
#else
# define LOCK_SPIN_IRQ(lock)					\
  HEXO_ATOMIC_SCOPE_BEGIN                                       \
  lock_spin(lock);
#endif

/** @this releases a lock */
ALWAYS_INLINE void lock_release(lock_t *lock)
{
#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
  order_smp_mem();
  __arch_lock_release(&lock->arch);
#endif
}

/** @This releases a lock and restore previous interrupt state */
ALWAYS_INLINE void lock_release_irq(lock_irq_t *lock)
{
#ifdef CONFIG_HEXO_IRQ
  cpu_irq_state_t state = lock->__interrupt_state;
#endif
#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
  order_smp_mem();
  __arch_lock_release(&lock->arch);
#endif
#ifdef CONFIG_HEXO_IRQ
  cpu_interrupt_restorestate(&state);
#endif
}

/** @This releases a lock and restore interrupt state from @tt *irq_state */
ALWAYS_INLINE void lock_release_irq2(lock_t *lock, const cpu_irq_state_t *irq_state)
{
#ifdef CONFIG_HEXO_IRQ
  cpu_irq_state_t state = *irq_state;
#endif
#if defined(CONFIG_ARCH_SMP) || defined(CONFIG_HEXO_LOCK_DEBUG)
  order_smp_mem();
  __arch_lock_release(&lock->arch);
#endif
#ifdef CONFIG_HEXO_IRQ
  cpu_interrupt_restorestate(&state);
#endif
}

/** @this releases a lock and restore previous interrupts state. This macro
    must be matched with the LOCK_SPIN_IRQ macro. @multiple */
#ifdef CONFIG_HEXO_IRQ
# define LOCK_RELEASE_IRQ(lock)					\
  lock_release(lock);						\
  cpu_interrupt_restorestate(&__interrupt_state);		\
  HEXO_ATOMIC_SCOPE_END

#else
# define LOCK_RELEASE_IRQ(lock)					\
  lock_release(lock);                                           \
  HEXO_ATOMIC_SCOPE_END

#endif

C_HEADER_END

#endif

