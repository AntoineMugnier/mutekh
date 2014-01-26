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
  @module{Hexo}
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
#ifdef CONFIG_ARCH_SMP
  /** architecture specific lock data */
  struct __arch_lock_s	arch;
#endif
};

typedef struct lock_s	lock_t;

struct			lock_irq_s
{
#ifdef CONFIG_HEXO_IRQ
  reg_t	__interrupt_state;
#endif
#ifdef CONFIG_ARCH_SMP
  /** architecture specific lock data */
  struct __arch_lock_s	arch;
#endif
};

typedef struct lock_irq_s lock_irq_t;

#ifdef CONFIG_ARCH_SMP
# define LOCK_INITIALIZER	{ .arch = ARCH_LOCK_INITIALIZER }
#else
# define LOCK_INITIALIZER	{ }
#endif

/** allocate a new lock */
static inline error_t lock_init(lock_t *lock)
{
#ifdef CONFIG_ARCH_SMP
  return __arch_lock_init(&lock->arch);
#else
  return 0;
#endif
}

/** allocate a new lock with interrupt state */
static inline error_t lock_init_irq(lock_irq_t *lock)
{
#ifdef CONFIG_ARCH_SMP
  return __arch_lock_init(&lock->arch);
#else
  return 0;
#endif
}


/** @this frees lock ressources */
static inline void lock_destroy(lock_t *lock)
{
#ifdef CONFIG_ARCH_SMP
  return __arch_lock_destroy(&lock->arch);
#endif
}

/** @this frees lock ressources */
static inline void lock_destroy_irq(lock_irq_t *lock)
{
#ifdef CONFIG_ARCH_SMP
  return __arch_lock_destroy(&lock->arch);
#endif
}


/** @this tries to take lock */
static inline bool_t lock_try(lock_t *lock)
{
#ifdef CONFIG_ARCH_SMP
  order_smp_mem();
  return __arch_lock_try(&lock->arch);
#else
  return 0;
#endif
}


/** @this spins to take the lock */
static inline void lock_spin(lock_t *lock)
{
#ifdef CONFIG_ARCH_SMP
  order_smp_mem();
  __arch_lock_spin(&lock->arch);
#endif
}

/** @this save and disable interrupts then spins to take the lock */
static inline void lock_spin_irq(lock_irq_t *lock)
{
#ifdef CONFIG_HEXO_IRQ
  cpu_interrupt_savestate_disable(&lock->__interrupt_state);
#endif
#ifdef CONFIG_ARCH_SMP
  order_smp_mem();
  __arch_lock_spin(&lock->arch);
#endif
}


/** @this returns the current lock state */
static inline bool_t lock_state(lock_t *lock)
{
#ifdef CONFIG_ARCH_SMP
  return __arch_lock_state(&lock->arch);
#else
  return 0;
#endif
}

/** @this saves interrupts state, disables interrupts, and spins to take
    lock. This macro must be matched with the LOCK_RELEASE_IRQ macro. */
#ifdef CONFIG_HEXO_IRQ
# define LOCK_SPIN_IRQ(lock)					\
{								\
  reg_t	__interrupt_state;					\
  cpu_interrupt_savestate_disable(&__interrupt_state);		\
  lock_spin(lock);
#else
# define LOCK_SPIN_IRQ(lock)					\
  lock_spin(lock);
#endif

/** @this releases a lock */
static inline void lock_release(lock_t *lock)
{
#ifdef CONFIG_ARCH_SMP
  order_smp_mem();
  __arch_lock_release(&lock->arch);
#endif
}

/** @this releases a lock and restore previous interrupt state */
static inline void lock_release_irq(lock_irq_t *lock)
{
#ifdef CONFIG_ARCH_SMP
  order_smp_mem();
  __arch_lock_release(&lock->arch);
#endif
#ifdef CONFIG_HEXO_IRQ
  cpu_interrupt_restorestate(&lock->__interrupt_state);
#endif
}

/** @this releases a lock and restore previous interrupts state. This macro
    must be matched with the LOCK_SPIN_IRQ macro. @multiple */
#ifdef CONFIG_HEXO_IRQ
# define LOCK_RELEASE_IRQ(lock)					\
  lock_release(lock);						\
  cpu_interrupt_restorestate(&__interrupt_state);		\
}

# define LOCK_RELEASE_IRQ_X(lock)				\
  lock_release(lock);						\
  cpu_interrupt_restorestate(&__interrupt_state);

#else
# define LOCK_RELEASE_IRQ(lock)					\
  lock_release(lock);

# define LOCK_RELEASE_IRQ_X(lock)				\
  lock_release(lock);
#endif

C_HEADER_END

#endif

