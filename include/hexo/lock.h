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

#ifndef LOCK_H_
#define LOCK_H_

#include "types.h"
#include "error.h"
#include "interrupt.h"

struct arch_lock_s;

static error_t arch_lock_init(struct arch_lock_s *lock);
static void arch_lock_destroy(struct arch_lock_s *lock);
static __bool_t arch_lock_try(struct arch_lock_s *lock);
static void arch_lock_spin(struct arch_lock_s *lock);
static __bool_t arch_lock_state(struct arch_lock_s *lock);
static void arch_lock_release(struct arch_lock_s *lock);

#include "arch/hexo/lock.h"

struct			lock_s
{
#ifdef CONFIG_SMP
  /** architecture specific lock data */
  struct arch_lock_s	arch;
#endif
  /** interrupt state save */
  __reg_t			irq_state;
};

typedef struct lock_s	lock_t;

#define LOCK_INITIALIZER	{ .arch = ARCH_LOCK_INITIALIZER }

/** allocate a new lock and return associated atomic memory location */
static inline error_t lock_init(lock_t *lock)
{
#ifdef CONFIG_SMP
  return arch_lock_init(&lock->arch);
#else
  return 0;
#endif
}

/** free lock ressources */
static inline void lock_destroy(lock_t *lock)
{
#ifdef CONFIG_SMP
  return arch_lock_destroy(&lock->arch);
#endif
}

/** try to take lock */
static inline __bool_t lock_try(lock_t *lock)
{
#ifdef CONFIG_SMP
  return arch_lock_try(&lock->arch);
#else
  return 0;
#endif
}

/** spin to take lock */
static inline void lock_spin(lock_t *lock)
{
#ifdef CONFIG_SMP
  arch_lock_spin(&lock->arch);
#endif
}

/** return current lock state */
static inline __bool_t lock_state(lock_t *lock)
{
#ifdef CONFIG_SMP
  return arch_lock_state(&lock->arch);
#else
  return 0;
#endif
}

/** save interrupts state, disable interrupts, and spin to take lock */
static inline void lock_spin_irq(lock_t *lock)
{
  __reg_t		state;

  cpu_interrupt_savestate_disable(&state);
#ifdef CONFIG_SMP
  arch_lock_spin(&lock->arch);
#endif
  lock->irq_state = state;
}

/** release lock */
static inline void lock_release(lock_t *lock)
{
#ifdef CONFIG_SMP
  arch_lock_release(&lock->arch);
#endif
}

/** release lock and restore previous interrupts state */
static inline void lock_release_irq(lock_t *lock)
{
  __reg_t		state = lock->irq_state;

#ifdef CONFIG_SMP
  arch_lock_release(&lock->arch);
#endif
  cpu_interrupt_restorestate(&state);
}

#endif

