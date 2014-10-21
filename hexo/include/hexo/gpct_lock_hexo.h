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
 * @file
 * @module{Hexo}
 * @internal
 * @short Hexo specific platform definition for GPCT
 */

#ifndef __GPCT_LOCK_HEXO_H__
#define __GPCT_LOCK_HEXO_H__

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <gct_platform.h>

#include <hexo/lock.h>

/*************************************************************
 *	HEXO_LOCK mutex lock functions to use with container
 */

#define GPCT_LOCK_HEXO_LOCK_INITIALIZER	LOCK_INITIALIZER
#define gpct_lock_HEXO_LOCK_initializer	LOCK_INITIALIZER

typedef lock_t gpct_lock_HEXO_LOCK_type_t;

ALWAYS_INLINE gpct_error_t
gpct_lock_HEXO_LOCK_init(lock_t *lock)
{
  return lock_init(lock);
}

ALWAYS_INLINE void
gpct_lock_HEXO_LOCK_destroy(lock_t *lock)
{
  lock_destroy(lock);
}

ALWAYS_INLINE void
gpct_lock_HEXO_LOCK_wrlock(lock_t *lock)
{
  lock_spin(lock);
}

ALWAYS_INLINE void
gpct_lock_HEXO_LOCK_rdlock(lock_t *lock)
{
  lock_spin(lock);
}

ALWAYS_INLINE void
gpct_lock_HEXO_LOCK_unlock(lock_t *lock)
{
  lock_release(lock);
}

/*************************************************************
 *	HEXO_LOCK_IRQ mutex lock functions to use with container
 */

#define GPCT_LOCK_HEXO_LOCK_IRQ_INITIALIZER	LOCK_INITIALIZER
#define gpct_lock_HEXO_LOCK_IRQ_initializer	LOCK_INITIALIZER

typedef struct 
{
  lock_t lock;
  reg_t irq;
} gpct_lock_HEXO_LOCK_IRQ_type_t;

ALWAYS_INLINE gpct_error_t
gpct_lock_HEXO_LOCK_IRQ_init(gpct_lock_HEXO_LOCK_IRQ_type_t *lock)
{
  return lock_init(&lock->lock);
}

ALWAYS_INLINE void
gpct_lock_HEXO_LOCK_IRQ_destroy(gpct_lock_HEXO_LOCK_IRQ_type_t *lock)
{
  lock_destroy(&lock->lock);
}

ALWAYS_INLINE void
gpct_lock_HEXO_LOCK_IRQ_wrlock(gpct_lock_HEXO_LOCK_IRQ_type_t *lock)
{
  cpu_interrupt_savestate_disable(&lock->irq);
  lock_spin(&lock->lock);
}

ALWAYS_INLINE void
gpct_lock_HEXO_LOCK_IRQ_rdlock(gpct_lock_HEXO_LOCK_IRQ_type_t *lock)
{
  gpct_lock_HEXO_LOCK_IRQ_wrlock(lock);
}

ALWAYS_INLINE void
gpct_lock_HEXO_LOCK_IRQ_unlock(gpct_lock_HEXO_LOCK_IRQ_type_t *lock)
{
  lock_release(&lock->lock);
  cpu_interrupt_restorestate(&lock->irq);
}

C_HEADER_END

#endif

