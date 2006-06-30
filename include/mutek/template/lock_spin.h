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

#ifndef __LOCK_MUTEK_H_
#define __LOCK_MUTEK_H_

#include <mutek/error.h>
#include <mutek/lock.h>

#include "container.h"

/*************************************************************
 *	`SPIN_IRQ' lock functions to use with CONTAINER_LOCKED
 */

#define __CONT_SPIN_IRQ_FIELD(field)	lock_t field

static inline error_t
__CONT_SPIN_IRQ_INIT(lock_t *lock)
{
  return lock_init(lock);
}

static inline void
__CONT_SPIN_IRQ_DESTROY(lock_t *lock)
{
  lock_destroy(lock);
}

static inline void
__CONT_SPIN_IRQ_WRLOCK(lock_t *lock)
{
  lock_spin_irq(lock);
}

static inline void
__CONT_SPIN_IRQ_RDLOCK(lock_t *lock)
{
  lock_spin_irq(lock);
}

static inline void
__CONT_SPIN_IRQ_UNLOCK(lock_t *lock)
{
  lock_release_irq(lock);
}

/*************************************************************
 *	`SPIN' lock functions to use with CONTAINER_LOCKED
 */

#define __CONT_SPIN_FIELD(field)	lock_t field

static inline error_t
__CONT_SPIN_INIT(lock_t *lock)
{
  return lock_init(lock);
}

static inline void
__CONT_SPIN_DESTROY(lock_t *lock)
{
  lock_destroy(lock);
}

static inline void
__CONT_SPIN_WRLOCK(lock_t *lock)
{
  lock_spin(lock);
}

static inline void
__CONT_SPIN_RDLOCK(lock_t *lock)
{
  lock_spin(lock);
}

static inline void
__CONT_SPIN_UNLOCK(lock_t *lock)
{
  lock_release(lock);
}

#endif

