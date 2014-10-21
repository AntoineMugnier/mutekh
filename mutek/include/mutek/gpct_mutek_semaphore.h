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
 * @module{Mutek}
 * @internal
 * @short Semaphore for GPCT
 */

#ifndef __GPCT_MUTEKH_SEMAPHORE_H__
#define __GPCT_MUTEKH_SEMAPHORE_H__

#include <mutek/semaphore.h>

/*************************************************************
 *	MUTEK_SEMAPHORE lock functions to use with container
 */

#define GPCT_LOCK_MUTEK_SEMAPHORE_INITIALIZER	SEMAPHORE_INITIALIZER
#define gpct_lock_MUTEK_SEMAPHORE_initializer	SEMAPHORE_INITIALIZER

typedef struct semaphore_s gpct_lock_MUTEK_SEMAPHORE_type_t;

ALWAYS_INLINE gpct_error_t
gpct_lock_MUTEK_SEMAPHORE_init(struct semaphore_s *lock)
{
  return semaphore_init(lock, 1);
}

ALWAYS_INLINE void
gpct_lock_MUTEK_SEMAPHORE_destroy(struct semaphore_s *lock)
{
  semaphore_destroy(lock);
}

ALWAYS_INLINE void
gpct_lock_MUTEK_SEMAPHORE_wrlock(struct semaphore_s *lock)
{
  semaphore_take(lock, 1);
}

ALWAYS_INLINE void
gpct_lock_MUTEK_SEMAPHORE_rdlock(struct semaphore_s *lock)
{
  semaphore_take(lock, 1);
}

ALWAYS_INLINE void
gpct_lock_MUTEK_SEMAPHORE_unlock(struct semaphore_s *lock)
{
  semaphore_give(lock, 1);
}

#endif

