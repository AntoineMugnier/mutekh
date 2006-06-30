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

#ifndef __TEMPLATE_LOCK_PTHREAD_H_
#define __TEMPLATE_LOCK_PTHREAD_H_

#include <mutek/error.h>
#include <pthread.h>

#include "container.h"

/*************************************************************
 *	PTHREAD_MUTEX lock functions to use with CONTAINER_LOCKED
 */

#define __CONT_PTHREAD_MUTEX_FIELD(field)	pthread_mutex_t field

static inline error_t
__CONT_PTHREAD_MUTEX_INIT(pthread_mutex_t *lock)
{
  return pthread_mutex_init(lock, 0);
}

static inline void
__CONT_PTHREAD_MUTEX_DESTROY(pthread_mutex_t *lock)
{
  pthread_mutex_destroy(lock);
}

static inline void
__CONT_PTHREAD_MUTEX_WRLOCK(pthread_mutex_t *lock)
{
  pthread_mutex_lock(lock);
}

static inline void
__CONT_PTHREAD_MUTEX_RDLOCK(pthread_mutex_t *lock)
{
  pthread_mutex_lock(lock);
}

static inline void
__CONT_PTHREAD_MUTEX_UNLOCK(pthread_mutex_t *lock)
{
  pthread_mutex_unlock(lock);
}

#endif

