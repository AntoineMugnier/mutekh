/*
 * This file is part of DSX, development environment for static
 * SoC applications.
 * 
 * This file is distributed under the terms of the GNU General Public
 * License.
 * 
 * Copyright (c) 2006, Nicolas Pouillon, <nipo@ssji.net>
 *     Laboratoire d'informatique de Paris 6 / ASIM, France
 * 
 *  $Id$
 */

#ifndef SRL_LOCK_H_
#define SRL_LOCK_H_

#ifndef CONFIG_PTHREAD
#include <hexo/lock.h>

static inline void srl_lock_lock( srl_lock_t lock )
{
	lock_spin(lock);
}

static inline void srl_lock_unlock( srl_lock_t lock )
{
	lock_release(lock);
}

static inline uint32_t srl_lock_try_lock( srl_lock_t lock )
{
	return lock_try(lock);
}

#else
#include <pthread.h>

static inline void srl_lock_lock( srl_lock_t lock )
{
	pthread_mutex_lock(lock);
}

static inline void srl_lock_unlock( srl_lock_t lock )
{
	pthread_mutex_unlock(lock);
}

static inline uint32_t srl_lock_try_lock( srl_lock_t lock )
{
	return pthread_mutex_trylock(lock);
}

#endif /* CONFIG_PTHREAD */

#endif
