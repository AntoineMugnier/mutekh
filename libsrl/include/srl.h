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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2007
    Copyright Nicola Pouillon <nipo@ssji.net> (c) 2007

*/

#ifndef SRL_H_
#define SRL_H_

#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/lock.h>
#include <hexo/scheduler.h>

typedef uint32_t srl_word_t;

typedef struct srl_memspace_s * srl_memspace_t;


enum {
    SRL_VERB_NONE,
    SRL_VERB_TRACE,
    SRL_VERB_DEBUG,
    SRL_VERB_MAX,
};

#define SRL_VERB(x) SRL_VERB_##x

#define srl_log( lev, c )			\
do {						\
    if ( SRL_VERB(lev) <= SRL_VERB(CONFIG_SRL_VERBOSITY) )	\
        puts(c);				\
} while(0)

#define srl_log_printf( lev, c... )		\
do {						\
    if ( SRL_VERB(lev) <= SRL_VERB(CONFIG_SRL_VERBOSITY) )	\
        printf(c);				\
} while(0)

#define srl_assert(x) assert(x)

#define srl_busy_cycles(n) do {} while (0)

static inline void srl_yield()
{
  cpu_interrupt_disable();
  sched_context_switch();
  cpu_interrupt_enable();
}

/******************************* locks */

#ifdef CONFIG_SRL_SPINLOCKS

typedef struct arch_lock_s * srl_lock_t;

static inline void srl_lock_reset(srl_lock_t l)
{
  arch_lock_init(l);
}

static inline void srl_lock_lock(srl_lock_t l)
{
  arch_lock_spin(l);
}

static inline void srl_lock_unlock(srl_lock_t l)
{
  arch_lock_release(l);
}

static inline void srl_lock_lock_b(srl_lock_t l)
{
  arch_lock_spin(l);
}

static inline error_t srl_lock_trylock(srl_lock_t l)
{
  arch_lock_try(spinlock) ? 1 : 0;
}

#define SRL_LOCK_INITIALIZER ARCH_LOCK_INITIALIZER

#else

typedef struct srl_lock_s * srl_lock_t;

void srl_lock_reset(srl_lock_t l);
void srl_lock_lock(srl_lock_t l);
void srl_lock_unlock(srl_lock_t l);
void srl_lock_lock_b(srl_lock_t l);
error_t srl_lock_trylock(srl_lock_t l);

#define SRL_LOCK_INITIALIZER							      \
{										      \
  .count = 0,									      \
  .wait = CONTAINER_ROOT_INITIALIZER(sched_queue, __SCHED_CONTAINER_ALGO, HEXO_SPIN), \
}

#endif

/******************************* MWMR */

#define SRL_MWMR_STATUS_INITIALIZER		\
{						\
  .lock = SRL_LOCK_INITIALIZER,			\
  .usage = 0,					\
  .readp = 0,					\
  .writep = 0					\
}

#define SRL_MWMR_INITIALIZER
{
}

typedef struct srl_mwmr_s * srl_mwmr_t;

void srl_mwmr_reset(srl_mwmr_t mwmr);
void srl_mwmr_read(srl_mwmr_t mwmr, void *data, srl_word_t size);
void srl_mwmr_write(srl_mwmr_t mwmr, const void *data, srl_word_t size);
srl_word_t srl_mwmr_try_read(srl_mwmr_t mwmr, void *data, srl_word_t size);
srl_word_t srl_mwmr_try_write(srl_mwmr_t mwmr, const void *data, srl_word_t size);

/******************************* barrier */

# define SRL_BARRIER_INITIALIZER(n)						       \
  {										       \
    .count = n,										\
    .wait = CONTAINER_ROOT_INITIALIZER(sched_queue, __SCHED_CONTAINER_ALGO, HEXO_SPIN), \
  }

typedef struct srl_barrier_s * srl_barrier_t;

void srl_barrier_reset(srl_barrier_t t);
void srl_barrier_wait(srl_barrier_t t);

#endif

