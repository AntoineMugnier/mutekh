/*
 * This file is distributed under the terms of the GNU General Public
 * License.
 * 
 * Copyright (c) UPMC / Lip6
 *     2005-2008, Nicolas Pouillon, <nipo@ssji.net>
 */

#ifndef MWMR_H_
#define MWMR_H_

typedef struct mwmr_s mwmr_t;

#if defined CONFIG_MWMR_PTHREAD

struct mwmr_s {
	size_t width;
	size_t depth;
	size_t gdepth;
	void *buffer;
	pthread_mutex_t lock;
	pthread_cond_t nempty;
	pthread_cond_t nfull;
	uint8_t *rptr, *wptr, *end;
	size_t usage;
	const char *const name;
};

typedef struct {} srl_mwmr_lock_t;
#define MWMR_LOCK_INITIALIZER {}

#define MWMR_INITIALIZER(w, d, b, st, n, l)					   \
	{														   \
		.width = w,											   \
		.depth = d,											   \
		.gdepth = (w)*(d),									   \
		.buffer = (void*)b,									   \
		.lock = PTHREAD_MUTEX_INITIALIZER,				       \
		.nempty = PTHREAD_COND_INITIALIZER,				       \
		.nfull = PTHREAD_COND_INITIALIZER,				       \
		.rptr = (void*)b,									   \
		.wptr = (void*)b,									   \
		.end = (uint8_t*)b+(w)*(d),						       \
		.usage = 0,										       \
		.name = n,											   \
	}

#elif defined CONFIG_MWMR_SOCLIB

#include <soclib/mwmr_controller.h>

#ifdef CONFIG_MWMR_USE_RAMLOCKS
typedef volatile uint32_t srl_mwmr_lock_t;
#define MWMR_LOCK_INITIALIZER 0
#endif

struct mwmr_s {
	size_t width;
	size_t depth;
	size_t gdepth;
	void *buffer;
	soclib_mwmr_status_s *status;
	const char *const name;
#ifdef CONFIG_MWMR_INSTRUMENTATION
	uint32_t n_read;
	uint32_t n_write;
	uint32_t time_read;
	uint32_t time_write;
#endif
#ifdef CONFIG_MWMR_USE_RAMLOCKS
	volatile srl_mwmr_lock_t *lock;
#endif
};

void mwmr_hw_init( void *coproc, enum SoclibMwmrWay way,
				   size_t no, const mwmr_t* mwmr );

#ifdef CONFIG_MWMR_USE_RAMLOCKS

# define MWMR_INITIALIZER(w, d, b, st, n, l)				   \
	{														   \
		.width = w,											   \
		.depth = d,											   \
		.gdepth = (w)*(d),									   \
		.buffer = (void*)b,									   \
		.status = st,									   	   \
		.name = n,											   \
		.lock = l,											   \
	}
#else

typedef struct {} srl_mwmr_lock_t;
#define MWMR_LOCK_INITIALIZER {}

# define MWMR_INITIALIZER(w, d, b, st, n, l)				   \
	{														   \
		.width = w,											   \
		.depth = d,											   \
		.gdepth = (w)*(d),									   \
		.buffer = (void*)b,									   \
		.status = st,									   	   \
		.name = n,											   \
	}
#endif

#ifdef CONFIG_MWMR_INSTRUMENTATION
void mwmr_dump_stats( const mwmr_t *mwmr );
void mwmr_clear_stats( mwmr_t *mwmr );
#endif

#else
# error No valid MWMR implementation
#endif

void mwmr_read( mwmr_t*, void *, size_t );
void mwmr_write( mwmr_t*, const void *, size_t );

size_t mwmr_try_read( mwmr_t*, void *, size_t );
size_t mwmr_try_write( mwmr_t*, const void *, size_t );

#endif
