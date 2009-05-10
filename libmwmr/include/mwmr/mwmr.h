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

# ifdef CONFIG_MWMR_LOCKFREE

enum SoclibMwmrRegisters {
    MWMR_IOREG_MAX = 16,
    MWMR_RESET = MWMR_IOREG_MAX,
    MWMR_CONFIG_FIFO_WAY,
    MWMR_CONFIG_FIFO_NO,
    MWMR_CONFIG_STATUS_ADDR,
    MWMR_CONFIG_DEPTH, // bytes
    MWMR_CONFIG_BUFFER_ADDR,
    MWMR_CONFIG_RUNNING,
    MWMR_CONFIG_WIDTH, // bytes
    MWMR_CONFIG_ENDIANNESS, // Write 0x11223344 here
    MWMR_FIFO_FILL_STATUS,
};

enum SoclibMwmrWay {
    MWMR_TO_COPROC,
    MWMR_FROM_COPROC,
};

typedef struct
{
	uint32_t free_tail; // bytes
	uint32_t free_head; // bytes
	uint32_t free_size; // bytes

	uint32_t data_tail; // bytes
	uint32_t data_head; // bytes
	uint32_t data_size; // bytes
} soclib_mwmr_status_s;

#define SOCLIB_MWMR_STATUS_INITIALIZER(w, d) {0,0,(w*d),0,0,0}

# else /* not CONFIG_MWMR_LOCKFREE */

enum SoclibMwmrRegisters {
    MWMR_IOREG_MAX = 16,
    MWMR_RESET = MWMR_IOREG_MAX,
    MWMR_CONFIG_FIFO_WAY,
    MWMR_CONFIG_FIFO_NO,
    MWMR_CONFIG_STATUS_ADDR,
    MWMR_CONFIG_DEPTH,
    MWMR_CONFIG_BUFFER_ADDR,
    MWMR_CONFIG_LOCK_ADDR,
    MWMR_CONFIG_RUNNING,
    MWMR_CONFIG_WIDTH,
    MWMR_FIFO_FILL_STATUS,
};

enum SoclibMwmrWay {
    MWMR_TO_COPROC,
    MWMR_FROM_COPROC,
};

typedef struct
{
	uint32_t rptr;
	uint32_t wptr;
	uint32_t usage;
	uint32_t lock;
} soclib_mwmr_status_s;

#define SOCLIB_MWMR_STATUS_INITIALIZER(w,d) {0,0,0,0}

# endif /* CONFIG_MWMR_LOCKFREE */

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
