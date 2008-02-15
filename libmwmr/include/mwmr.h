/*
 * This file is distributed under the terms of the GNU General Public
 * License.
 * 
 * Copyright (c) UPMC / Lip6
 *     2005-2008, Nicolas Pouillon, <nipo@ssji.net>
 */

#ifndef MWMR_H_
#define MWMR_H_

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
};

#define MWMR_INITIALIZER(w, d, b)						   \
	{														   \
		.width = w,											   \
		.depth = d,											   \
		.gdepth = (w)*(d),									   \
		.buffer = (void*)b,									   \
		.lock = PTHREAD_MUTEX_INITIALIZER,				   \
		.nempty = PTHREAD_COND_INITIALIZER,				   \
		.nfull = PTHREAD_COND_INITIALIZER,				   \
		.rptr = (void*)b,									   \
		.wptr = (void*)b,									   \
		.end = (uint8_t*)b+(w)*(d),						   \
		.usage = 0,										   \
	}

void mwmr_read( mwmr_s*, void *, size_t );
void mwmr_write( mwmr_s*, void *, size_t );

ssize_t mwmr_try_read( mwmr_s*, void *, size_t );
ssize_t mwmr_try_write( mwmr_s*, void *, size_t );

#endif
