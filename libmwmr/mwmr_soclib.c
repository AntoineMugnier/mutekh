/*
 *
 * Copyright (c) UPMC, Lip6, SoC
 *         Nicolas Pouillon <nipo@ssji.net>, 2008
 */

#include <hexo/scheduler.h>
#include <hexo/types.h>
#include <hexo/atomic.h>
#include <hexo/interrupt.h>
#include <string.h>
#include <mwmr/mwmr.h>
#include <soclib/mwmr_controller.h>

static inline size_t min(size_t a, size_t b)
{
	if ( a < b )
		return a;
	else
		return b;
}

void
mwmr_hw_init( void *coproc, enum SoclibMwmrWay way,
			  size_t no, const mwmr_t* mwmr )
{
	volatile uint32_t *c = coproc;
	c[MWMR_CONFIG_FIFO_WAY] = endian_le32(way);
	c[MWMR_CONFIG_FIFO_NO] = endian_le32(no);
	c[MWMR_CONFIG_STATUS_ADDR] = endian_le32((uintptr_t)mwmr->status);
	c[MWMR_CONFIG_DEPTH] = endian_le32(mwmr->gdepth);
	c[MWMR_CONFIG_BUFFER_ADDR] = endian_le32((uintptr_t)mwmr->buffer);
	c[MWMR_CONFIG_RUNNING] = endian_le32(1);
}

void mwmr_config( void *coproc, size_t no, const uint32_t val )
{
	// assert(no < MWMR_IOREG_MAX);
	volatile uint32_t *c = coproc;
	c[no] = endian_le32(val);
}

uint32_t mwmr_status( void *coproc, size_t no )
{
	// assert(no < MWMR_IOREG_MAX);
	volatile uint32_t *c = coproc;
	return endian_le32(c[no]);
}

static inline void mwmr_lock( uint32_t *lock )
{
#if 0
	while (cpu_atomic_bit_testset((atomic_int_t*)lock, 0)) {
		cpu_interrupt_disable();
		sched_context_switch();
		cpu_interrupt_enable();
	}
#else
	cpu_atomic_bit_waitset((atomic_int_t*)lock, 0);
#endif
}

static inline uint32_t mwmr_try_lock( uint32_t *lock )
{
	return cpu_atomic_bit_testset((atomic_int_t*)lock, 0);
}

static inline void mwmr_unlock( uint32_t *lock )
{
	*lock = 0;
}

typedef struct {
	uint32_t usage, wptr, rptr, modified;
} local_mwmr_status_t;

static inline void rehash_status( mwmr_t *fifo, local_mwmr_status_t *status )
{
	volatile soclib_mwmr_status_s *fstatus = fifo->status;
	cpu_dcache_invld_buf(fstatus, sizeof(*fstatus));
	status->usage = fstatus->usage;
	status->wptr = fstatus->wptr;
	status->rptr = fstatus->rptr;
	status->modified = 0;
}

static inline void writeback_status( mwmr_t *fifo, local_mwmr_status_t *status )
{
	volatile soclib_mwmr_status_s *fstatus = fifo->status;
	if ( !status->modified )
		return;
	fstatus->usage = status->usage;
	fstatus->wptr = status->wptr;
	fstatus->rptr = status->rptr;
}

void mwmr_read( mwmr_t *fifo, void *_ptr, size_t lensw )
{
	uint8_t *ptr = _ptr;
	local_mwmr_status_t status;

	mwmr_lock( &fifo->status->lock );
	rehash_status( fifo, &status );
    while ( lensw ) {
        size_t len;
		while ( status.usage < fifo->width ) {
			writeback_status( fifo, &status );
            mwmr_unlock( &fifo->status->lock );
			cpu_interrupt_disable();
			sched_context_switch();
			cpu_interrupt_enable();
            mwmr_lock( &fifo->status->lock );
			rehash_status( fifo, &status );
        }
        while ( lensw && status.usage >= fifo->width ) {
			void *sptr;

            if ( status.rptr < status.wptr )
                len = status.usage;
            else
                len = (fifo->gdepth - status.rptr);
            len = min(len, lensw);
			sptr = &((uint8_t*)fifo->buffer)[status.rptr];
			cpu_dcache_invld_buf(sptr, len);
            memcpy( ptr, sptr, len );
            status.rptr += len;
            if ( status.rptr == fifo->gdepth )
                status.rptr = 0;
            ptr += len;
            status.usage -= len;
            lensw -= len;
			status.modified = 1;
        }
    }
	writeback_status( fifo, &status );
	mwmr_unlock( &fifo->status->lock );
}

void mwmr_write( mwmr_t *fifo, const void *_ptr, size_t lensw )
{
	uint8_t *ptr = _ptr;
    local_mwmr_status_t status;

	mwmr_lock( &fifo->status->lock );
	rehash_status( fifo, &status );
    while ( lensw ) {
        size_t len;
        while (status.usage >= fifo->gdepth) {
			writeback_status( fifo, &status );
            mwmr_unlock( &fifo->status->lock );
			cpu_interrupt_disable();
			sched_context_switch();
			cpu_interrupt_enable();
            mwmr_lock( &fifo->status->lock );
			rehash_status( fifo, &status );
        }
        while ( lensw && status.usage < fifo->gdepth ) {
			void *dptr;

            if ( status.rptr <= status.wptr )
                len = (fifo->gdepth - status.wptr);
            else
                len = fifo->gdepth - status.usage;
            len = min(len, lensw);
			dptr = &((uint8_t*)fifo->buffer)[status.wptr];
            memcpy( dptr, ptr, len );
            status.wptr += len;
            if ( status.wptr == fifo->gdepth )
                status.wptr = 0;
            ptr += len;
            status.usage += len;
            lensw -= len;
			status.modified = 1;
        }
    }
	writeback_status( fifo, &status );
	mwmr_unlock( &fifo->status->lock );
}

size_t mwmr_try_read( mwmr_t *fifo, void *_ptr, size_t lensw )
{
	uint8_t *ptr = _ptr;
	size_t done = 0;
    volatile soclib_mwmr_status_s *const status = fifo->status;

	if ( mwmr_try_lock( &fifo->status->lock ) )
		return done;
    while ( lensw ) {
        size_t len;
        if (status->usage == 0) {
            mwmr_unlock( &fifo->status->lock );
			return done;
        }
        while ( lensw && status->usage >= fifo->width ) {
			void *sptr;

            if ( status->rptr < status->wptr )
                len = status->usage;
            else
                len = (fifo->gdepth - status->rptr);
            len = min(len, lensw);
			sptr = &((uint8_t*)fifo->buffer)[status->rptr];
            memcpy( ptr, sptr, len );
            status->rptr += len;
            if ( status->rptr == fifo->gdepth )
                status->rptr = 0;
            ptr += len;
            status->usage -= len;
            lensw -= len;
			done += len;
        }
    }
	mwmr_unlock( &fifo->status->lock );
	return done;
}

size_t mwmr_try_write( mwmr_t *fifo, const void *_ptr, size_t lensw )
{
	uint8_t *ptr = _ptr;
	size_t done = 0;
    volatile soclib_mwmr_status_s *const status = fifo->status;

	if ( mwmr_try_lock( &fifo->status->lock ) )
		return done;
    while ( lensw ) {
        size_t len;
        while (status->usage >= fifo->gdepth) {
            mwmr_unlock( &fifo->status->lock );
			return done;
        }
        while ( lensw && status->usage < fifo->gdepth ) {
			void *dptr;

            if ( status->rptr <= status->wptr )
                len = (fifo->gdepth - status->wptr);
            else
                len = fifo->gdepth - status->usage;
            len = min(len, lensw);
			dptr = &((uint8_t*)fifo->buffer)[status->wptr];
            memcpy( dptr, ptr, len );
            status->wptr += len;
            if ( status->wptr == fifo->gdepth )
                status->wptr = 0;
            ptr += len;
            status->usage += len;
            lensw -= len;
			done += len;
        }
    }
	mwmr_unlock( &fifo->status->lock );
	return done;
}
