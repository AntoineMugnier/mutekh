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
	soclib_io_set( coproc, MWMR_CONFIG_FIFO_WAY, way );
	soclib_io_set( coproc, MWMR_CONFIG_FIFO_NO, no );
	soclib_io_set( coproc, MWMR_CONFIG_STATUS_ADDR, mwmr->status );
	soclib_io_set( coproc, MWMR_CONFIG_DEPTH, mwmr->gdepth );
	soclib_io_set( coproc, MWMR_CONFIG_WIDTH, mwmr->width );
	soclib_io_set( coproc, MWMR_CONFIG_BUFFER_ADDR, mwmr->buffer );
	soclib_io_set( coproc, MWMR_CONFIG_RUNNING, 1 );
}

void mwmr_config( void *coproc, size_t no, const uint32_t val )
{
	// assert(no < MWMR_IOREG_MAX);
	soclib_io_set( coproc, no, val );
}

uint32_t mwmr_status( void *coproc, size_t no )
{
	// assert(no < MWMR_IOREG_MAX);
	return soclib_io_get( coproc, no );
}

static inline void mwmr_lock( uint32_t *lock )
{
#if 0
	while (cpu_atomic_bit_testset((atomic_int_t*)lock, 0)) {
		printf("Didnt got the lock, switching\n");
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

void mwmr_read( mwmr_t *fifo, void *_ptr, size_t lensw )
{
	uint8_t *ptr = _ptr;
    volatile soclib_mwmr_status_s *const status = fifo->status;

	mwmr_lock( &status->lock );
    while ( lensw ) {
        size_t len;
        while (status->usage < fifo->width) {
            mwmr_unlock( &status->lock );
			printf("Not enough data for read, switching\n");
			cpu_interrupt_disable();
			sched_context_switch();
			cpu_interrupt_enable();
            mwmr_lock( &status->lock );
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
        }
    }
	mwmr_unlock( &status->lock );
	printf("Successful read\n");
}

void mwmr_write( mwmr_t *fifo, const void *_ptr, size_t lensw )
{
	uint8_t *ptr = _ptr;
    volatile soclib_mwmr_status_s *const status = fifo->status;

	mwmr_lock( &status->lock );
    while ( lensw ) {
        size_t len;
        while (status->usage >= fifo->gdepth) {
            mwmr_unlock( &status->lock );
			printf("Not enough room for write, switching\n");
			cpu_interrupt_disable();
			sched_context_switch();
			cpu_interrupt_enable();
            mwmr_lock( &status->lock );
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
        }
    }
	mwmr_unlock( &status->lock );
	printf("Successful write\n");
}

size_t mwmr_try_read( mwmr_t *fifo, void *_ptr, size_t lensw )
{
	uint8_t *ptr = _ptr;
	size_t done = 0;
    volatile soclib_mwmr_status_s *const status = fifo->status;

	if ( mwmr_try_lock( &status->lock ) )
		return done;
    while ( lensw ) {
        size_t len;
        if (status->usage == 0) {
            mwmr_unlock( &status->lock );
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
	mwmr_unlock( &status->lock );
	return done;
}

size_t mwmr_try_write( mwmr_t *fifo, const void *_ptr, size_t lensw )
{
	uint8_t *ptr = _ptr;
	size_t done = 0;
    volatile soclib_mwmr_status_s *const status = fifo->status;

	if ( mwmr_try_lock( &status->lock ) )
		return done;
    while ( lensw ) {
        size_t len;
        while (status->usage >= fifo->gdepth) {
            mwmr_unlock( &status->lock );
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
	mwmr_unlock( &status->lock );
	return done;
}
