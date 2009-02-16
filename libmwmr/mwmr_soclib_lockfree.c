/*
 * This file is part of MutekH.
 * 
 * MutekH is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; version 2.1 of the License.
 * 
 * MutekH is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with MutekH; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Copyright (c) UPMC, Lip6, SoC
 *         Nicolas Pouillon <nipo@ssji.net>, 2008
 */

#include <mutek/scheduler.h>
#include <hexo/types.h>
#include <hexo/atomic.h>
#include <hexo/endian.h>
#include <hexo/interrupt.h>
#include <string.h>
#include <mwmr/mwmr.h>

#if defined(CONFIG_SRL) && !defined(CONFIG_PTHREAD)
# include <srl/srl_sched_wait.h>
# include <srl/srl_log.h>
# ifndef SRL_VERBOSITY
#  define SRL_VERBOSITY VERB_DEBUG
# endif
#elif defined(CONFIG_PTHREAD)
# include <pthread.h>
#endif

void
mwmr_hw_init( void *coproc, enum SoclibMwmrWay way,
			  size_t no, const mwmr_t* mwmr )
{
	volatile uint32_t *c = coproc;
	c[MWMR_CONFIG_FIFO_WAY] = endian_le32(way);
	c[MWMR_CONFIG_FIFO_NO] = endian_le32(no);
	c[MWMR_CONFIG_STATUS_ADDR] = endian_le32((uintptr_t)mwmr->status);
	c[MWMR_CONFIG_WIDTH] = endian_le32(mwmr->width);
	c[MWMR_CONFIG_DEPTH] = endian_le32(mwmr->gdepth);
	c[MWMR_CONFIG_BUFFER_ADDR] = endian_le32((uintptr_t)mwmr->buffer);
	c[MWMR_CONFIG_RUNNING] = endian_le32(1);
}

void mwmr_config( void *coproc, size_t no, const uint32_t val )
{
	volatile uint32_t *c = coproc;
	c[no] = endian_le32(val);
}

uint32_t mwmr_status( void *coproc, size_t no )
{
	volatile uint32_t *c = coproc;
	return endian_le32(c[no]);
}

#ifdef __mips

// returns substracted value
static inline uint32_t
cpu_atomic_sub_minz(volatile uint32_t *a, uint32_t val)
{
	uint32_t  result, temp, temp2;

  asm volatile(
	       "1:     ll      %2, %3		     \n"
		   "       slt     %0, %2, %5             \n"
		   "       movn    %1, %2, %0             \n"
		   "       movz    %1, %5, %0             \n"
		   // if *a <  val: %0 = 1 then %1 = *a
		   // if *a >= val: %0 = 0 then %1 = val
	       "       subu    %0, %2, %1             \n"
	       "       sc      %0, %3                \n"
	       "       beqz    %0, 1b                \n"
	       : "=&r" (temp), "=&r" (result), "=&r" (temp2), "=m" (*a)
	       : "m" (*a), "r" (val)
	       );

  return result;
}

// returns previous value
static inline uint32_t
cpu_atomic_add(volatile uint32_t *a, uint32_t val)
{
  uint32_t  result, temp;

  asm volatile(
	       "1:     ll      %1, %2		     \n"
	       "       addu    %0, %1, %4            \n"
	       "       sc      %0, %2                \n"
	       "       beqz    %0, 1b                \n"
	       : "=&r" (temp), "=&r" (result), "=m" (*a)
	       : "m" (*a), "r" (val)
	       );

  return result;
}

// returns previous value
static inline uint32_t
cpu_atomic_add_wrap(volatile uint32_t *a, uint32_t val, uint32_t mod)
{
	uint32_t  result, temp, temp2, temp3;

  asm volatile(
	       "1:     ll      %2, %4		     \n"
	       "       addu    %1, %2, %6            \n"
	       "       subu    %0, %1, %7            \n"
		   "       slt     %3, %1, %7            \n"
		   "       movn    %0, %1, %3            \n"
	       "       sc      %0, %4                \n"
	       "       beqz    %0, 1b                \n"
	       : "=&r" (temp2), "=&r" (temp), "=&r" (result)
		   , "=&r" (temp3), "=m" (*a)
	       : "m" (*a), "r" (val), "r" (mod)
	       );

  return result;
}

// 0: failed, 1: done
static inline uint32_t
cpu_atomic_wait_and_swap(volatile uint32_t *a, uint32_t old, uint32_t new)
{
	uint32_t  result, temp;

  asm volatile(
	       ".set push   \n"
	       ".set noreorder   \n"
	       "1:     ll      %0, %1		     \n"
		   "       bne     %0, %3, 1f        \n"
	       "       or      %0, %4, $0         \n"
	       "       sc      %0, %1                \n"
	       ".set pop   \n"
	       "       beqz    %0, 1b                \n"
	       : "=&r" (result), "=m" (*a)
	       : "m" (*a), "r" (old), "r" (new)
	       );

  return result;
}

#else
# error Only MIPS is supported !
#endif

void mwmr_read( mwmr_t *fifo, void *_ptr, size_t lensw )
{
	size_t done = 0;

	while ( done < lensw ) {
		uint32_t xfer_size = cpu_atomic_sub_minz(&fifo->status->data_size, lensw);

		if ( xfer_size == 0 ) {
#if defined(CONFIG_SRL) && !defined(CONFIG_PTHREAD)
			srl_sched_wait_ne_le(&fifo->status->data_size, 0);
#elif defined(CONFIG_PTHREAD)
			pthread_yield();
#else
			cpu_interrupt_disable();
			sched_context_switch();
			cpu_interrupt_enable();
#endif
			continue;
		}
		uint32_t rptr = cpu_atomic_add_wrap(&fifo->status->data_tail, xfer_size, fifo->gdepth);
		uint32_t wptr;

		if ( rptr + xfer_size > fifo->gdepth ) {
			size_t tx1 = fifo->gdepth - rptr;
			size_t tx2 = xfer_size - tx1;

			cpu_dcache_invld_buf(&((uint8_t*)fifo->buffer)[rptr], tx1);
			cpu_dcache_invld_buf(fifo->buffer, tx2);
			memcpy( _ptr, &((uint8_t*)fifo->buffer)[rptr], tx1 );
			memcpy( (uint8_t*)_ptr + tx1, fifo->buffer, tx2 );
			wptr = tx2;
		} else {
			cpu_dcache_invld_buf(&((uint8_t*)fifo->buffer)[rptr], xfer_size);
			memcpy( _ptr, &((uint8_t*)fifo->buffer)[rptr], xfer_size );
			wptr = rptr + xfer_size;
			if ( wptr == fifo->gdepth )
				wptr = 0;
		}
		cpu_atomic_wait_and_swap(&fifo->status->free_head, rptr, wptr);
		cpu_atomic_add(&fifo->status->free_size, xfer_size);
		done += xfer_size;
	}
}


void mwmr_write( mwmr_t *fifo, const void *_ptr, size_t lensw )
{
	size_t done = 0;

	while ( done < lensw ) {
		uint32_t xfer_size = cpu_atomic_sub_minz(&fifo->status->free_size, lensw);

		if ( xfer_size == 0 ) {
#if defined(CONFIG_SRL) && !defined(CONFIG_PTHREAD)
			srl_sched_wait_ne_le(&fifo->status->free_size, 0);
#elif defined(CONFIG_PTHREAD)
			pthread_yield();
#else
			cpu_interrupt_disable();
			sched_context_switch();
			cpu_interrupt_enable();
#endif
			continue;
		}
		uint32_t rptr = cpu_atomic_add_wrap(&fifo->status->free_tail, xfer_size, fifo->gdepth);
		uint32_t wptr;

		if ( rptr + xfer_size > fifo->gdepth ) {
			size_t tx1 = fifo->gdepth - rptr;
			size_t tx2 = xfer_size - tx1;

			cpu_dcache_invld_buf(&((uint8_t*)fifo->buffer)[rptr], tx1);
			cpu_dcache_invld_buf(fifo->buffer, tx2);
			memcpy( &((uint8_t*)fifo->buffer)[rptr], _ptr, tx1 );
			memcpy( fifo->buffer, (uint8_t*)_ptr + tx1, tx2 );
			wptr = tx2;
		} else {
			cpu_dcache_invld_buf(&((uint8_t*)fifo->buffer)[rptr], xfer_size);
			memcpy( &((uint8_t*)fifo->buffer)[rptr], _ptr, xfer_size );
			wptr = rptr + xfer_size;
			if ( wptr == fifo->gdepth )
				wptr = 0;
		}
		cpu_atomic_wait_and_swap(&fifo->status->data_head, rptr, wptr);
		cpu_atomic_add(&fifo->status->data_size, xfer_size);
		done += xfer_size;
	}
}

size_t mwmr_try_read( mwmr_t *fifo, void *_ptr, size_t lensw )
{
	size_t done = 0;

	while ( done < lensw ) {
		uint32_t xfer_size = cpu_atomic_sub_minz(&fifo->status->data_size, lensw);
		if ( xfer_size == 0 )
			break;
		uint32_t rptr = cpu_atomic_add_wrap(&fifo->status->data_tail, xfer_size, fifo->gdepth);
		uint32_t wptr;
		if ( rptr + xfer_size > fifo->gdepth ) {
			size_t tx1 = fifo->gdepth - rptr;
			size_t tx2 = xfer_size - tx1;
			cpu_dcache_invld_buf(&((uint8_t*)fifo->buffer)[rptr], tx1);
			cpu_dcache_invld_buf(fifo->buffer, tx2);
			memcpy( _ptr, &((uint8_t*)fifo->buffer)[rptr], tx1 );
			memcpy( (uint8_t*)_ptr + tx1, fifo->buffer, tx2 );
			wptr = tx2;
		} else {
			cpu_dcache_invld_buf(&((uint8_t*)fifo->buffer)[rptr], xfer_size);
			memcpy( _ptr, &((uint8_t*)fifo->buffer)[rptr], xfer_size );
			wptr = rptr + xfer_size;
			if ( wptr == fifo->gdepth )
				wptr = 0;
		}
		cpu_atomic_wait_and_swap(&fifo->status->free_head, rptr, wptr);
		cpu_atomic_add(&fifo->status->free_size, xfer_size);
		done += xfer_size;
	}
	return done;
}

size_t mwmr_try_write( mwmr_t *fifo, const void *_ptr, size_t lensw )
{
	size_t done = 0;

	while ( done < lensw ) {
		uint32_t xfer_size = cpu_atomic_sub_minz(&fifo->status->free_size, lensw);
		if ( xfer_size == 0 )
			break;
		uint32_t rptr = cpu_atomic_add_wrap(&fifo->status->free_tail, xfer_size, fifo->gdepth);
		uint32_t wptr;
		if ( rptr + xfer_size > fifo->gdepth ) {
			size_t tx1 = fifo->gdepth - rptr;
			size_t tx2 = xfer_size - tx1;
			cpu_dcache_invld_buf(&((uint8_t*)fifo->buffer)[rptr], tx1);
			cpu_dcache_invld_buf(fifo->buffer, tx2);
			memcpy( &((uint8_t*)fifo->buffer)[rptr], _ptr, tx1 );
			memcpy( fifo->buffer, (uint8_t*)_ptr + tx1, tx2 );
			wptr = tx2;
		} else {
			cpu_dcache_invld_buf(&((uint8_t*)fifo->buffer)[rptr], xfer_size);
			memcpy( &((uint8_t*)fifo->buffer)[rptr], _ptr, xfer_size );
			wptr = rptr + xfer_size;
			if ( wptr == fifo->gdepth )
				wptr = 0;
		}
		cpu_atomic_wait_and_swap(&fifo->status->data_head, rptr, wptr);
		cpu_atomic_add(&fifo->status->data_size, xfer_size);
		done += xfer_size;
	}
	return done;
}
