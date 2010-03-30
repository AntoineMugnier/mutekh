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
#include <hexo/cpu.h>

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
	uintptr_t c = (uintptr_t)coproc;
	cpu_mem_write_32( c + sizeof(uint32_t) * MWMR_CONFIG_FIFO_WAY, endian_le32(way));
	cpu_mem_write_32( c + sizeof(uint32_t) * MWMR_CONFIG_FIFO_NO, endian_le32(no));
	cpu_mem_write_32( c + sizeof(uint32_t) * MWMR_CONFIG_STATUS_ADDR, endian_le32((uintptr_t)mwmr->status));
	cpu_mem_write_32( c + sizeof(uint32_t) * MWMR_CONFIG_WIDTH, endian_le32(mwmr->width));
	cpu_mem_write_32( c + sizeof(uint32_t) * MWMR_CONFIG_DEPTH, endian_le32(mwmr->gdepth));
	cpu_mem_write_32( c + sizeof(uint32_t) * MWMR_CONFIG_BUFFER_ADDR, endian_le32((uintptr_t)mwmr->buffer));
	cpu_mem_write_32( c + sizeof(uint32_t) * MWMR_CONFIG_RUNNING, endian_le32(1));
	cpu_mem_write_32( c + sizeof(uint32_t) * MWMR_CONFIG_ENDIANNESS, 0x11223344);
}

void mwmr_config( void *coproc, size_t no, const uint32_t val )
{
	uintptr_t c = (uintptr_t)coproc;
	cpu_mem_write_32( c + sizeof(uint32_t) * no, val);
}

uint32_t mwmr_status( void *coproc, size_t no )
{
	uintptr_t c = (uintptr_t)coproc;
	return cpu_mem_read_32( c + sizeof(uint32_t) * no );
}

#if defined(CONFIG_CPU_MIPS)

// returns substracted value
static inline uint32_t
cpu_atomic_sub_minz(uint32_t *a, uint32_t val)
{
	uint32_t  result, cmp_to_store, before;

  asm volatile(
	       "1:     ll      %[before], %[a_in]		     \n"
		   "       slt     %[cmp_to_store], %[before], %[val]             \n"
		   "       movn    %[result], %[before], %[cmp_to_store]             \n"
		   "       movz    %[result], %[val], %[cmp_to_store]             \n"
		   // if *a <  val: %[cmp_to_store] = 1 then %[result] = *a
		   // if *a >= val: %[cmp_to_store] = 0 then %[result] = val
	       "       subu    %[cmp_to_store], %[before], %[result]             \n"
	       "       sc      %[cmp_to_store], %[a_out]                \n"
	       "       beqz    %[cmp_to_store], 1b                \n"
	       : [cmp_to_store] "=&r" (cmp_to_store), [result] "=&r" (result)
		   , [before] "=&r" (before), [a_out] "=m" (*a)
	       : [a_in] "m" (*a), [val] "r" (val)
	       );

  return result;
}

// returns previous value
static inline uint32_t
cpu_atomic_add(uint32_t *a, uint32_t val)
{
  uint32_t  result, temp;

  asm volatile(
	       "1:     ll      %[result], %[a_in]		     \n"
	       "       addu    %[temp], %[result], %[val]            \n"
	       "       sc      %[temp], %[a_out]                \n"
	       "       beqz    %[temp], 1b                \n"
	       : [temp] "=&r" (temp), [result] "=&r" (result), [a_out] "=m" (*a)
	       : [a_in] "m" (*a), [val] "r" (val)
	       );

  return result;
}

// returns previous value
static inline uint32_t
cpu_atomic_add_wrap(uint32_t *a, uint32_t val, uint32_t mod)
{
	uint32_t  before, before_plus_val, before_plus_val_minus_mod, compar;

  asm volatile(
	       "1:     ll      %[before], %[a_in]		     \n"
	       "       addu    %[before_plus_val], %[before], %[val]            \n"
	       "       subu    %[before_plus_val_minus_mod], %[before_plus_val], %[mod]            \n"
		   "       slt     %[compar], %[before_plus_val], %[mod]            \n"
		   "       movn    %[before_plus_val_minus_mod], %[before_plus_val], %[compar]            \n"
	       "       sc      %[before_plus_val_minus_mod], %[a_out]                \n"
	       "       beqz    %[before_plus_val_minus_mod], 1b                \n"
	       : [before_plus_val_minus_mod] "=&r" (before_plus_val_minus_mod)
		   , [before_plus_val] "=&r" (before_plus_val)
		   , [before] "=&r" (before)
		   , [compar] "=&r" (compar)
		   , [a_out] "=m" (*a)
	       : [a_in] "m" (*a), [val] "r" (val), [mod] "r" (mod)
	       );

  return before;
}

static inline void
cpu_atomic_wait_and_swap(uint32_t *a, uint32_t old, uint32_t new)
{
	uint32_t  result;

  asm volatile(
	       ".set push   \n"
	       ".set noreorder   \n"
	       "1:     ll      %[result], %[a_in]		     \n"
		   "       bne     %[result], %[old], 1b        \n"
	       "       move    %[result], %[new]         \n"
	       "       sc      %[result], %[a_out]                \n"
	       ".set pop   \n"
	       "       beqz    %[result], 1b                \n"
	       : [result] "=&r" (result), [a_out] "=m" (*a)
	       : [a_in] "m" (*a), [old] "r" (old), [new] "r" (new)
	       );
}

# else // CONFIG_CPU_MIPS not defined

// returns substracted value
static inline uint32_t
cpu_atomic_sub_minz(uint32_t *a, uint32_t _tosub)
{
	uint32_t oldval;
	uint32_t tosub;

	do {
		cpu_dcache_invld(a);
		oldval = *a;
		tosub = _tosub;
		if ( tosub > oldval )
			tosub = oldval;

		if ( tosub == 0 )
			return 0;
	} while ( ! atomic_compare_and_swap((atomic_int_t*)a, oldval, oldval-tosub) );

	return tosub;
}

// returns previous value
static inline uint32_t
cpu_atomic_add(uint32_t *a, uint32_t val)
{
	uint32_t oldval;

	do {
		cpu_dcache_invld(a);
		oldval = *a;
	} while ( ! atomic_compare_and_swap((atomic_int_t*)a, oldval, oldval+val) );

	return oldval;
}

// returns previous value
static inline uint32_t
cpu_atomic_add_wrap(uint32_t *a, uint32_t val, uint32_t mod)
{
	uint32_t oldval, newval;

	do {
		cpu_dcache_invld(a);
		oldval = *a;
		newval = (oldval+val);
		if ( newval >= mod )
			newval -= mod;
	} while ( ! atomic_compare_and_swap((atomic_int_t*)a, oldval, newval ) );

	return oldval;
}

static inline void
cpu_atomic_wait_and_swap(uint32_t *a, uint32_t old, uint32_t new)
{
	do {
	} while ( ! atomic_compare_and_swap((atomic_int_t*)a, old, new ) );
}

#endif

size_t mwmr_read_unit(
	soclib_mwmr_status_s *status,
	const uint8_t *mwmr_buffer,
	const size_t gdepth,
	uint8_t *user_buffer,
	size_t user_len)
{
#if MWMR_DEBUG
	printk("mwmr_read_unit(status = %p, mwmr_buffer = %p, gdepth = %d, user_buf = %p, user_len = %d)\n",
		   status, mwmr_buffer, (int)gdepth, user_buffer, (int)user_len);
#endif

	assert(mwmr_buffer);
	assert(user_buffer);
	if ( user_len > gdepth )
		user_len = gdepth;

	uint32_t xfer_size = cpu_atomic_sub_minz(&status->data_size, user_len);
	if ( xfer_size == 0 )
		return 0;

	assert( xfer_size <= gdepth );
	assert( xfer_size <= user_len );

	uint32_t rptr = cpu_atomic_add_wrap(&status->data_tail, xfer_size, gdepth);
	uint32_t future_rptr;

	assert( rptr < gdepth );

	if ( rptr + xfer_size > gdepth ) {
		size_t tx1 = gdepth - rptr;
		size_t tx2 = xfer_size - tx1;

		cpu_dcache_invld_buf((char*)mwmr_buffer + rptr, tx1);
		cpu_dcache_invld_buf((char*)mwmr_buffer, tx2);
		memcpy( user_buffer, mwmr_buffer + rptr, tx1 );
		memcpy( user_buffer + tx1, mwmr_buffer, tx2 );
		future_rptr = tx2;
	} else {
		cpu_dcache_invld_buf((char*)mwmr_buffer + rptr, xfer_size);
		memcpy( user_buffer, mwmr_buffer + rptr, xfer_size );
		future_rptr = rptr + xfer_size;
		if ( future_rptr == gdepth )
			future_rptr = 0;
	}
	assert( future_rptr < gdepth );
	cpu_atomic_wait_and_swap(&status->free_head, rptr, future_rptr);
	cpu_atomic_add(&status->free_size, xfer_size);
	return xfer_size;
}

size_t mwmr_write_unit(
	soclib_mwmr_status_s *status,
	uint8_t *mwmr_buffer,
	const size_t gdepth,
	const uint8_t *user_buffer,
	size_t user_len)
{
#if MWMR_DEBUG
	printk("mwmr_write_unit(status = %p, mwmr_buffer = %p, gdepth = %d, user_buf = %p, user_len = %d)\n",
		   status, mwmr_buffer, (int)gdepth, user_buffer, (int)user_len);
#endif

	assert(mwmr_buffer);
	assert(user_buffer);
	if ( user_len > gdepth )
		user_len = gdepth;

	uint32_t xfer_size = cpu_atomic_sub_minz(&status->free_size, user_len);
	if ( xfer_size == 0 )
		return 0;

	assert( xfer_size <= gdepth );
	assert( xfer_size <= user_len );

	uint32_t wptr = cpu_atomic_add_wrap(&status->free_tail, xfer_size, gdepth);
	uint32_t future_wptr;

	assert( wptr < gdepth );

	if ( wptr + xfer_size > gdepth ) {
		size_t tx1 = gdepth - wptr;
		size_t tx2 = xfer_size - tx1;

		memcpy( mwmr_buffer + wptr, user_buffer, tx1 );
		memcpy( mwmr_buffer, user_buffer + tx1, tx2 );
		future_wptr = tx2;
	} else {
		memcpy( mwmr_buffer + wptr, user_buffer, xfer_size );
		future_wptr = wptr + xfer_size;
		if ( future_wptr == gdepth )
			future_wptr = 0;
	}
	assert( future_wptr < gdepth );
	cpu_atomic_wait_and_swap(&status->data_head, wptr, future_wptr);
	cpu_atomic_add(&status->data_size, xfer_size);
	return xfer_size;
}

void mwmr_read( mwmr_t *fifo, void *_ptr, size_t lensw )
{
#if MWMR_DEBUG
	printk("mwmr_read(fifo = %p, status = %p, user_buf = %p, user_len = %d)\n",
		   fifo, fifo->status, _ptr, (int)lensw);
#endif
	size_t done = 0;
	uint8_t *ptr = _ptr;
	const uint8_t *buffer = fifo->buffer;
	const size_t gdepth = fifo->gdepth;

	while ( done < lensw ) {
		size_t xfer_size = mwmr_read_unit(
			fifo->status, buffer, gdepth,
			ptr+done, lensw-done);

		if ( xfer_size == 0 ) {
#if defined(CONFIG_SRL) && !defined(CONFIG_PTHREAD)
			srl_sched_wait_ne_cpu(&fifo->status->data_size, 0);
#elif defined(CONFIG_PTHREAD)
			pthread_yield();
#else
			cpu_interrupt_disable();
			sched_context_switch();
			cpu_interrupt_enable();
#endif
			continue;
		}

		ptr += xfer_size;
		done += xfer_size;
	}
}

void mwmr_write( mwmr_t *fifo, const void *_ptr, size_t lensw )
{
#if MWMR_DEBUG
	printk("mwmr_write(fifo = %p, status = %p, user_buf = %p, user_len = %d)\n",
		   fifo, fifo->status, _ptr, (int)lensw);
#endif
	size_t done = 0;
	const uint8_t *ptr = _ptr;
	uint8_t *buffer = fifo->buffer;
	const size_t gdepth = fifo->gdepth;

	while ( done < lensw ) {
		size_t xfer_size = mwmr_write_unit(
			fifo->status, buffer, gdepth,
			ptr+done, lensw-done);

		if ( xfer_size == 0 ) {
#if defined(CONFIG_SRL) && !defined(CONFIG_PTHREAD)
			srl_sched_wait_ne_cpu(&fifo->status->free_size, 0);
#elif defined(CONFIG_PTHREAD)
			pthread_yield();
#else
			cpu_interrupt_disable();
			sched_context_switch();
			cpu_interrupt_enable();
#endif
			continue;
		}

		ptr += xfer_size;
		done += xfer_size;
	}
}

size_t mwmr_try_read( mwmr_t *fifo, void *_ptr, size_t lensw )
{
#if MWMR_DEBUG
	printk("mwmr_try_read(fifo = %p, status = %p, user_buf = %p, user_len = %d)\n",
		   fifo, fifo->status, _ptr, (int)lensw);
#endif
	uint8_t *ptr = _ptr;
	const uint8_t *buffer = fifo->buffer;
	const size_t gdepth = fifo->gdepth;

	return mwmr_read_unit(
		fifo->status, buffer, gdepth,
		ptr, lensw);
}

size_t mwmr_try_write( mwmr_t *fifo, const void *_ptr, size_t lensw )
{
#if MWMR_DEBUG
	printk("mwmr_write_write(fifo = %p, status = %p, user_buf = %p, user_len = %d)\n",
		   fifo, fifo->status, _ptr, (int)lensw);
#endif
	const uint8_t *ptr = _ptr;
	uint8_t *buffer = fifo->buffer;
	const size_t gdepth = fifo->gdepth;

	return mwmr_write_unit(
		fifo->status, buffer, gdepth,
		ptr, lensw);
}
