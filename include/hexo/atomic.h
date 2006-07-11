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

#ifndef ATOMIC_H_
#define ATOMIC_H_

#include "types.h"
#include "lock.h"

/** atomicaly increment value in memory */
static __bool_t cpu_atomic_inc(__atomic_t *a);
/** atomicaly decrement value in memory */
static __bool_t cpu_atomic_dec(__atomic_t *a);
/** set bit in memory */
static void cpu_atomic_bit_set(__atomic_t *a, uint_fast8_t n);
/** atomicaly test and set bit in memory */
static __bool_t cpu_atomic_bit_testset(__atomic_t *a, uint_fast8_t n);
/** atomicaly wait for bit clear and set it */
static void cpu_atomic_bit_waitset(__atomic_t *a, uint_fast8_t n);
/** clear bit in memory */
static void cpu_atomic_bit_clr(__atomic_t *a, uint_fast8_t n);
/** atomicaly test and clear bit in memory */
static __bool_t cpu_atomic_bit_testclr(__atomic_t *a, uint_fast8_t n);
/** atomicaly wait for bit set and clear it */
static void cpu_atomic_bit_waitclr(__atomic_t *a, uint_fast8_t n);
/** test bit in memory */
static __bool_t cpu_atomic_bit_test(__atomic_t *a, uint_fast8_t n);

#include "cpu/hexo/atomic.h"

#endif

