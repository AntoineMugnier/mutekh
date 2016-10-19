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

/**
   @file CPU atomic operations
*/

#if !defined(ATOMIC_H_) || defined(CPU_ATOMIC_H_)
#error This file can not be included directly
#else

#define CPU_ATOMIC_H_

#define HAS_CPU_ATOMIC_SWAP

ALWAYS_INLINE atomic_int_t
__cpu_atomic_swap(atomic_int_t *a, atomic_int_t value)
{
  asm volatile("swap [%3], %0        \n"
               : "=r" (value), "+m" (*a)
               : "0" (value), "r" (a)
               );

  return value;
}

#if defined(CONFIG_CPU_SPARC_LEON3_CASA) || defined(CONFIG_CPU_SPARC_SOCLIB)
# define HAS_CPU_ATOMIC_COMPARE_AND_SWAP

ALWAYS_INLINE bool_t
__cpu_atomic_compare_and_swap(atomic_int_t *a, atomic_int_t old, atomic_int_t future)
{
  asm volatile("casa [%4] 0, %3, %0        \n"
               : "=r" (future), "+m" (*a)
               : "0" (future), "r" (old), "r" (a)
               );

  return future == old;
}
#endif

#endif

