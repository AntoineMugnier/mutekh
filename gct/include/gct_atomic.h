/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014
*/

/**
 * @file
 * @module{Generic C templates}
 * @internal
 * @short atomic operations for GCT
 */

#ifndef __GCT_ATOMIC_H__
#define __GCT_ATOMIC_H__

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <hexo/atomic.h>

#define GCT_CONFIG_ATOMIC_OTHER

typedef atomic_int_t gct_atomic_int_t;
typedef atomic_t gct_atomic_t;

/* static value init */
#define GCT_ATOMIC_INITIALIZER(v)	ATOMIC_INITIALIZER(v)

ALWAYS_INLINE void gct_atomic_init(gct_atomic_t *a)
{
}

ALWAYS_INLINE void gct_atomic_destroy(gct_atomic_t *a)
{
}

ALWAYS_INLINE void gct_atomic_set(gct_atomic_t *a, gct_atomic_int_t v)
{
  atomic_set(a, v);
}

ALWAYS_INLINE gct_atomic_int_t gct_atomic_get(gct_atomic_t *a)
{
  return atomic_get(a);
}

ALWAYS_INLINE gct_bool_t gct_atomic_inc(gct_atomic_t *a)
{
  return atomic_inc(a);
}

ALWAYS_INLINE gct_bool_t gct_atomic_dec(gct_atomic_t *a)
{
  return atomic_dec(a);
}

ALWAYS_INLINE gct_bool_t gct_atomic_bit_test(gct_atomic_t *a, gct_atomic_int_t n)
{
  return atomic_bit_test(a, n);
}

ALWAYS_INLINE gct_bool_t gct_atomic_bit_test_set(gct_atomic_t *a, gct_atomic_int_t n)
{
  return atomic_bit_testset(a, n);
}

ALWAYS_INLINE gct_bool_t gct_atomic_bit_test_clr(gct_atomic_t *a, gct_atomic_int_t n)
{
  return atomic_bit_testclr(a, n);
}

ALWAYS_INLINE void gct_atomic_bit_clr(gct_atomic_t *a, gct_atomic_int_t n)
{
  return atomic_bit_clr(a, n);
}

ALWAYS_INLINE void gct_atomic_bit_set(gct_atomic_t *a, gct_atomic_int_t n)
{
  return atomic_bit_set(a, n);
}

C_HEADER_END

#endif

