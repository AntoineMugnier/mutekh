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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef STDINT_H_
#define STDINT_H_

#include <hexo/decls.h>

C_HEADER_BEGIN

/**
 * @file
 * @module {Core::C library}
 */

#include <hexo/types.h>

/**
 * @multiple
 * @showcontent
 * This macro defines an integer type limit
 */

#define INT8_MIN		__MINOF_TYPE(int8_t)
#define INT16_MIN		__MINOF_TYPE(int16_t)
#define INT32_MIN		__MINOF_TYPE(int32_t)
#define INT64_MIN		__MINOF_TYPE(int64_t)
#define UINT8_MIN		__MINOF_TYPE(uint8_t)
#define UINT16_MIN		__MINOF_TYPE(uint16_t)
#define UINT32_MIN		__MINOF_TYPE(uint32_t)
#define UINT64_MIN		__MINOF_TYPE(uint64_t)

#define INT8_MAX		__MAXOF_TYPE(int8_t)
#define INT16_MAX		__MAXOF_TYPE(int16_t)
#define INT32_MAX		__MAXOF_TYPE(int32_t)
#define INT64_MAX		__MAXOF_TYPE(int64_t)
#define UINT8_MAX		__MAXOF_TYPE(uint8_t)
#define UINT16_MAX		__MAXOF_TYPE(uint16_t)
#define UINT32_MAX		__MAXOF_TYPE(uint32_t)
#define UINT64_MAX		__MAXOF_TYPE(uint64_t)

#define INT_LEAST8_MIN		__MINOF_TYPE(int_least8_t)
#define INT_LEAST16_MIN		__MINOF_TYPE(int_least16_t)
#define INT_LEAST32_MIN		__MINOF_TYPE(int_least32_t)
#define INT_LEAST64_MIN		__MINOF_TYPE(int_least64_t)
#define INT_LEAST8_MAX		__MAXOF_TYPE(int_least8_t)
#define INT_LEAST16_MAX		__MAXOF_TYPE(int_least16_t)
#define INT_LEAST32_MAX		__MAXOF_TYPE(int_least32_t)
#define INT_LEAST64_MAX		__MAXOF_TYPE(int_least64_t)
#define UINT_LEAST8_MAX		__MAXOF_TYPE(uint_least8_t)
#define UINT_LEAST16_MAX	__MAXOF_TYPE(uint_least16_t)
#define UINT_LEAST32_MAX	__MAXOF_TYPE(uint_least32_t)
#define UINT_LEAST64_MAX	__MAXOF_TYPE(uint_least64_t)

#define INT_FAST8_MIN		__MINOF_TYPE(int_fast8_t)
#define INT_FAST16_MIN		__MINOF_TYPE(int_fast16_t)
#define INT_FAST32_MIN		__MINOF_TYPE(int_fast32_t)
#define INT_FAST64_MIN		__MINOF_TYPE(int_fast64_t)
#define INT_FAST8_MAX		__MAXOF_TYPE(int_fast8_t)
#define INT_FAST16_MAX		__MAXOF_TYPE(int_fast16_t)
#define INT_FAST32_MAX		__MAXOF_TYPE(int_fast32_t)
#define INT_FAST64_MAX		__MAXOF_TYPE(int_fast64_t)
#define UINT_FAST8_MAX		__MAXOF_TYPE(uint_fast8_t)
#define UINT_FAST16_MAX		__MAXOF_TYPE(uint_fast16_t)
#define UINT_FAST32_MAX		__MAXOF_TYPE(uint_fast32_t)
#define UINT_FAST64_MAX		__MAXOF_TYPE(uint_fast64_t)

#define INTPTR_MIN		__MINOF_TYPE(intptr_t)
#define INTPTR_MAX		__MAXOF_TYPE(intptr_t)
#define UINTPTR_MAX		__MAXOF_TYPE(uintptr_t)

#define INTMAX_MIN		__MINOF_TYPE(intmax_t)
#define INTMAX_MAX		__MAXOF_TYPE(intmax_t)
#define UINTMAX_MAX		__MAXOF_TYPE(uintmax_t)

#define PTRDIFF_MIN		__MINOF_TYPE(ptrdiff_t)
#define PTRDIFF_MAX		__MAXOF_TYPE(ptrdiff_t)

typedef int_fast8_t   int_least8_t  ;
typedef uint_fast8_t  uint_least8_t ;
typedef int_fast16_t  int_least16_t ;
typedef uint_fast16_t uint_least16_t;
typedef int_fast32_t  int_least32_t ;
typedef uint_fast32_t uint_least32_t;
typedef int_fast64_t  int_least64_t ;
typedef uint_fast64_t uint_least64_t;

C_HEADER_END

#endif

