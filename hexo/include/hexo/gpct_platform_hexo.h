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

#ifndef __GPCT_PLATFORM_HEXO_H__
#define __GPCT_PLATFORM_HEXO_H__

#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/atomic.h>

#define GPCT_CONFIG_NOPLATFORM

#define GPCT_ULONG	__compiler_ulong_t
#define GPCT_ULONGLONG	__compiler_ulonglong_t

typedef bool_t		gpct_bool_t;
typedef	error_t		gpct_error_t;
typedef atomic_t	gpct_atomic_t;

/* static value init */
#define gpct_ATOMIC_INITIALIZER(v)	ATOMIC_INITIALIZER(v)

/* set atomic value */
#define	gpct_atomic_set(a, v)	atomic_set(a, v)

/* get atomic value */
#define gpct_atomic_get(a)		atomic_get(a)

/* increment atomic value return true if not 0 */
#define gpct_atomic_inc(a)		atomic_inc(a)

/* decrement atomic value return true if not 0 */
#define gpct_atomic_dec(a)		atomic_dec(a)

#endif

