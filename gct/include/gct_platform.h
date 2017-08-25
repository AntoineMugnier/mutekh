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
 * @short platform defs for GCT
 */

#ifndef __MUTEK_GCT_PLATFORM_H__
#define __MUTEK_GCT_PLATFORM_H__

#include <hexo/decls.h>

C_HEADER_BEGIN

#ifdef _GCT_PLATFORM_H_
# error gct_platform.h must be included before any GCT header
#endif

#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/bit.h>

#define GCT_CONFIG_LIBRARY
#define GCT_CONFIG_NOPLATFORM
#define GCT_CONFIG_COMPACT_BINTREE

#define _GCT_INT	int_fast8_t
#define _GCT_UINT	uint_fast8_t
#define _GCT_ULONG	__compiler_ulong_t
#define _GCT_ULONGLONG	__compiler_ulonglong_t

#ifndef __STDC_VERSION__
# define __STDC_VERSION__ 199901
#endif

typedef bool_t		gct_bool_t;
typedef	error_t		gct_err_t;
typedef size_t		_gct_index_t;
typedef ssize_t		_gct_sindex_t;

ALWAYS_INLINE void* _gct_malloc(size_t s)
{
  extern void *malloc(size_t);

  return malloc(s);
}

ALWAYS_INLINE void* _gct_realloc(void *p, size_t s)
{
  extern void *realloc(void *, size_t);

  return realloc(p, s);
}

ALWAYS_INLINE void _gct_free(void *p)
{
  extern void free(void *);

  free(p);
}

#include <gct_atomic.h>

#define _GCT_POPCOUNT(n) bit_popc(n)
#define _GCT_BIT_FFS(n)  bit_ffs(n)
#define _GCT_BIT_CTZ(n)  bit_ctz(n)
#define _GCT_BIT_MSB(n)  bit_msb_index(n)

C_HEADER_END

#endif

