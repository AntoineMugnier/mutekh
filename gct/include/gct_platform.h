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

#define GCT_CONFIG_LIBRARY
#define GCT_CONFIG_NOPLATFORM

#define _GCT_INT	int_fast8_t
#define _GCT_UINT	uint_fast8_t
#define _GCT_ULONG	__compiler_ulong_t
#define _GCT_ULONGLONG	__compiler_ulonglong_t

typedef bool_t		gct_bool_t;
typedef	error_t		gct_err_t;
typedef size_t		_gct_index_t;
typedef ssize_t		_gct_sindex_t;

#define _gct_malloc(s) malloc(s)
#define _gct_realloc(p, s) realloc(p, s)
#define _gct_free(p) free(p)


C_HEADER_END

#endif

