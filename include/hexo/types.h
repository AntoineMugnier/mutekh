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


#ifndef TYPES_H_
#define TYPES_H_

/** base interger type, 8 bits unsigned int */
typedef unsigned char		uint8_t;

/** base interger type, 16 bits unsigned int */
typedef unsigned short		uint16_t;

/** base interger type, 32 bits unsigned int */
#if CONFIG_COMPILE_SIZEOF_INT == 2
typedef unsigned long		uint32_t;
#else
typedef unsigned int		uint32_t;
#endif

/** base interger type, 64 bits unsigned int */
typedef unsigned long long	uint64_t;

/** base interger type, 8 bits signed int */
typedef signed char		int8_t;

/** base interger type, 16 bits signed int */
typedef signed short		int16_t;

/** base interger type, 32 bits signed int */
#if CONFIG_COMPILE_SIZEOF_INT == 2
typedef signed long		int32_t;
#else
typedef signed int		int32_t;
#endif

/** base interger type, 64 bits signed int */
typedef signed long long	int64_t;

#include "cpu/hexo/types.h"
#include "arch/hexo/types.h"

/* type used to prevent ld script symbols from being placed in
   .sdata section */
typedef struct __ldscript_symbol_s __ldscript_symbol_t;

#ifdef __MUTEK__
/** prevent use of compiler native short type,
    int_fast*_t and uint_fast*_t types are prefered */
typedef short _dont_use_native_short_type_t __attribute__ ((deprecated));
# define short	_dont_use_native_short_type_t

/** prevent use of compiler native int type,
    int_fast*_t and uint_fast*_t types are prefered */
typedef int _dont_use_native_int_type_t __attribute__ ((deprecated));
# define int	_dont_use_native_int_type_t

/** prevent use of compiler native long type,
    int_fast*_t and uint_fast*_t types are prefered */
typedef long _dont_use_native_long_type_t __attribute__ ((deprecated));
# define long	_dont_use_native_long_type_t
#endif

/* integer types MAX and MIN values */

/** return max integer value for a type */
#define __MINOF_TYPE(t)        ((typeof(t))(((typeof(t))-1) < 0 ?  (((typeof(t))1) << sizeof(typeof(t)) * 8 - 1) :  0))

/** return min integer value for a type */
#define __MAXOF_TYPE(t)        ((typeof(t))(((typeof(t))-1) < 0 ? ~(((typeof(t))1) << sizeof(typeof(t)) * 8 - 1) : -1))

/** NULL pointer definition */
#define NULL	((void*)0)

#endif

