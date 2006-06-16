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
typedef unsigned int		uint32_t;

/** base interger type, 64 bits unsigned int */
typedef unsigned long long	uint64_t;

/** base interger type, 8 bits signed int */
typedef signed char		int8_t;

/** base interger type, 16 bits signed int */
typedef signed short		int16_t;

/** base interger type, 32 bits signed int */
typedef signed int		int32_t;

/** base interger type, 64 bits signed int */
typedef signed long long	int64_t;

#include "cpu/mutek/types.h"
#include "arch/mutek/types.h"

/* type used to prevent ld script symbols from being placed in
   .sdata section */
typedef struct __ldscript_symbol_s __ldscript_symbol_t;

#ifdef __MUTEK__
/** prevent use of compiler native short type */
# define short	struct _dont_use_short_

/** prevent use of compiler native int type */
# define int	struct _dont_use_int_

/** prevent use of compiler native long type */
# define long	struct _dont_use_long_
#endif


/* integer types MAX and MIN values */

/** return max integer value for a type */
#define __MINOF_TYPE(t)        ((typeof(t))(((typeof(t))-1) < 0 ?  (((typeof(t))1) << sizeof(typeof(t)) * 8 - 1) :  0))

/** return min integer value for a type */
#define __MAXOF_TYPE(t)        ((typeof(t))(((typeof(t))-1) < 0 ? ~(((typeof(t))1) << sizeof(typeof(t)) * 8 - 1) : -1))

/** NULL pointer definition */
#define NULL	((void*)0)

#endif

