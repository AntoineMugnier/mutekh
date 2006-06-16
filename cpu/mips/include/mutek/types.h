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
   @file

   CPU dependant prefered types definition
 */

#if !defined(TYPES_H_) || defined(CPU_TYPES_H_)
#error This file can not be included directly
#else

#define CPU_TYPES_H_

/** signed integer type of CPU prefered size, at least 8 bits */
typedef	int32_t			int_fast8_t;

/** signed integer type of CPU prefered size, at least 16 bits */
typedef	int32_t			int_fast16_t;

/** signed integer type of CPU prefered size, at least 32 bits */
typedef	int32_t			int_fast32_t;

/** signed integer type of CPU prefered size, at least 64 bits */
typedef	int64_t			int_fast64_t;

/** unsigned integer type of CPU prefered size, at least 8 bits */
typedef	uint32_t		uint_fast8_t;

/** unsigned integer type of CPU prefered size, at least 16 bits */
typedef	uint32_t		uint_fast16_t;

/** unsigned integer type of CPU prefered size, at least 32 bits */
typedef	uint32_t		uint_fast32_t;

/** unsigned integer type of CPU prefered size, at least 64 bits */
typedef	uint64_t		uint_fast64_t;

/** unsigned integer type suitable for memory addresses */
typedef uint32_t		uintptr_t;

/** signed integer type suitable for memory addresses */
typedef uint32_t		intptr_t;

/** signed integer type suitable for memory addresses */
typedef int32_t			ptrdiff_t;

/** integer type used for atomic operation */
typedef	int32_t			__atomic_t;

/** general cpu register integer type */
typedef uint32_t		__reg_t;

#endif

