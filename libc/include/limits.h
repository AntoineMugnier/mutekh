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

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#ifndef LIMITS_H_
#define LIMITS_H_

/**
 * @file
 * @module{C library}
 */

#include <hexo/types.h>

/**
 * @multiple
 * @showcontent
 * This macro defines an integer type limit
 */

/* char type */

#define CHAR_BIT	8

#define SCHAR_MIN	(__MINOF_TYPE(int8_t))
#define SCHAR_MAX	(__MAXOF_TYPE(int8_t))

#define UCHAR_MAX	(__MAXOF_TYPE(uint8_t))

#define CHAR_MIN	SCHAR_MIN
#define CHAR_MAX	SCHAR_MAX

/* short type */

#define SHRT_MIN	(__MINOF_TYPE(int16_t))
#define SHRT_MAX	(__MAXOF_TYPE(int16_t))

#define USHRT_MAX	(__MAXOF_TYPE(uint16_t))

/* int type */

#define INT_MIN		(__MINOF_TYPE(int32_t))
#define INT_MAX		(__MAXOF_TYPE(int32_t))

#define UINT_MAX	(__MAXOF_TYPE(uint32_t))

#endif
