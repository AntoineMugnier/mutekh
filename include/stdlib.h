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

#ifndef STDLIB_H_
#define STDLIB_H_

#include <hexo/types.h>

/********************* memory allocation */

void * __attribute__ ((malloc))
calloc(size_t nmemb, size_t size);

void * __attribute__ ((malloc))
malloc(size_t size);

void 
free(void *ptr);

void * __attribute__ ((malloc))
realloc(void *ptr, size_t size);

/********************* integer conversion */

uint_fast8_t strto_uintl8(const char *nptr, char **endptr, int_fast8_t base);
int_fast8_t strto_intl8(const char *nptr, char **endptr, int_fast8_t base);
int_fast8_t ato_intl8(const char *nptr);

uint_fast16_t strto_uintl16(const char *nptr, char **endptr, int_fast8_t base);
int_fast16_t strto_intl16(const char *nptr, char **endptr, int_fast8_t base);
int_fast16_t ato_intl16(const char *nptr);

uint_fast32_t strto_uintl32(const char *nptr, char **endptr, int_fast8_t base);
int_fast32_t strto_intl32(const char *nptr, char **endptr, int_fast8_t base);
int_fast32_t ato_intl32(const char *nptr);

uint_fast64_t strto_uintl64(const char *nptr, char **endptr, int_fast8_t base);
int_fast64_t strto_intl64(const char *nptr, char **endptr, int_fast8_t base);
int_fast64_t ato_intl64(const char *nptr);

/** strtol function is deprecated because integer type width can not be
    clearly determined */
int_fast32_t __attribute__ ((deprecated))
strtol(const char *nptr, char **endptr, int_fast8_t base);

/** strtoul function is deprecated because integer type width can not be
    clearly determined */
uint_fast32_t __attribute__ ((deprecated))
strtoul(const char *nptr, char **endptr, int_fast8_t base);

/** atoi function is deprecated because integer type width can not be
    clearly determined */
int_fast32_t __attribute__ ((deprecated))
atoi(const char *nptr);

/** atol function is deprecated because integer type width can not be
    clearly determined */
int_fast32_t __attribute__ ((deprecated))
atol(const char *nptr);

/** strtoll function is deprecated because integer type width can not be
    clearly determined */
int_fast32_t __attribute__ ((deprecated))
strtoll(const char *nptr, char **endptr, int_fast8_t base);

/** strtoull function is deprecated because integer type width can not be
    clearly determined */
uint_fast64_t __attribute__ ((deprecated))
strtoull(const char *nptr, char **endptr, int_fast8_t base);

/** atoll function is deprecated because integer type width can not be
    clearly determined */
int_fast64_t __attribute__ ((deprecated))
atoll(const char *nptr);

/********************* misc */

/** integer minimum value */
#define __MIN(a, b) ({ const typeof(a) __a = (a); const typeof(b) __b = (b); __b < __a ? __b : __a; })

/** integer maximum value */
#define __MAX(a, b) ({ const typeof(a) __a = (a); const typeof(b) __b = (b); __b > __a ? __b : __a; })

/******************** random */

typedef uint_fast8_t	__rand_type_t;

#define RAND_MAX	(sizeof (__rand_type_t) > 1 ? 32767 : 255)

__rand_type_t rand(void);
__rand_type_t rand_r(__rand_type_t *seedp);
void srand(__rand_type_t seed);

#endif

