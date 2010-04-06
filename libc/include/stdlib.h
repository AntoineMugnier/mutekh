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

#include <hexo/decls.h>

C_HEADER_BEGIN

/**
 * @file
 * @module{C library}
 */

#include <hexo/types.h>
#include <hexo/cpu.h>

/********************* memory allocation */

#include "alloca.h"

void * __attribute__ ((malloc))
calloc(size_t nmemb, size_t size);

void * __attribute__ ((malloc))
malloc(size_t size);

void 
free(void *ptr);

void * __attribute__ ((malloc))
realloc(void *ptr, size_t size);

/********************* integer conversion */

#if 0
uint_fast8_t strto_uintl8(const char *nptr, char **endptr, int_fast8_t base);
int_fast8_t strto_intl8(const char *nptr, char **endptr, int_fast8_t base);
int_fast8_t ato_intl8(const char *nptr);
#endif

uint_fast16_t strto_uintl16(const char *nptr, char **endptr, int_fast8_t base);
int_fast16_t strto_intl16(const char *nptr, char **endptr, int_fast8_t base);
int_fast16_t ato_intl16(const char *nptr);

uint_fast32_t strto_uintl32(const char *nptr, char **endptr, int_fast8_t base);
int_fast32_t strto_intl32(const char *nptr, char **endptr, int_fast8_t base);
int_fast32_t ato_intl32(const char *nptr);

uint_fast64_t strto_uintl64(const char *nptr, char **endptr, int_fast8_t base);
int_fast64_t strto_intl64(const char *nptr, char **endptr, int_fast8_t base);
int_fast64_t ato_intl64(const char *nptr);

# define _STRTOINT__(a, b) a ## b
# define _STRTOINT_(a, b) _STRTOINT__(a, b)

# define strtol _STRTOINT_(strto_intl, CPU_SIZEOF_LONG)
# define strtoul _STRTOINT_(strto_uintl, CPU_SIZEOF_LONG)
# define strtoll strto_intl64
# define strtoull strto_uintl64

# define atoi _STRTOINT_(ato_intl, CPU_SIZEOF_INT)
# define atol _STRTOINT_(ato_intl, CPU_SIZEOF_LONG)
# define atoll ato_intl64

/********************* misc */

/** Get integer minimum value */
#define __MIN(a, b) ({ const typeof(a) __a = (a); const typeof(b) __b = (b); __b < __a ? __b : __a; })

/** Get integer maximum value */
#define __MAX(a, b) ({ const typeof(a) __a = (a); const typeof(b) __b = (b); __b > __a ? __b : __a; })


/**
   @this defines the function prototype used for comparaison in qsort.
 */
typedef int_fast8_t qsort_compar_t(const void *, const void *);

/**
   @this sorts the @tt{nel}-element long array pointed by @tt base
   containing elements of size @tt width. Using @tt compar as the
   comparaison function.

   @param base Base pointer of the array
   @param nel Element count in the array
   @param width sizeof(element)
   @param compar comparaison fuction
*/
void qsort(void *base, size_t nel, size_t width, qsort_compar_t *compar);

/******************** random */

typedef uint_fast8_t	__rand_type_t;

#define RAND_MAX	(sizeof (__rand_type_t) > 1 ? 32767 : 255)

__rand_type_t rand(void);
__rand_type_t rand_r(__rand_type_t *seedp);
void srand(__rand_type_t seed);

/******************** abort */

static inline void
abort(void)
{
  cpu_trap();

  while (1)
    ;
}

/******************* exit */

/** standard EXIT_SUCCESS macro */
#define EXIT_SUCCESS	0
/** standard EXIT_FAILURE macro */
#define EXIT_FAILURE	-1

void
__attribute__ ((deprecated))
exit(uint_fast8_t status);

error_t
__attribute__ ((deprecated))
atexit(void (*function)(void));

/****************** abs */

#if 0
__compiler_sint_t abs(__compiler_sint_t j);
__compiler_slong_t labs(__compiler_slong_t j);
__compiler_slonglong_t llabs(__compiler_slonglong_t j);
#endif

/** standard abs function */
#define abs(x) ({ const typeof(x) __x = (x); __x < 0 ? -__x : __x; })
/** standard labs function */
#define labs abs
/** standard llabs function */
#define llabs abs

C_HEADER_END

#endif

