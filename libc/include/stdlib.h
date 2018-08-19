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

#ifndef STDLIB_H_
#define STDLIB_H_

#include <hexo/decls.h>

C_HEADER_BEGIN

/**
 * @file
 * @module {Core::C library}
 */

#include <hexo/types.h>
#include <hexo/cpu.h>

/********************* memory allocation */

#include "alloca.h"

config_depend(CONFIG_MUTEK_MEMALLOC)
void * __attribute__ ((malloc))
calloc(size_t nmemb, size_t size);

config_depend(CONFIG_MUTEK_MEMALLOC)
void * __attribute__ ((malloc))
malloc(size_t size);

config_depend(CONFIG_MUTEK_MEMALLOC)
void 
free(void *ptr);

config_depend(CONFIG_MUTEK_MEMALLOC_SMART)
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
# define strtoptr _STRTOINT_(strto_uintl, INT_PTR_SIZE)

# define atoi _STRTOINT_(ato_intl, CPU_SIZEOF_INT)
# define atol _STRTOINT_(ato_intl, CPU_SIZEOF_LONG)
# define atoll ato_intl64

double strtod(const char *d, const char **ret);

ALWAYS_INLINE
float atof(const char *f)
{
  return strtod(f, NULL);
}

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

void *bsearch(
  const void *key,
  const void *base,
  size_t nel,
  size_t width,
  __compiler_sint_t (*compar) (const void *, const void *)
  );

/******************** random */

/** @This specifies the maximum value returned by the
    @ref rand and @ref random functions. @showvalue */
#define RAND_MAX 0x7fffffff

/** @internal seed for standard @ref rand and @ref random functions */
extern __compiler_uint_t __rand_seed;

/** @internal pseudo random generator backend for standard functions,
    relies on a 32 bits lfsr. */
uint32_t __rand_r32(__compiler_uint_t *seedp);

/** @This returns a value in the range @em{[0, RAND_MAX]}. */
ALWAYS_INLINE __compiler_sint_t rand(void)
{
  return __rand_r32(&__rand_seed);
}

/** @This returns a value in the range @em{[0, RAND_MAX]} and update
    the user provided seed. */
ALWAYS_INLINE __compiler_sint_t rand_r(__compiler_uint_t *seedp)
{
  return __rand_r32(seedp);
}

/** @This sets the seed used by the @ref rand and @ref random
    functions. Because the seed is used system-wide, different
    software componenents may update the seed, resulting in unexpected
    behavior. */
__attribute__((deprecated("Random seed is used system-wide")))
ALWAYS_INLINE void srand(__compiler_uint_t seed)
{
  __rand_seed = seed;
}

/** @This is the same as @ref rand. */
ALWAYS_INLINE __compiler_slong_t random(void)
{
  return __rand_r32(&__rand_seed);
}

/** @This is the same as @ref srand. */
__attribute__((deprecated("Random seed is used system-wide")))
ALWAYS_INLINE void srandom(__compiler_uint_t seed)
{
  __rand_seed = seed;
}

/** @This returns an unsigned 32 bits pseudo random value. The
    internal state is 64 bits wide and can not be assigned directly.
    @see rand_64_merge @see rand_64_r */
uint32_t rand_64(void);

/** @This returns an unsigned 32 bits pseudo random value
    and update the user provided seed. */
uint32_t rand_64_r(uint64_t *s);

/** @This contributes entropy to the internal state which is used by
    the @ref rand_64 function. Note that this does not just assign a
    new value to the internal state which still depends on the
    previous state. */
void rand_64_merge(const void *data, size_t size);

/******************** abort */

__attribute__((noreturn))
void abort(void);

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

__attribute__((deprecated))
char *getenv(const char *key);

__attribute__((deprecated))
error_t system(const char *cmd);

/****************** abs */

/** Compute the absolute value of an integer. This generic macro
    adapts to the integer type width. @see #abs */
#define __ABS(n)                                                        \
({                                                                      \
  typedef typeof(n) _t;                                                 \
  _t _n = (n);                                                          \
                                                                        \
  __builtin_types_compatible_p(_t, __compiler_slong_t) ||               \
    __builtin_types_compatible_p(_t, __compiler_ulong_t)                \
    ? __builtin_labs(_n)                                                \
    : __builtin_types_compatible_p(_t, __compiler_slonglong_t) ||       \
      __builtin_types_compatible_p(_t, __compiler_ulonglong_t)          \
    ? __builtin_llabs(_n) :                                             \
      __builtin_abs(_n);                                                \
})

/** standard @tt abs function @see #__ABS */
#define abs(x) __builtin_abs(x)
/** standard @tt labs function @see #__ABS */
#define labs(x) __builtin_labs(x)
/** standard @tt llabs function @see #__ABS */
#define llabs(x) __builtin_llabs(x)

uint32_t gcd32(uint32_t a, uint32_t b);
uint64_t gcd64(uint64_t a, uint64_t b);

/** @see div */
typedef struct {
  __compiler_sint_t quot;
  __compiler_sint_t rem;
} div_t;

div_t div(__compiler_sint_t number, __compiler_sint_t denom);

/** @see ldiv */
typedef struct {
  __compiler_slong_t quot;
  __compiler_slong_t rem;
} ldiv_t;

ldiv_t ldiv(__compiler_slong_t number, __compiler_slong_t denom);

C_HEADER_END

#endif

