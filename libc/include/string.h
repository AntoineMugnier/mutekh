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

    Based on string.h from dietlibc-0.29 http://www.fefe.de/dietlibc/

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef STRING_H_
#define STRING_H_

#include <hexo/decls.h>

C_HEADER_BEGIN

/**
 * @file
 * @module {Core::C library}
 */

#include <hexo/types.h>
#include <hexo/error.h>

#if defined (CONFIG_LIBC_STRING_ASM)
# include <cpu/string.h>
#endif

void * memset(void *dst, int_fast8_t data, size_t size);
#define memset __builtin_memset

void * memcpy(void *dst, const void *src, size_t size);
#define memcpy __builtin_memcpy

void *memccpy(void *dst, const void *src, char c, size_t count);

/* Reverse memcpy */
void memrevcpy(void *dest, const void *src, size_t size);

void memxor(void *dest, const void *a, const void *b, size_t len);

void *__memcpy_reverse(void *dst, const void *src, size_t size);

void *memmove(void *dst, const void *src, size_t size);

int_fast8_t memcmp(const void *s1, const void *s2, size_t n) __attribute__ ((pure));

int_fast8_t memcstcmp(const void *s1, int_fast8_t, size_t n) __attribute__ ((pure));

/***************************************** string operations */

size_t  __attribute__ ((pure))
strlen(const char *s);
#define strlen __builtin_strlen

char *
strcat(char *dest, const char *src);
#define strcat __builtin_strcat

char *
strncat(char *dest, const char *src, size_t n);
#define strncat __builtin_strncat

char * __attribute__ ((pure))
strchr(const char *s, int_fast8_t c);
#define strchr __builtin_strchr


char * __attribute__ ((pure))
strrchr(const char *s, int_fast8_t c);
#define strrchr __builtin_strrchr


int_fast8_t __attribute__ ((pure))
strcmp(const char *s1, const char *s2);
#define strcmp __builtin_strcmp

ALWAYS_INLINE int_fast8_t
__attribute__ ((deprecated,pure))
strcoll(const char *s1, const char *s2)
{
  return strcmp(s1, s2);
}

char *
strcpy(char *dest, const char *src);
#define strcpy __builtin_strcpy

char *  __attribute__ ((malloc))
strdup(const char *s);
#define strdup __builtin_strdup

char *  __attribute__ ((pure))
strstr(const char *haystack, const char *needle);
#define strstr __builtin_strstr

void *  __attribute__ ((pure))
memchr(const void *s, int_fast8_t c, size_t n);

int_fast8_t __attribute__ ((pure))
strcasecmp(const char* s1, const char* s2);

int_fast8_t __attribute__ ((pure))
strncasecmp(const char* s1, const char* s2, size_t len);

char *
strncpy(char *dest, const char *src, size_t n);
#define strncpy __builtin_strncpy

int_fast8_t __attribute__ ((pure))
strncmp(const char *s1, const char *s2, size_t n);
#define strncmp __builtin_strncmp

char *
strtok_r(char *s, const char *delim, char **ptrptr);

size_t
strspn(const char *s, const char *_accept);

size_t
strcspn(const char *s, const char *reject);

char *
strpbrk(const char *s1, const char *s2);

/*

int_fast8_t memccmp(const void *s1, const void *s2, int_fast8_t c, size_t n) __attribute__ ((pure));

void* memrchr(const void *s, int_fast8_t c, size_t n) __attribute__ ((pure));

void *memmem(const void *haystack, size_t haystacklen, const void *needle, size_t needlelen);

char * __attribute__ ((pure))
strrchr(const char *s, int_fast8_t c);

char *
strpbrk(const char *s, const char *_accept);

char *
strsep(char **stringp, const char *delim);

char *
strerror(error_t errnum);

error_t __attribute__ ((deprecated))
strerror_r(error_t errnum, char* buf, size_t n);

char * __attribute__ ((malloc))
strndup(const char *s, size_t n);

char *
strtok(char *s, const char *delim);

char *
strtok_r(char *s, const char *delim, char **ptrptr);

size_t
strlcpy(char *dst, const char *src, size_t size);

size_t
strlcat(char *dst, const char *src, size_t size);

size_t
strxfrm(char *dest, const char *src, size_t n);

char *
stpcpy(char *dest, const char *src);
*/

/***************************************** bit string operations */

/** Find first bit set in an integer. This generic macro adapts to the
    integer type width. @see #__CLZ */
#define __FFS(n)                                                        \
({                                                                      \
  typedef typeof(n) _t;                                                 \
  _t _n = (n);                                                          \
                                                                        \
  __builtin_types_compatible_p(_t, __compiler_slong_t) ||               \
    __builtin_types_compatible_p(_t, __compiler_ulong_t)                \
    ? __builtin_ffsl(_n)                                                \
    : __builtin_types_compatible_p(_t, __compiler_slonglong_t) ||       \
    __builtin_types_compatible_p(_t, __compiler_ulonglong_t)            \
    ? __builtin_ffsll(_n)                                               \
    : __builtin_ffs(_n);                                                \
})

/** standard @tt ffs function @see #__FFS */
#define ffs(x) __builtin_ffs(x)
/** standard @tt ffsl function @see #__FFS */
#define ffsl(x) __builtin_ffsl(x)
/** standard @tt ffsll function @see #__FFS */
#define ffsll(x) __builtin_ffsll(x)

const char *strerror(error_t errnum);

C_HEADER_END

#endif

