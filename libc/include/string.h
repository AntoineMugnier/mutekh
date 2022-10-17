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
#include <hexo/bit.h>

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

config_depend(CONFIG_MUTEK_MEMALLOC)
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

/** standard @tt ffs function @see #bit_ffs */
#define ffs(x) bit_ffs(x)
/** standard @tt ffsl function @see #bit_ffs */
#define ffsl(x) bit_ffs(x)
/** standard @tt ffsll function @see #bit_ffs */
#define ffsll(x) bit_ffs(x)

const char *strerror(error_t errnum);

#define FNV_32_INIT 0x811c9dc5
#define FNV_64_INIT 0xcbf29ce484222325ULL

inline uint32_t fnv1a_32_update(uint32_t f, uint8_t x)
{
  f ^= x;
#if CONFIG_CPU_HARDWARE_MUL >= 32
  f *= 0x01000193;
#else
  uint32_t f3 = f + (f << 1);
  f = (f << 24) + (f << 4) + f3 + (f3 << 7);
#endif
  return f;
}

inline uint32_t fnv1a_32_update_buf(uint32_t f, const void *data, size_t len)
{
  const uint8_t *d = data;
  for (size_t i = 0; i < len; i++)
    f = fnv1a_32_update(f, d[i]);
  return f;
}

inline uint32_t fnv1a_32(const void *data, size_t len)
{
  return fnv1a_32_update_buf(FNV_32_INIT, data, len);
}

inline uint64_t fnv1a_64_update(uint64_t f, uint8_t x)
{
  f ^= x;
#if CONFIG_CPU_HARDWARE_MUL >= 64
  f = (f * 0x1b3) + (f << 40);
#else
  uint64_t f3 = f + (f << 1);
  f = (f << 40) + (f3 << 7) + (f3 << 4) + f3;
#endif
  return f;
}

inline uint64_t fnv1a_64_update_buf(uint64_t f, const void *data, size_t len)
{
  const uint8_t *d = data;
  for (size_t i = 0; i < len; i++)
    f = fnv1a_64_update(f, d[i]);
  return f;
}

inline uint64_t fnv1a_64(const void *data, size_t len)
{
  return fnv1a_64_update_buf(FNV_64_INIT, data, len);
}

C_HEADER_END

#endif

