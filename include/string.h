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

    Based on string.h from dietlibc-0.29 http://www.fefe.de/dietlibc/

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#ifndef STRING_H_
#define STRING_H_

#include <mutek/types.h>
#include <mutek/error.h>
#include <cpu/string.h>

void * memset(void *dst, int_fast8_t data, size_t size);
#define memset __builtin_memset

void * memcpy(void *dst, const void *src, size_t size);
#define memcpy __builtin_memcpy

void *__memcpy_reverse(void *dst, const void *src, size_t size);

void *memmove(void *dst, const void *src, size_t size);

int_fast8_t memcmp(const void *s1, const void *s2, size_t n) __attribute__ ((pure));

/***************************************** string operations */

size_t  __attribute__ ((pure))
strlen(const char *s);
#define strlen __builtin_strlen

char *
strcat(char *dest, const char *src);
#define strcat __builtin_strcat

char * __attribute__ ((pure))
strchr(const char *s, int_fast8_t c);
#define strchr __builtin_strchr

int_fast8_t __attribute__ ((pure))
strcmp(const char *s1, const char *s2);
#define strcmp __builtin_strcmp

char *
strcpy(char *dest, const char *src);
#define strcpy __builtin_strcpy

char *  __attribute__ ((malloc))
strdup(const char *s);
#define strdup __builtin_strdup

char *  __attribute__ ((pure))
strstr(const char *haystack, const char *needle);
#define strstr __builtin_strstr

/*

int_fast8_t memccmp(const void *s1, const void *s2, int_fast8_t c, size_t n) __attribute__ ((pure));
void *memccpy(void *dest, const void *src, int_fast8_t c, size_t n);

void* memchr(const void *s, int_fast8_t c, size_t n) __attribute__ ((pure));
void* memrchr(const void *s, int_fast8_t c, size_t n) __attribute__ ((pure));

void *memmem(const void *haystack, size_t haystacklen, const void *needle, size_t needlelen);

char *
strncpy(char *dest, const char *src, size_t n);

int_fast8_t __attribute__ ((pure))
strncmp(const char *s1, const char *s2, size_t n);

char *
strncat(char *dest, const char *src, size_t n);

char * __attribute__ ((pure))
strrchr(const char *s, int_fast8_t c);

size_t
strspn(const char *s, const char *_accept);

size_t
strcspn(const char *s, const char *reject);

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

int_fast8_t ffsl(uint32_t i);
#define ffsl	__builtin_ffsl

int_fast8_t ffsll(uint64_t i);
#define ffsll	__builtin_ffsll

#endif

