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

    Based on dietlibc-0.29 http://www.fefe.de/dietlibc/

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <stdlib.h>
#include <string.h>

/********************************/

#ifndef HAS_CPU_MEMSET
#undef memset
inline void * memset(void * dst, int_fast8_t s, size_t count)
{
  char	*a = dst;

  while (count--)
    *a++ = s;

  return dst;
}
#endif

/********************************/

#ifndef HAS_CPU_MEMCPY
#undef memcpy
inline void * memcpy (void *dst, const void *src, size_t n)
{
  void	*res = dst;
  uint8_t	*c1, *c2;

  c1 = (uint8_t*)dst;
  c2 = (uint8_t*)src;

  while (n--)
    *c1++ = *c2++;

  return (res);
}
#endif

/********************************/

#ifndef HAS_CPU_MEMCMP
#undef memcmp
inline int_fast8_t memcmp(const void *dst, const void *src, size_t count)
{
  const uint8_t	*d = dst;
  const uint8_t	*s = src;
  int_fast8_t	r;

  count++;

  while (__builtin_expect(--count, 1))
    {
      if (__builtin_expect(r = (*d - *s), 0))
	return r;
      ++d;
      ++s;
  }

  return 0;
}
#endif

/********************************/

#ifndef HAS_CPU_MEMCPY_REVERSE
inline void *
__memcpy_reverse(void *dst, const void *src, size_t size)
{
  uint8_t	*dst_ = (uint8_t*)dst + size;
  uint8_t	*src_ = (uint8_t*)src + size;

  while (size--)
    *--dst_ = *--src_;
  return dst;
}
#endif

inline void *
memmove(void *dst, const void *src, size_t size)
{
  if (dst > src)
    return __memcpy_reverse(dst, src, size);
  else
    return memcpy(dst, src, size);
}

/********************************/

#ifndef HAS_CPU_STRLEN
#undef strlen
inline size_t strlen(const char *s)
{
  const char	*s_ = s;

  while (*s)
    s++;

  return s - s_;
}
#endif

/********************************/

#ifndef HAS_CPU_STRCAT
#undef strcat
inline char* strcat(char *s, const char *t)
{
  char	*dest = s;

  s += strlen(s);

  while (1)
    {
      if (!(*s = *t))
	break;
      s++;
      t++;
    }

  return dest;
}
#endif

/********************************/

#ifndef HAS_CPU_STRCHR
#undef strchr
inline char *strchr(const char *t, int_fast8_t c)
{
  char	ch = c;

  while (1)
    {
      if (__builtin_expect(*t == ch, 0))
	break;

      if (__builtin_expect(!*t, 0))
	return 0;

      t++;
    }

  return (char*)t;
}
#endif

/********************************/

#ifndef HAS_CPU_STRCMP
#undef strcmp
inline int_fast8_t strcmp (const char *s1, const char *s2)
{
  while (*s1 && *s1 == *s2)
    s1++, s2++;

  return (*s1 - *s2);
}
#endif

/********************************/

#ifndef HAS_CPU_STRCPY
#undef strcpy
inline char * strcpy (char *s1, const char *s2)
{
  char	*res = s1;

  while ((*s1++ = *s2++))
    ;

  return (res);
}
#endif

/********************************/

#ifndef HAS_CPU_STRDUP
#undef strdup
inline char *strdup(const char *s)
{
  size_t	len = strlen(s);
  char		*tmp;

  if ((tmp = malloc(len + 1)))
    {
      memcpy(tmp, s, len);
      tmp[len] = 0;
    }

  return tmp;
}
#endif

/********************************/

#ifndef HAS_CPU_STRSTR
#undef strstr
inline char *strstr(const char *haystack, const char *needle)
{
  size_t	nl = strlen(needle);
  size_t	hl = strlen(haystack);
  size_t	i;

  if (!nl)
    return (char*)haystack;

  if (nl > hl)
    return 0;

  for (i = hl - nl + 1; __builtin_expect(i, 1); i--)
    {
      if ((*haystack == *needle) && !memcmp(haystack, needle, nl))
	return (char*)haystack;

      haystack++;
    }

  return 0;
}
#endif

