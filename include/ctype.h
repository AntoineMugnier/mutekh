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

#ifndef CTYPE_H_
#define CTYPE_H_

#include <hexo/types.h>

static inline bool_t
isalpha(int_fast8_t ch)
{
  return (uint8_t)((ch | 0x20) - 'a') < 26u;
}

static inline bool_t
isdigit(int_fast8_t ch)
{
  return (uint8_t)(ch - '0') < 10u;
}

static inline bool_t
isalnum(int_fast8_t ch)
{
  return isalpha(ch) || isdigit(ch);
}

static inline bool_t
isascii(int_fast8_t ch)
{
  return (uint8_t)ch < 128u;
}

static inline bool_t
isblank(int_fast8_t ch)
{
  return (ch == ' ') || (ch == '\t');
}

static inline bool_t
iscntrl(int_fast8_t ch)
{
  return ((uint8_t)ch < 32u) || (ch == 127);
}

static inline bool_t
isgraph(int_fast8_t ch)
{
  return (uint8_t)(ch - '!') < (127u - '!');
}

static inline bool_t
islower(int_fast8_t ch)
{
  return (uint8_t)(ch - 'a') < 26u;
}

static inline bool_t
isupper(int_fast8_t ch)
{
  return (uint8_t)(ch - 'A') < 26u;
}

static inline bool_t
isprint(int_fast8_t ch)
{
  return (uint8_t)(ch - ' ') < (127u - ' ');
}

static inline bool_t
isspace(int_fast8_t ch)
{
  return ((uint8_t)(ch - 9) < 5u) || (ch == ' ');
}

static inline bool_t
ispunct(int_fast8_t ch)
{
  return isprint(ch) && !isalnum(ch) && !isspace(ch);
}

static inline bool_t
isxdigit(int_fast8_t ch)
{
  return isdigit(ch) || ((uint8_t)((ch | 0x20) - 'a') < 6u);
}

static inline int_fast8_t
toupper(int_fast8_t c)
{
  return isalpha(c) ? c & ~0x20 : c;
}

static inline int_fast8_t
tolower(int_fast8_t c)
{
  return isalpha(c) ? c | 0x20 : c;
}

#endif

