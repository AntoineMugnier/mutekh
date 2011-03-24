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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2009
*/

#ifndef TIME_H_
#define TIME_H_

#include <hexo/decls.h>

C_HEADER_BEGIN

/**
   @file
   @module{C library}
   @short Time-related function stubs
 */

struct timeval;
struct timezone;

static inline error_t gettimeofday(struct timeval *tv, struct timezone *tz)
{
  return 0;
}

static inline error_t settimeofday(const struct timeval *tv, const struct timezone *tz)
{
  return 0;
}

static inline error_t time(time_t *t)
{
  return 0;
}

C_HEADER_END

#endif
