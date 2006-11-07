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

#if !defined(ALLOC_H_) || defined(ARCH_ALLOC_H_)
#error This file can not be included directly
#else

#define ARCH_ALLOC_H_

#include <assert.h>

extern struct mem_alloc_region_s mem_region_mmap;

/** allocated memory scope is system global */
#define MEM_SCOPE_SYS		(&mem_region_mmap)

/** set default allocation policy */
static inline void
mem_alloc_set_default(struct mem_alloc_region_s *region)
{
  assert(region == &mem_region_mmap);
}

#endif

