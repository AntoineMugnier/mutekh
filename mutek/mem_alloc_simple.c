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

#include <mutek/mem_alloc.h>
#include <string.h>

#ifdef CONFIG_MUTEK_MEMALLOC_SIMPLE

/***************** Memory allocatable region management ******************/

static const size_t	mem_hdr_size = 0;

struct mem_alloc_region_s
{
  lock_t		lock;
  void			*next;
  void			*last;
};

struct mem_alloc_region_s mem_region_system;

/***************** Memory allocation interface ******************/

void *mem_alloc_region_pop(struct mem_alloc_region_s *region, size_t size)
{
  void	*res, *next;

  lock_spin(&region->lock);

  res = region->next;

#ifdef CONFIG_MUTEK_MEMALLOC_DEBUG
  memset(res, 0x5a, size);
#endif

  next = (uint8_t*)region->next + size;

  if (next > region->last)
    res = NULL;

  region->next = next;

  lock_release(&region->lock);

  return res;
}

void mem_alloc_region_init(struct mem_alloc_region_s *region,
			   void *address, void *end)
{
  lock_init(&region->lock);

  region->next = address;
  region->last = end;
}

/** allocate a new memory block in given region */
void *mem_alloc(size_t size, struct mem_alloc_region_s *region)
{
  void *hdr;

  size = mem_hdr_size
    + ALIGN_VALUE_UP(size, CONFIG_MUTEK_MEMALLOC_ALIGN);

  hdr = mem_alloc_region_pop(region, size);

  return hdr != NULL
    ? (uint8_t*)hdr + mem_hdr_size : NULL;
}

/** free allocated memory block */
void mem_free(void *ptr)
{
}

void mem_region_init()
{
}

struct mem_alloc_region_s *mem_region_create(uintptr_t start, uintptr_t end, bool_t cached)
{
}

struct mem_alloc_region_s *mem_region_get_scope(enum mem_scope_e scope)
{
}

void mem_region_set_scope(enum mem_scope_e scope, struct mem_alloc_region_s *region);
{
}

#endif
