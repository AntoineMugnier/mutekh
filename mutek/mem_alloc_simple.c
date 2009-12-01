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
#include "memalloc.h"

/***************** Memory allocation interface ******************/

void *mem_alloc_region_pop(struct mem_alloc_region_s *region, size_t size)
{
  void *res, *next;

  lock_spin(&region->lock);

  res = region->next;

  /* insert a single word header to keep track of allocated size */
  size_t *hdr = res;
  res = hdr + 1;

#ifdef CONFIG_MUTEK_MEMALLOC_DEBUG
  memset(res, 0x5a, size);
#endif

  next = (uint8_t*)res + size;

  if (next <= region->last)
    {
      *hdr = size;
      region->next = next;
    }
  else
    {
      res = NULL;
    }

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

void *mem_alloc(size_t size, enum mem_scope_e scope)
{
  struct mem_alloc_region_s *region;
  void *res;

  region = mem_region_get_scope(scope);

  res = mem_alloc_region_pop(region, ALIGN_VALUE_UP(size, CONFIG_MUTEK_MEMALLOC_ALIGN));

  if (!res)
    printk("malloc(%d,%d) returns NULL\n", size, scope);

  return res;
}

size_t mem_alloc_getsize(void *ptr)
{
  return ((size_t*)ptr)[-1];
}

void mem_free(void *ptr)
{
#ifdef CONFIG_MUTEK_MEMALLOC_DEBUG
  memset(ptr, 0xa5, mem_alloc_getsize(ptr));
#endif
}

void *mem_reserve(struct mem_alloc_region_s *region, void *start, size_t size)
{
  printk("mem_reserve(,%p,%d) returns NULL (not implemented with CONFIG_MUTEK_MEMALLOC_SIMPLE)\n", start, size);
  return NULL;
}

