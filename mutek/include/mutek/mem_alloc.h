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
    Copyright Dimitri Refauvelet <dimitri.refauvelet@lip6.fr> (c) 2009
*/

/**
 * @file
 * @module{Mutek}
 * @short High level memory allocation stuff
 */


#ifndef MEM_ALLOC_H_ 
#define MEM_ALLOC_H_

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/lock.h>
#include <mutek/memory_allocator.h>
#include <mutek/printk.h>

#if defined(CONFIG_MUTEK_MEM_REGION)
#include <mutek/mem_region.h>
#endif

extern struct memory_allocator_region_s *default_region;

enum mem_scope_e
  {
    mem_scope_sys,
    mem_scope_cluster,
    mem_scope_context,
    mem_scope_cpu,
    mem_scope_default,
    mem_scope_e_count,
  };


/** @this allocate a new memory block in given scope*/
static inline
void *mem_alloc(size_t size, enum mem_scope_e scope)
{
  void *ptr = NULL;

  if (default_region != NULL)
    ptr = memory_allocator_pop (default_region, size);

# if defined (CONFIG_MUTEK_MEM_REGION)
  else
    {
      struct mem_region_s *region_item;
      mem_region_lock(scope);
      region_item = mem_region_get_first (scope);
      while( region_item )
	{
	  ptr = memory_allocator_pop (region_item->region, size);
	  if (ptr != NULL)
	    break;
	  region_item = mem_region_get_next (scope, region_item);
	}
      
      mem_region_unlock(scope);
    }
# endif

  return ptr;
}

/** @this free allocated memory block */
static inline
void mem_free(void *ptr)
{
  memory_allocator_push(ptr);
}

/** @this return the size of given memory block */
static inline
size_t mem_getsize(void *ptr)
{
  return memory_allocator_getsize(ptr);
}

#endif
