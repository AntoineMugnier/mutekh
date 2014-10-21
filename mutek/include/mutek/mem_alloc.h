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

#if defined(CONFIG_MUTEK_MEM_REGION)
#include <mutek/mem_region.h>
#endif

enum mem_scope_e
  {
    mem_scope_sys,
    mem_scope_cluster,
    mem_scope_context,
    mem_scope_cpu,
    mem_scope_default,
    mem_scope_e_count,
  };

/** @This checks memory allocator structures.
    @see #CONFIG_MUTEK_MEMALLOC_CRC
    @see #CONFIG_MUTEK_MEMALLOC_GUARD
    @see #CONFIG_MUTEK_MEMALLOC_SCRAMBLE
    @see #CONFIG_MUTEK_MEMALLOC_STATS
*/
ALWAYS_INLINE
void mem_check()
{
  memory_allocator_region_check(default_region);
}

/** @this allocates a new memory block in given scope with specified
    alignment constraint. 
    @see #CONFIG_MUTEK_MEMALLOC_ALIGN
*/
ALWAYS_INLINE
void *mem_alloc_align(size_t size, size_t align, enum mem_scope_e scope)
{
  if (size == 0) 
    return NULL;

  void *ptr = NULL;

  if (default_region != NULL)
    ptr = memory_allocator_pop (default_region, size, align);

# if defined (CONFIG_MUTEK_MEM_REGION)
  else
    {
      struct mem_region_s *region_item;
      mem_region_lock(scope);
      region_item = mem_region_get_first (scope);
      while( region_item )
	{
	  ptr = memory_allocator_pop (region_item->region, size, align);
	  if (ptr != NULL)
	    break;
	  region_item = mem_region_get_next (scope, region_item);
	}
      
      mem_region_unlock(scope);
    }
# endif

  return ptr;
}

/** @This allocates a new memory block in given scope. */
ALWAYS_INLINE
void *mem_alloc(size_t size, enum mem_scope_e scope)
{
  return mem_alloc_align(size, 1, scope);
}

/** @This allocate a new memory block for another cpu in given scope*/
ALWAYS_INLINE
void *mem_alloc_cpu(size_t size, enum mem_scope_e scope, cpu_id_t cpu_id)
{
  if (size == 0)
    return NULL;

  void *ptr = NULL;

  if (default_region != NULL)
    ptr = memory_allocator_pop (default_region, size, 1);

# if defined (CONFIG_MUTEK_MEM_REGION)
  else
    {
      struct mem_region_s *region_item;
      mem_region_id_lock(cpu_id, scope);
      region_item = mem_region_id_get_first(cpu_id, scope);
      while( region_item )
	{
	  ptr = memory_allocator_pop (region_item->region, size, 1);
	  if (ptr != NULL)
	    break;
	  region_item = mem_region_id_get_next(cpu_id, scope, region_item);
	}
      
      mem_region_id_unlock(cpu_id, scope);
    }
# endif

  return ptr;
}

/** @This frees allocated memory block */
ALWAYS_INLINE
void mem_free(void *ptr)
{
  memory_allocator_push(ptr);
}

/** @This returns the size of given memory block */
ALWAYS_INLINE
size_t mem_getsize(void *ptr)
{
  return memory_allocator_getsize(ptr);
}

/** @This resizes the given memory block, return NULL if failed. The
    allocated memory block is not moved; if not enough free memory
    blocks are available next to allocated memory, this function
    fails. */
ALWAYS_INLINE
void *mem_resize(void *ptr, size_t size)
{
  return memory_allocator_resize(ptr, size);
}

#endif
