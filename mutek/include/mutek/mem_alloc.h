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
 * @module {Core::Kernel services}
 * @short Memory allocation stuff
 */


#ifndef MEM_ALLOC_H_ 
#define MEM_ALLOC_H_

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/lock.h>

struct memory_allocator_region_s;

/** @internal */
extern struct memory_allocator_region_s *default_region;

/** @internal @this initialize a memory region*/
struct memory_allocator_region_s *
memory_allocator_init(struct memory_allocator_region_s *container_region, void *start, void *end);

/** @internal @this extend an existing memory region with a new memory space */
struct memory_allocator_header_s *
memory_allocator_extend(struct memory_allocator_region_s *region, void *start, size_t size);

/** @internal @this resize the given memory block */
void *memory_allocator_resize(void *address, size_t size);

/** @internal @this allocate a new memory block in given region */
void *memory_allocator_pop(struct memory_allocator_region_s *region, size_t size, size_t align);

/** @internal @this free allocated memory block */
void memory_allocator_push(void *address);

/** @internal @this reserve a memory space in given region */
void *memory_allocator_reserve(struct memory_allocator_region_s *region, void *start, size_t size);

/** @internal @this return the size of given memory block */
size_t memory_allocator_getsize(void *ptr);

/** @internal @this return statistic of memory region use */
error_t memory_allocator_stats(struct memory_allocator_region_s *region,
			size_t *alloc_blocks,
			size_t *free_size,
			size_t *free_blocks);

/** @internal @this make memory region check depending to activated token: guard zone, headers' integrity (crc and size) 
    and if free space was used */
void memory_allocator_region_check(struct memory_allocator_region_s *region);

/** @internal @this return the size of given memory region */
size_t memory_allocator_region_size(struct memory_allocator_region_s *region);

/** @internal @this return the size of given memory region */
void* memory_allocator_region_address(struct memory_allocator_region_s *region);

/** @This dumps the list of memory allocator blocks of the specified region. */
config_depend_and2(CONFIG_MUTEK_PRINTK, CONFIG_MUTEK_MEMALLOC_SMART)
void memory_allocator_dumpk(struct memory_allocator_region_s *region);

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
config_depend_alwaysinline(CONFIG_MUTEK_MEMALLOC,
void mem_check(void),
{
  memory_allocator_region_check(default_region);
});

/** @this allocates a new memory block in given scope with specified
    alignment constraint. 
    @see #CONFIG_MUTEK_MEMALLOC_ALIGN
*/
config_depend_inline(CONFIG_MUTEK_MEMALLOC,
void *
mem_alloc_align(size_t size, size_t align, enum mem_scope_e scope),
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
});

/** @This allocates a new memory block in given scope. */
config_depend_alwaysinline(CONFIG_MUTEK_MEMALLOC,
void *mem_alloc(size_t size, enum mem_scope_e scope),
{
  return mem_alloc_align(size, 1, scope);
});

/** @This allocate a new memory block for another cpu in given scope*/
config_depend_inline(CONFIG_MUTEK_MEMALLOC,
void *
mem_alloc_cpu(size_t size, enum mem_scope_e scope, cpu_id_t cpu_id),
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
});

/** @This frees allocated memory block */
config_depend_inline(CONFIG_MUTEK_MEMALLOC,
void mem_free(void *ptr),
{
  memory_allocator_push(ptr);
});

/** @This returns the size of given memory block */
config_depend_alwaysinline(CONFIG_MUTEK_MEMALLOC_SMART,
size_t mem_getsize(void *ptr),
{
  return memory_allocator_getsize(ptr);
});

/** @This resizes the given memory block, return NULL if failed. The
    allocated memory block is not moved; if not enough free memory
    blocks are available next to allocated memory, this function
    fails. */
config_depend_alwaysinline(CONFIG_MUTEK_MEMALLOC_SMART,
void *mem_resize(void *ptr, size_t size),
{
  return memory_allocator_resize(ptr, size);
});

#endif
