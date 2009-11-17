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
              Dimitri Refauvelet <dimitri.refauvelet@lip6.fr> (c) 2009
*/

#include <mutek/mem_alloc.h>
#include <string.h>
#include <hexo/types.h>
#include <hexo/lock.h>
#include <hexo/endian.h>
#include "memalloc.h"

#ifdef CONFIG_HEXO_MMU
#include <hexo/mmu.h>
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
#include <arch/mem_checker.h>
#endif

#ifdef CONFIG_MUTEK_MEMALLOC_SMART

CONTAINER_FUNC(alloc_list, CLIST, static inline, alloc_list, list_entry);

#if defined(CONFIG_MUTEK_MEMALLOC_ALGO_FIRSTFIT)

/* FIRST FIT allocation algorithm */

static inline struct mem_alloc_header_s *
mem_alloc_region_candidate(struct mem_alloc_region_s *region, size_t size)
{
  CONTAINER_FOREACH(alloc_list, CLIST, &region->root,
  {
    if (item->is_free && item->size >= size)
      return item;
  });

  return NULL;
}

#elif defined(CONFIG_MUTEK_MEMALLOC_ALGO_BESTFIT)

/* BEST FIT allocation algorithm */

static inline struct mem_alloc_header_s *
mem_alloc_region_candidate(struct mem_alloc_region_s *region, size_t size)
{
  struct mem_alloc_header_s	*best = NULL;

  CONTAINER_FOREACH(alloc_list, CLIST, &region->root,
  {
    if (item->is_free && item->size >= size &&
	((best == NULL) || (best->size > item->size)))
      best = item;
  });

  return best;
}

#endif

#ifdef CONFIG_HEXO_MMU
static inline struct mem_alloc_header_s *
mem_alloc_region_extend(struct mem_alloc_region_s *region, size_t size)
{
  struct mem_alloc_header_s	*hdr = NULL;
  
  hdr = vmem_ops.vpage_alloc(initial_ppage_region, size);
  if(hdr)
  {
    hdr->size=size * CONFIG_HEXO_MMU_PAGESIZE;
    hdr->region=region;
    hdr->is_free=1;
#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
    hdr->signature = MEMALLOC_SIGNATURE;
#endif
    
    alloc_list_push(&region->root, hdr);
    
#ifdef CONFIG_MUTEK_MEMALLOC_STATS
    region->free_size += ( size * CONFIG_HEXO_MMU_PAGESIZE );
    region->free_blocks++;
#endif
  }
  return hdr;
}
#endif

void *mem_alloc_region_pop(struct mem_alloc_region_s *region, size_t size)
{
  struct mem_alloc_header_s	*hdr;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&region->lock);

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
#endif

  /* find suitable free block */
  if ((hdr = mem_alloc_region_candidate(region, size)) 
#ifdef CONFIG_HEXO_MMU
      || (hdr = mem_alloc_region_extend(region, (size / CONFIG_HEXO_MMU_PAGESIZE) + 1))
#endif
      )
    {
      hdr->is_free = 0;

      /* check if split is needed */
      if (hdr->size >= size + MEMALLOC_SPLIT_SIZE)
	{
	  struct mem_alloc_header_s	*next = (void*)((uint8_t*)hdr + size);


#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
	  next->signature = MEMALLOC_SIGNATURE;
#endif
	  next->is_free = 1;
	  next->size = hdr->size - size;
	  next->region = hdr->region;
	  hdr->size = size;

	  alloc_list_insert_post(&region->root, hdr, next);
	}

#ifdef CONFIG_MUTEK_MEMALLOC_STATS
      region->free_size -= size;
      region->alloc_blocks++;
#endif

#ifdef CONFIG_MUTEK_MEMALLOC_DEBUG
      memset(hdr + 1, 0x5a, hdr->size - sizeof(*hdr));
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
      soclib_mem_check_region_status(hdr + 1, hdr->size - sizeof(*hdr), SOCLIB_MC_REGION_ALLOC);
#endif

    }

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_enable(SOCLIB_MC_CHECK_REGIONS);
#endif

  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;

  return hdr;
}

void mem_alloc_region_push(void *address)
{
  struct mem_alloc_header_s	*hdr = address, *next, *prev;

  CPU_INTERRUPT_SAVESTATE_DISABLE;

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
#endif

  struct mem_alloc_region_s	*region = hdr->region;
  lock_spin(&region->lock);

  assert(hdr->size >= mem_hdr_size);

#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
  assert(hdr->signature == MEMALLOC_SIGNATURE);
#endif

  hdr->is_free = 1;

#ifdef CONFIG_MUTEK_MEMALLOC_DEBUG
  memset(hdr + 1, 0xa5, hdr->size - sizeof(*hdr));
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_region_status(hdr + 1, hdr->size - sizeof(*hdr), SOCLIB_MC_REGION_FREE);
#endif

#ifdef CONFIG_MUTEK_MEMALLOC_STATS
  region->free_size += hdr->size;
  region->alloc_blocks--;
  region->free_blocks++;
#endif

  if ((next = alloc_list_next(&region->root, hdr)) == (void*)((uint8_t*)hdr + hdr->size))//next exist and next is contiguous with hdr
    {
#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
      assert(next->signature == MEMALLOC_SIGNATURE);
#endif
      assert((uint8_t*)next == (uint8_t*)hdr + hdr->size);

      /* merge with next if free */
      if (next->is_free)
	{
	  hdr->size += next->size;
	  alloc_list_remove(&region->root, next);
#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
	  next->signature = 0;
#endif
#ifdef CONFIG_MUTEK_MEMALLOC_STATS
	  region->free_blocks--;
#endif

#ifdef CONFIG_MUTEK_MEMALLOC_DEBUG
	  memset(next, 0xa5, mem_hdr_size);
#endif
	}
    }

  if ((prev = alloc_list_prev(&region->root, hdr)) && ((void*)hdr == (void*)((uint8_t*)prev + prev->size)))//prev exist and prev is contiguous with hdr
    {
#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
      assert(prev->signature == MEMALLOC_SIGNATURE);
#endif
      assert((uint8_t*)hdr == (uint8_t*)prev + prev->size);

      /* merge with prev if free */
      if (prev->is_free)
	{
	  prev->size += hdr->size;
	  alloc_list_remove(&region->root, hdr);
#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
	  hdr->signature = 0;
#endif
#ifdef CONFIG_MUTEK_MEMALLOC_STATS
	  region->free_blocks--;
#endif
#ifdef CONFIG_MUTEK_MEMALLOC_DEBUG
	  memset(hdr, 0xa5, mem_hdr_size);
#endif
	}
    }

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_enable(SOCLIB_MC_CHECK_REGIONS);
#endif

  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;
}

void mem_alloc_region_init(struct mem_alloc_region_s *region,
			   void *start, void *end)
{
  struct mem_alloc_header_s	*hdr = start;
  size_t			size = (uint8_t*)end - (uint8_t*)start;
  
#ifdef CONFIG_SOCLIB_MEMCHECK
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
  soclib_mem_check_region_status(start, size, SOCLIB_MC_REGION_FREE);
#elif defined( CONFIG_MUTEK_MEMALLOC_DEBUG )
  memset(hdr, 0xa5, size);
#endif

  /* init region struct */

  lock_init(&region->lock);
  alloc_list_init(&region->root);

#ifdef CONFIG_MUTEK_MEMALLOC_STATS
  region->alloc_blocks = 0;
  region->free_size = size;
  region->free_blocks = 1;
#endif

  /* push initial block */

  assert(size > mem_hdr_size);

#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
  hdr->signature = MEMALLOC_SIGNATURE;
#endif
  hdr->size = size;
  hdr->is_free = 1;
  hdr->region = region;

  alloc_list_push(&region->root, hdr);

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
  CPU_INTERRUPT_RESTORESTATE;
#endif

}


#ifdef CONFIG_MUTEK_MEMALLOC_GUARD

bool_t mem_alloc_chk(const char *str, uint8_t *data, uint8_t value)
{
  uintptr_t	i;
  bool_t	res = 0;

  for (i = 0; i < CONFIG_MUTEK_MEMALLOC_GUARD_SIZE; i++)
    {
      if (data[i] != value)
	{
	  printk("%s: value mismatch at block %p offset %p: %02x\n", str, data, i, data[i]);
	  res = 1;
	}
    }

  return res;
}

bool_t mem_alloc_region_guard_check(struct mem_alloc_region_s *region)
{
  bool_t	res = 0;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&region->lock);

  CONTAINER_FOREACH(alloc_list, CLIST, &region->root,
  {
    if (item->is_free)
      {
	uint8_t		*data = (void*)(item + 1);

	if (mem_alloc_chk("alloc free", data, 0xa5))
	  res = 1;
      }
    else
      {
	uint8_t		*pre = ((uint8_t*)item) + mem_hdr_size;
	uint8_t		*post = ((uint8_t*)item) + item->size - CONFIG_MUTEK_MEMALLOC_GUARD_SIZE;

	if (mem_alloc_chk("alloc pre ", pre, 0x5a) |
	    mem_alloc_chk("alloc post", post, 0x5a))
	  res = 1;
      }
  });

  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;

  return res;
}

#endif

error_t mem_alloc_stats(struct mem_alloc_region_s *region,
			size_t *alloc_blocks,
			size_t *free_size,
			size_t *free_blocks)
{
#ifdef CONFIG_MUTEK_MEMALLOC_STATS

  if (alloc_blocks)
    *alloc_blocks = region->alloc_blocks;

  if (free_size)
    *free_size = region->free_size;

  if (free_blocks)
    *free_blocks = region->free_blocks;

  return 0;
#else
  return -ENOTSUP;
#endif
}


/** free allocated memory block */
void mem_free(void *ptr)
{
  struct mem_alloc_header_s *hdr = (void*)((uint8_t*)ptr
		      - mem_hdr_size
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
		      - CONFIG_MUTEK_MEMALLOC_GUARD_SIZE
#endif
		      );

  mem_alloc_region_push(hdr);
}

size_t mem_alloc_getsize(void *ptr)
{
  size_t result;

#ifdef CONFIG_SOCLIB_MEMCHECK
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
#endif

  struct mem_alloc_header_s *hdr = (void*)((uint8_t*)ptr
		      - mem_hdr_size
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
		      - CONFIG_MUTEK_MEMALLOC_GUARD_SIZE
#endif
		      );

  result = hdr->size
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
    - CONFIG_MUTEK_MEMALLOC_GUARD_SIZE * 2
#endif
    - mem_hdr_size;

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_enable(SOCLIB_MC_CHECK_REGIONS);
  CPU_INTERRUPT_RESTORESTATE;
#endif

  return result;
}

struct mem_alloc_header_s *get_hdr_for_rsv(struct mem_alloc_region_s *region, void *start, size_t size)
{
  CONTAINER_FOREACH(alloc_list, CLIST, &region->root,
  {
    if (item->is_free && 
	( (void *)(item + 1) <= start ) &&
	( ((void*)item + item->size) >= (start + size) ))
      return item;
  });

  return NULL;
}

void *mem_reserve(struct mem_alloc_region_s *region, void *start, size_t size)
{
  struct mem_alloc_header_s	*hdr;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&region->lock);

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
#endif

  /* test if the reserve memory space is not already used and return the header which contains the reserve space*/
  if ((hdr = get_hdr_for_rsv(region, start, size)))
    {
      /* check if split is needed */
      if (hdr->size >= size + MEMALLOC_SPLIT_SIZE)
	{
	  struct mem_alloc_header_s	*next = start;

#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
	  next->signature = MEMALLOC_SIGNATURE;
#endif
	  next->is_free = 0;
	  next->size = size;
	  next->region = hdr->region;
	  alloc_list_insert_post(&region->root, hdr, next);
	  
	  /*check is split is needed after the reseve space*/
	  if( hdr->size >= ( next - hdr + next->size + MEMALLOC_SPLIT_SIZE ) )
	    {
	      struct mem_alloc_header_s	*last;
	      last = next + next->size;
	      last->is_free = 1;
	      last->size = (hdr + hdr->size) - (next + next->size);
	      last->region = hdr->region;
	      alloc_list_insert_post(&region->root, next, last);

	    }
	  hdr->size = next - hdr;
	  hdr = next;
	}
      else
	hdr->is_free = 0;
      
#ifdef CONFIG_MUTEK_MEMALLOC_STATS
      region->free_size -= size;
      region->alloc_blocks++;
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
      soclib_mem_check_region_status(hdr + 1, hdr->size - sizeof(*hdr), SOCLIB_MC_REGION_ALLOC);
#endif
    }

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_enable(SOCLIB_MC_CHECK_REGIONS);
#endif

  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;

  return hdr;
}

/********************** mem_alloc **********************/

/** allocate a new memory block in given region */
#ifndef CONFIG_MUTEK_NUMA 

void *mem_alloc(size_t size, enum mem_scope_e scope)
{
  struct mem_alloc_region_s *region;
  void *hdr;

  region = mem_region_get_scope(scope);

  size = mem_hdr_size
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
    + CONFIG_MUTEK_MEMALLOC_GUARD_SIZE * 2
#endif
    + ALIGN_VALUE_UP(size, CONFIG_MUTEK_MEMALLOC_ALIGN);

  hdr = mem_alloc_region_pop(region, size);

  return hdr != NULL
    ? (uint8_t*)hdr + mem_hdr_size
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
    + CONFIG_MUTEK_MEMALLOC_GUARD_SIZE
#endif
    : NULL;
}

#else /*CONFIG_MUTEK_NUMA*/

void *mem_alloc(size_t size, enum mem_scope_e scope)
{
  region_queue_root_t *region_list;
  struct mem_region_s *region = get_local_item(scope, &region_list);
  struct mem_region_s *local = region;

  void *hdr = NULL;

  size = mem_hdr_size
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
    + CONFIG_MUTEK_MEMALLOC_GUARD_SIZE * 2
#endif
    + ALIGN_VALUE_UP(size, CONFIG_MUTEK_MEMALLOC_ALIGN);
  
  if (region != NULL)
    {
      hdr = mem_alloc_region_pop(region->region, size);
  
      while(hdr == NULL)
	{
	  region = region_queue_next(&region_list, region);
	  if( region == local ) break;
	  hdr = mem_alloc_region_pop(region->region, size);
	}
    }
  else
    hdr = mem_alloc_region_pop(mem_region_get_scope(mem_scope_sys), size);

  return hdr != NULL
    ? (uint8_t*)hdr + mem_hdr_size
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
    + CONFIG_MUTEK_MEMALLOC_GUARD_SIZE
#endif
    : NULL;

}

#endif /*CONFIG_MUTEK_NUMA*/

#endif /*CONFIG_MUTEK_MEMALLOC_SMART*/
