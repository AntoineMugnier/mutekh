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

/*

    %config CONFIG_HEXO_MEMALLOC_ALGO_FIRSTFIT
    desc select memory allocation algorithm first fit
    default defined
    exclude CONFIG_HEXO_MEMALLOC_ALGO_BESTFIT
    %config end

    %config CONFIG_HEXO_MEMALLOC_ALGO_BESTFIT
    desc select memory allocation algorithm best fit
    default undefined
    exclude CONFIG_HEXO_MEMALLOC_ALGO_FIRSTFIT
    %config end

    %config CONFIG_HEXO_MEMALLOC_ALGO_META
    desc	meta configuration token used to impose requirements
    default	defined
    flags	mandatory noexport nodefine
    require	CONFIG_HEXO_MEMALLOC_ALGO_FIRSTFIT CONFIG_HEXO_MEMALLOC_ALGO_BESTFIT
    %config end

*/

#include <hexo/alloc.h>

CONTAINER_FUNC(static inline, alloc_list, DLIST, alloc_list, NOLOCK, list_entry);


#if defined(CONFIG_HEXO_MEMALLOC_ALGO_FIRSTFIT)

/* FIRST FIT allocation algorithm */

static inline struct mem_alloc_header_s *
mem_alloc_region_cadidate(struct mem_alloc_region_s *region, size_t size)
{
  CONTAINER_FOREACH(alloc_list, DLIST, NOLOCK, &region->root,
  {
    if (item->is_free && item->size >= size)
      return item;
  });

  return NULL;
}

#elif defined(CONFIG_HEXO_MEMALLOC_ALGO_BESTFIT)

/* BEST FIT allocation algorithm */

static inline struct mem_alloc_header_s *
mem_alloc_region_cadidate(struct mem_alloc_region_s *region, size_t size)
{
  struct mem_alloc_header_s	*best = NULL;

  CONTAINER_FOREACH(alloc_list, DLIST, NOLOCK, &region->root,
  {
    if (item->is_free && item->size >= size &&
	((best == NULL) || (best->size > item->size)))
      best = item;
  });

  return best;
}

#else
# error no memory allocation algorithm selected in config.h
#endif


void *mem_alloc_region_pop(struct mem_alloc_region_s *region, size_t size)
{
  struct mem_alloc_header_s	*hdr;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&region->lock);

  /* find suitable free block */
  if ((hdr = mem_alloc_region_cadidate(region, size)))
    {
      hdr->is_free = 0;

      /* check if split is needed */
      if (hdr->size >= size + MEMALLOC_SPLIT_SIZE)
	{
	  struct mem_alloc_header_s	*next = (void*)((uint8_t*)hdr + size);

#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
	  next->signature = MEMALLOC_SIGNATURE;
#endif
	  next->is_free = 1;
	  next->size = hdr->size - size;
	  next->region = hdr->region;
	  hdr->size = size;

	  alloc_list_insert_post(&region->root, hdr, next);
	}

#ifdef CONFIG_HEXO_MEMALLOC_STATS
      region->free_size -= size;
      region->alloc_blocks++;
#endif

#ifdef CONFIG_HEXO_MEMALLOC_DEBUG
      memset(hdr + 1, 0x5a, hdr->size - sizeof(*hdr));
#endif
    }

  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;

  return hdr;
}

void mem_alloc_region_push(void *address)
{
  struct mem_alloc_header_s	*hdr = address, *next, *prev;
  struct mem_alloc_region_s	*region = hdr->region;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&region->lock);

  assert(hdr->size >= sizeof(*hdr));

#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
  assert(hdr->signature == MEMALLOC_SIGNATURE);
#endif

  hdr->is_free = 1;

#ifdef CONFIG_HEXO_MEMALLOC_DEBUG
  memset(hdr + 1, 0xa5, hdr->size - sizeof(*hdr));
#endif

#ifdef CONFIG_HEXO_MEMALLOC_STATS
  region->free_size += hdr->size;
  region->alloc_blocks--;
  region->free_blocks++;
#endif

  if ((next = alloc_list_next(&region->root, hdr)))
    {
#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
      assert(next->signature == MEMALLOC_SIGNATURE);
#endif
      assert((uint8_t*)next == (uint8_t*)hdr + hdr->size);

      /* merge with next if free */
      if (next->is_free)
	{
	  hdr->size += next->size;
	  alloc_list_remove(&region->root, next);
#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
	  next->signature = 0;
#endif
#ifdef CONFIG_HEXO_MEMALLOC_STATS
	  region->free_blocks--;
#endif
	}
    }

  if ((prev = alloc_list_prev(&region->root, hdr)))
    {
#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
      assert(prev->signature == MEMALLOC_SIGNATURE);
#endif
      assert((uint8_t*)hdr == (uint8_t*)prev + prev->size);

      /* merge with prev if free */
      if (prev->is_free)
	{
	  prev->size += hdr->size;
	  alloc_list_remove(&region->root, hdr);
#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
	  hdr->signature = 0;
#endif
#ifdef CONFIG_HEXO_MEMALLOC_STATS
	  region->free_blocks--;
#endif
	}
    }

  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;
}

void mem_alloc_region_init(struct mem_alloc_region_s *region,
			   void *start, void *end)
{
  struct mem_alloc_header_s	*hdr = start;
  size_t			size = (uint8_t*)end - (uint8_t*)start;

  /* init region struct */

  lock_init(&region->lock);
  alloc_list_init(&region->root);

#ifdef CONFIG_HEXO_MEMALLOC_STATS
  region->alloc_blocks = 0;
  region->free_size = size;
  region->free_blocks = 1;
#endif

  /* push initial block */

  assert(size > sizeof(*hdr));

#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
  hdr->signature = MEMALLOC_SIGNATURE;
#endif
  hdr->size = size;
  hdr->is_free = 1;
  hdr->region = region;

  alloc_list_push(&region->root, hdr);
}

error_t mem_alloc_stats(struct mem_alloc_region_s *region,
			size_t *alloc_blocks,
			size_t *free_size,
			size_t *free_blocks)
{
#ifdef CONFIG_HEXO_MEMALLOC_STATS

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

