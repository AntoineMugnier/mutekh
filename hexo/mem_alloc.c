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

#include <hexo/alloc.h>

CONTAINER_FUNC(static inline, alloc_list, DLIST, alloc_list, NOLOCK, list_entry);

void *mem_alloc_region_pop(struct mem_alloc_region_s *region, size_t size)
{
  struct mem_alloc_header_s	*hdr = NULL;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&region->lock);

  /* find suitable free block */
  CONTAINER_FOREACH(alloc_list, DLIST, NOLOCK, &region->root,
  {
    if (hdr->is_free && hdr->size >= size)
      {
	hdr = item;
	goto exit_foreach;
      }
  });

 exit_foreach:

  hdr->is_free = 0;

  /* check if split is needed */
  if (hdr && hdr->size >= size + MEMALLOC_SPLIT_SIZE)
    {
      struct mem_alloc_header_s	*next = (void*)((uint8_t*)hdr + size);

#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
      next->signature = MEMALLOC_SIGNATURE;
#endif
      next->is_free = 1;
      next->size = hdr->size - size;
      hdr->size = size;

      alloc_list_insert_post(&region->root, hdr, next);
    }

#ifdef CONFIG_HEXO_MEMALLOC_STATS
  region->free_size -= size;
  region->alloc_blocks++;
#endif

  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;

  return hdr;
}

void mem_alloc_region_push(struct mem_alloc_region_s *region, void *address)
{
  struct mem_alloc_header_s	*hdr = address, *next, *prev;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&region->lock);

  next = alloc_list_next(&region->root, hdr);
  prev = alloc_list_prev(&region->root, hdr);

  assert(hdr->size >= sizeof(*hdr));

#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
  assert(hdr->signature == MEMALLOC_SIGNATURE);
#endif

  if (next)
    {
#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
      assert(next->signature == MEMALLOC_SIGNATURE);
#endif
      assert((uint8_t*)next == (uint8_t*)hdr + hdr->size);
    }

  if (prev)
    {
#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
      assert(prev->signature == MEMALLOC_SIGNATURE);
#endif
      assert(!prev || (uint8_t*)hdr == (uint8_t*)prev + prev->size);
    }

  hdr->is_free = 1;
#ifdef CONFIG_HEXO_MEMALLOC_STATS
  region->free_size += hdr->size;
  region->alloc_blocks--;
#endif

  /* merge with next if free */
  if (next && next->is_free)
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

  /* merge with prev if free */
  if (prev && prev->is_free)
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

  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;
}

void mem_alloc_region_init(struct mem_alloc_region_s *region,
			   void *address, size_t size)
{
  struct mem_alloc_header_s	*hdr = address;

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

  alloc_list_push(&region->root, hdr);
}

