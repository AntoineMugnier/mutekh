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
#include <string.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
#include <arch/mem_checker.h>
#include <hexo/iospace.h>
#endif

#ifdef CONFIG_HEXO_MEMALLOC_ALGO

CONTAINER_FUNC(alloc_list, CLIST, static inline, alloc_list, list_entry);


#if defined(CONFIG_HEXO_MEMALLOC_ALGO_FIRSTFIT)

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

#elif defined(CONFIG_HEXO_MEMALLOC_ALGO_BESTFIT)

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


void *mem_alloc_region_pop(struct mem_alloc_region_s *region, size_t size)
{
  struct mem_alloc_header_s	*hdr;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&region->lock);

#ifdef CONFIG_SOCLIB_MEMCHECK
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_DISABLE, SOCLIB_MC_CHECK_REGIONS);
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
#endif

  /* find suitable free block */
  if ((hdr = mem_alloc_region_candidate(region, size)))
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

#ifdef CONFIG_SOCLIB_MEMCHECK
      cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
      cpu_mem_write_32(SOCLIB_MC_R1, (uint32_t)(hdr + 1));
      cpu_mem_write_32(SOCLIB_MC_R2, hdr->size - sizeof(*hdr));
      cpu_mem_write_32(SOCLIB_MC_REGION_UPDATE, SOCLIB_MC_REGION_ALLOC);
      cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
#endif

    }

#ifdef CONFIG_SOCLIB_MEMCHECK
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_ENABLE, SOCLIB_MC_CHECK_REGIONS);
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
#endif

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

#ifdef CONFIG_SOCLIB_MEMCHECK
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_DISABLE, SOCLIB_MC_CHECK_REGIONS);
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
#endif

  assert(hdr->size >= mem_hdr_size);

#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
  assert(hdr->signature == MEMALLOC_SIGNATURE);
#endif

  hdr->is_free = 1;

#ifdef CONFIG_HEXO_MEMALLOC_DEBUG
  memset(hdr + 1, 0xa5, hdr->size - sizeof(*hdr));
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_R1, (uint32_t)(hdr + 1));
  cpu_mem_write_32(SOCLIB_MC_R2, hdr->size - sizeof(*hdr));
  cpu_mem_write_32(SOCLIB_MC_REGION_UPDATE, SOCLIB_MC_REGION_FREE);
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
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

#ifdef CONFIG_HEXO_MEMALLOC_DEBUG
	  memset(next, 0xa5, mem_hdr_size);
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
#ifdef CONFIG_HEXO_MEMALLOC_DEBUG
	  memset(hdr, 0xa5, mem_hdr_size);
#endif
	}
    }

#ifdef CONFIG_SOCLIB_MEMCHECK
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_ENABLE, SOCLIB_MC_CHECK_REGIONS);
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
#endif

  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;
}

void mem_alloc_region_init(struct mem_alloc_region_s *region,
			   void *start, void *end)
{
  struct mem_alloc_header_s	*hdr = start;
  size_t			size = (uint8_t*)end - (uint8_t*)start;

  CPU_INTERRUPT_SAVESTATE_DISABLE;

#ifdef CONFIG_SOCLIB_MEMCHECK
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_DISABLE, SOCLIB_MC_CHECK_REGIONS);
  cpu_mem_write_32(SOCLIB_MC_R1, (uint32_t)start);
  cpu_mem_write_32(SOCLIB_MC_R2, size);
  cpu_mem_write_32(SOCLIB_MC_REGION_UPDATE, SOCLIB_MC_REGION_FREE);
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
#elif defined( CONFIG_HEXO_MEMALLOC_DEBUG )
  memset(hdr, 0xa5, size);
#endif

  /* init region struct */

  lock_init(&region->lock);
  alloc_list_init(&region->root);

#ifdef CONFIG_HEXO_MEMALLOC_STATS
  region->alloc_blocks = 0;
  region->free_size = size;
  region->free_blocks = 1;
#endif

  /* push initial block */

  assert(size > mem_hdr_size);

#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
  hdr->signature = MEMALLOC_SIGNATURE;
#endif
  hdr->size = size;
  hdr->is_free = 1;
  hdr->region = region;

  alloc_list_push(&region->root, hdr);

#ifdef CONFIG_SOCLIB_MEMCHECK
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_ENABLE, SOCLIB_MC_CHECK_REGIONS);
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
#endif

  CPU_INTERRUPT_RESTORESTATE;
}

#ifdef CONFIG_HEXO_MEMALLOC_GUARD

bool_t mem_alloc_chk(const char *str, uint8_t *data, uint8_t value)
{
  uintptr_t	i;
  bool_t	res = 0;

  for (i = 0; i < CONFIG_HEXO_MEMALLOC_GUARD_SIZE; i++)
    {
      if (data[i] != value)
	{
	  printf("%s: value mismatch at block %p offset %p: %02x\n", str, data, i, data[i]);
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
	uint8_t		*post = ((uint8_t*)item) + item->size - CONFIG_HEXO_MEMALLOC_GUARD_SIZE;

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


/************************************************************************/

#else /* !CONFIG_HEXO_MEMALLOC_ALGO */

void *mem_alloc_region_pop(struct mem_alloc_region_s *region, size_t size)
{
  void	*res, *next;

  lock_spin(&region->lock);

  res = region->next;

#ifdef CONFIG_HEXO_MEMALLOC_DEBUG
  memset(res, 0x5a, size);
#endif

  next = (uint8_t*)region->next + size;

  if (next > region->last)
    res = NULL;

  region->next = next;

  lock_release(&region->lock);

  return res;
}

void mem_alloc_region_push(void *address)
{
  /* no free() ! */
}

void mem_alloc_region_init(struct mem_alloc_region_s *region,
			   void *address, void *end)
{
  lock_init(&region->lock);

  region->next = address;
  region->last = end;
}

#endif /* CONFIG_HEXO_MEMALLOC_ALGO */





/************************************************************************/

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


