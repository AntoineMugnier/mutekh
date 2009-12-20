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

#include <mutek/memory_allocator.h>
#include <string.h>
#include <hexo/types.h>
#include <hexo/lock.h>
#include <hexo/endian.h>

#ifdef CONFIG_HEXO_MMU
#include <hexo/mmu.h>
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
#include <arch/mem_checker.h>
#endif

/****************************************/

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_clist.h>

#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
# define MEMALLOC_SIGNATURE	0x3a1b2ce1
#endif

/** memory block header */
struct memory_allocator_header_s
{
#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
  uint32_t			signature;
#endif
  struct memory_allocator_region_s	*region;
  uint8_t			is_free;
  /* block size including header */
  uintptr_t			size;
  CONTAINER_ENTRY_TYPE(CLIST)	list_entry;
};

static const size_t	mem_hdr_size = ALIGN_VALUE_UP(sizeof (struct memory_allocator_header_s),
						      CONFIG_MUTEK_MEMALLOC_ALIGN);

CONTAINER_TYPE(alloc_list, CLIST, struct memory_allocator_header_s, list_entry);

#define MEMALLOC_SPLIT_SIZE	(2 * mem_hdr_size + 16)

/** memory region handler */
struct memory_allocator_region_s
{
  lock_t		lock;
  alloc_list_root_t	root;
#ifdef CONFIG_MUTEK_MEMALLOC_STATS
  size_t		alloc_blocks;
  size_t		free_size;
  size_t		free_blocks;
#endif
};

static const size_t region_hdr_size = ALIGN_VALUE_UP ( sizeof( struct memory_allocator_region_s ),
						       CONFIG_MUTEK_MEMALLOC_ALIGN);

struct memory_allocator_region_s *default_region;
/***************************************/

CONTAINER_FUNC(alloc_list, CLIST, static inline, alloc_list, list_entry);

#if defined(CONFIG_MUTEK_MEMALLOC_ALGO_FIRSTFIT)

/* FIRST FIT allocation algorithm */

/** @internal */
static inline struct memory_allocator_header_s *
memory_allocator_candidate(struct memory_allocator_region_s *region, size_t size)
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

/** @internal */
static inline struct memory_allocator_header_s *
memory_allocator_candidate(struct memory_allocator_region_s *region, size_t size)
{
  struct memory_allocator_header_s	*best = NULL;

  CONTAINER_FOREACH(alloc_list, CLIST, &region->root,
  {
    if (item->is_free && item->size >= size &&
	((best == NULL) || (best->size > item->size)))
      best = item;
  });

  return best;
}

#endif

/** @internal */
static inline
struct memory_allocator_header_s *
memory_allocator_nolock_extend(struct memory_allocator_region_s *region, void *start, size_t size)
{
  struct memory_allocator_header_s *hdr = start;
  assert( hdr != NULL );
  
  if(hdr)
  {
    hdr->size=size;
    hdr->region=region;
    hdr->is_free=1;
#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
    hdr->signature = MEMALLOC_SIGNATURE;
#endif
    
    alloc_list_push(&region->root, hdr);
    
#ifdef CONFIG_MUTEK_MEMALLOC_STATS
    region->free_size += size;
    region->free_blocks++;
#endif
  }
  return hdr;
}

struct memory_allocator_header_s *
memory_allocator_extend(struct memory_allocator_region_s *region, void *start, size_t size)
{
    struct memory_allocator_header_s *h;

    CPU_INTERRUPT_SAVESTATE_DISABLE;
    lock_spin(&region->lock);
    h = memory_allocator_nolock_extend(region, start, size);
    lock_release(&region->lock);
    CPU_INTERRUPT_RESTORESTATE;

    return h;
}

# ifdef CONFIG_HEXO_MMU
/** @internal */
static inline struct memory_allocator_header_s *
mmu_region_nolock_extend(struct memory_allocator_region_s *region, size_t size)
{
  return memory_allocator_nolock_extend(region, vmem_ops.vpage_alloc(initial_ppage_region, size), size * CONFIG_HEXO_MMU_PAGESIZE);
}
# endif



void *memory_allocator_pop(struct memory_allocator_region_s *region, size_t size)
{
  struct memory_allocator_header_s	*hdr;

  size = mem_hdr_size
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
    + CONFIG_MUTEK_MEMALLOC_GUARD_SIZE * 2
#endif
    + ALIGN_VALUE_UP(size, CONFIG_MUTEK_MEMALLOC_ALIGN);


  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&region->lock);

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
#endif

  /* find suitable free block */
  if ((hdr = memory_allocator_candidate(region, size)) 
#ifdef CONFIG_HEXO_MMU
      || (hdr = mmu_region_nolock_extend(region, (size / CONFIG_HEXO_MMU_PAGESIZE) + 1))
#endif
      )
    {
      hdr->is_free = 0;

      /* check if split is needed */
      if (hdr->size >= size + MEMALLOC_SPLIT_SIZE)
	{
	  struct memory_allocator_header_s	*next = (void*)((uint8_t*)hdr + size);


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
      else
        {
          region->free_blocks--;
        }
      region->free_size -= hdr->size;
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

  return hdr != NULL
    ? (uint8_t*)hdr + mem_hdr_size
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
    + CONFIG_MUTEK_MEMALLOC_GUARD_SIZE
#endif
    : NULL;

}

void memory_allocator_push(void *address)
{
  struct memory_allocator_header_s	*hdr, *next, *prev;
  hdr = (void*)((uint8_t*)address
		 - mem_hdr_size
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
		 - CONFIG_MUTEK_MEMALLOC_GUARD_SIZE
#endif
		 );

  CPU_INTERRUPT_SAVESTATE_DISABLE;

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
#endif

  struct memory_allocator_region_s	*region = hdr->region;
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


size_t memory_allocator_getsize(void *ptr)
{
  size_t result;

#ifdef CONFIG_SOCLIB_MEMCHECK
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
#endif

  struct memory_allocator_header_s *hdr = (void*)((uint8_t*)ptr
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

/** @internal */
struct memory_allocator_header_s *get_hdr_for_rsv(struct memory_allocator_region_s *region, void *start, size_t size)
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

void *memory_allocator_reserve(struct memory_allocator_region_s *region, void *start, size_t size)
{
  struct memory_allocator_header_s	*hdr;

  size = ALIGN_VALUE_UP(size, CONFIG_MUTEK_MEMALLOC_ALIGN);

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
	  struct memory_allocator_header_s	*next = start;

#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
	  next->signature = MEMALLOC_SIGNATURE;
#endif
	  next->is_free = 0;
	  next->size = size;
	  next->region = hdr->region;
	  alloc_list_insert_post(&region->root, hdr, next);
	  
	  /*check is split is needed after the reserve space*/
	  if( hdr->size >= ( next - hdr + next->size + MEMALLOC_SPLIT_SIZE ) )
	    {
	      struct memory_allocator_header_s	*last;
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
      region->free_size -= hdr->size;
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

struct memory_allocator_region_s *
memory_allocator_init(struct memory_allocator_region_s *container_region,
			   void *start, void *end)
{
  struct memory_allocator_region_s *region;
  struct memory_allocator_header_s	*hdr;

    start = ALIGN_ADDRESS_UP(start, CONFIG_MUTEK_MEMALLOC_ALIGN);
    end = ALIGN_ADDRESS_LOW(end, CONFIG_MUTEK_MEMALLOC_ALIGN);

  if (container_region == NULL)
    {
      region = start;
      hdr = start + region_hdr_size;
    }
  else
    {
      region = memory_allocator_pop (container_region, sizeof (struct memory_allocator_region_s));
      hdr = start;
    }

  size_t size = (uint8_t*)end - (uint8_t*)hdr;
  
#ifdef CONFIG_SOCLIB_MEMCHECK
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
  soclib_mem_check_region_status(hdr, size, SOCLIB_MC_REGION_FREE);
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
  
  return region;
}

#ifdef CONFIG_MUTEK_MEMALLOC_GUARD

/** @internal */
bool_t memory_allocator_chk(const char *str, uint8_t *data, uint8_t value)
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

bool_t memory_allocator_guard_check(struct memory_allocator_region_s *region)
{
  bool_t	res = 0;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&region->lock);

  CONTAINER_FOREACH(alloc_list, CLIST, &region->root,
  {
    if (item->is_free)
      {
	uint8_t		*data = (void*)(item + 1);

	if (memory_allocator_chk("alloc free", data, 0xa5))
	  res = 1;
      }
    else
      {
	uint8_t		*pre = ((uint8_t*)item) + mem_hdr_size;
	uint8_t		*post = ((uint8_t*)item) + item->size - CONFIG_MUTEK_MEMALLOC_GUARD_SIZE;

	if (memory_allocator_chk("alloc pre ", pre, 0x5a) |
	    memory_allocator_chk("alloc post", post, 0x5a))
	  res = 1;
      }
  });

  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;

  return res;
}

#endif

error_t memory_allocator_stats(struct memory_allocator_region_s *region,
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

void memory_allocator_dump_used(struct memory_allocator_region_s *region, size_t ignore)
{
#ifdef CONFIG_SOCLIB_MEMCHECK
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
#endif

    CONTAINER_FOREACH(alloc_list, CLIST, &region->root, {
            if ( !item->is_free ) {
                if ( ignore ) {
                    --ignore;
                } else {
                    printk("Memory block at %p, %d bytes:\n",
                           item+1, item->size-sizeof(*item));
                    hexdumpk((void*)(((uintptr_t)(item+1))+0xc),
                            item->size-sizeof(*item)-0xc);
                    printk("\n");
                }
            }
        });

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
  CPU_INTERRUPT_RESTORESTATE;
#endif
}
