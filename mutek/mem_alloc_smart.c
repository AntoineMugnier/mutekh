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

#ifdef CONFIG_HEXO_MMU
#include <hexo/mmu.h>
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
#include <arch/mem_checker.h>
#endif

#ifdef CONFIG_MUTEK_MEMALLOC_SMART

/***************** Memory allocatable region management ******************/

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_clist.h>

#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
# define MEMALLOC_SIGNATURE	0x3a1b2ce1
#endif

/** memory block header */
struct mem_alloc_header_s
{
#ifdef CONFIG_MUTEK_MEMALLOC_SIGNED
  uint32_t			signature;
#endif
  struct mem_alloc_region_s	*region;
  uint8_t			is_free;
  /* block size including header */
  uintptr_t			size;
  CONTAINER_ENTRY_TYPE(CLIST)	list_entry;
};

static const size_t	mem_hdr_size = ALIGN_VALUE_UP(sizeof (struct mem_alloc_header_s),
						      CONFIG_MUTEK_MEMALLOC_ALIGN);

CONTAINER_TYPE(alloc_list, CLIST, struct mem_alloc_header_s, list_entry);

#define MEMALLOC_SPLIT_SIZE	(2 * mem_hdr_size + 16)

/** memory region handler */
struct mem_alloc_region_s
{
  lock_t		lock;
  alloc_list_root_t	root;
#ifdef CONFIG_MUTEK_MEMALLOC_STATS
  size_t		alloc_blocks;
  size_t		free_size;
  size_t		free_blocks;
#endif
};

struct mem_alloc_region_s mem_region_system;

/***************** Memory allocation interface ******************/


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


/******************************/
/*memory region allocator part*/
/******************************/


#include <gpct/cont_dlist.h>
#include <device/device.h>
#include <device/mem.h>
#include <device/driver.h>

struct mem_region_s
{
  CONTAINER_ENTRY_TYPE(DLIST) list_entry;
  uint_fast16_t cluster_id;
  struct mem_alloc_region_s *region;
};

CPU_LOCAL struct mem_alloc_region_s *local_cached_region;
CPU_LOCAL struct mem_alloc_region_s *local_uncached_region;

CPU_LOCAL struct mem_region_s *cached_region_queue;
CPU_LOCAL struct mem_region_s *uncached_region_queue;

CONTAINER_TYPE(region_queue, DLIST, struct mem_region_s, list_entry);
CONTAINER_KEY_TYPE(region_queue, SCALAR, cluster_id);
CONTAINER_FUNC(region_queue, DLIST, static, region_queue);
CONTAINER_KEY_FUNC(region_queue, DLIST, static, region_queue_id, cluster_id);

region_queue_root_t region_cached_list, region_uncached_list ;


#if ( defined(CONFIG_HEXO_DEVICE_TREE) && defined(CONFIG_FDT) )

#include <fdt/reader.h>

void mem_region_init(struct device_s *root, void *blob)
{
  struct dev_mem_info_s mem_info;

  region_queue_init(&region_cached_list);
  region_queue_init(&region_uncached_list);

  CONTAINER_FOREACH(device_list, CLIST, &root->children, {
      if (item->drv == NULL) CONTAINER_FOREACH_CONTINUE;
      if (item->drv->class == device_class_mem ){
	dev_mem_get_info(item, &mem_info);

	struct mem_region_s *region_item;
	region_item = mem_alloc( sizeof(struct mem_region_s), mem_scope_sys );
	    
	/* 	/\* looking for reserve memory space in the fdt: *\/ */
	    
	/* 	/\* First pass: the border memory space*\/	 */
	/* 	uint64_t addr=0; */
	/* 	uint64_t size=0; */
	/* 	uint_fast16_t i=0; */
	/* 	fdt_get_rsvmap(blob, i++, &addr, &size); */
	    
	/* 	while( !( addr == 0 && size == 0 ) ) */
	/* 	  { */
	/* 	    addr -= mem_hdr_size; */
	/* 	    size += mem_hdr_size; */
	/* 	    if( addr <= mem_info.base + mem_hdr_size + sizeof(struct mem_alloc_region_s) && addr+size > mem_info.base + mem_hdr_size + sizeof(struct mem_alloc_region_s) ) */
	/* 	      { */
	/* 		if( addr + size >= mem_info.base + mem_info.size - ( mem_hdr_size + sizeof(struct mem_alloc_region_s) ) ) */
	/* 		  CONTAINER_FOREACH_CONTINUE;		/\*all the region is reserve*\/ */
	/* 		mem_info.size -= (addr + size) - mem_info.base; */
	/* 		mem_info.base = (addr + size); */
	/* 	      } */
	/* 	    else if( addr < (mem_info.base + mem_info.size)  && addr+size >= (mem_info.base + mem_info.size - ( mem_hdr_size + sizeof(struct mem_alloc_region_s) ) ) ) */
	/*  	      { */
	/* 		mem_info.size -= (mem_info.base + mem_info.size) - addr; */
	/* 	      } */
	/* 	    fdt_get_rsvmap(blob, i++, &addr, &size); */
	/* 	  } */
	    
	/* when the start and the end is well know, create the memory region */
	if(mem_info.base != CONFIG_RAM_ADDR) 
	  region_item->region = mem_region_create(mem_info.base, mem_info.base + mem_info.size, mem_info.flags & DEV_MEM_CACHED);
	/* 	/\* Second pass: the reserve memory space into the region*\/ */
	/* 	i = 0; */
	/* 	fdt_get_rsvmap(blob, i++, &addr, &size); */
	/* 	while( !( addr == 0 && size == 0 ) ) */
	/* 	  { */
	/* 	    addr -= mem_hdr_size; */
	/* 	    size += mem_hdr_size; */
	/* 	    if( addr > mem_info.base && addr+size < mem_info.base+mem_info.size ) */
	/* 	      { */
	/* 		mem_reserve( region_item->region , (void*)(uint32_t)addr, size);  */
	/* 	      } */
	/* 	    fdt_get_rsvmap(blob, i++, &addr, &size); */
	/* 	  } */
	    
	/* Add the region to the corresponding list*/
	if(mem_info.flags & DEV_MEM_CACHED)
	  region_queue_push(&region_cached_list, region_item);
	else
	  region_queue_push(&region_uncached_list, region_item);
      }
    });

  //  region_queue_id_sort_ascend(&region_cached_list);
  //  region_queue_id_sort_ascend(&region_uncached_list);
  
  /**/
  /*FIXME: add CLUSTER suport, test if a cached and a uncached regions exist*/  
  mem_region_set_scope(mem_scope_cluster,region_queue_head(&region_uncached_list)->region );
  mem_region_set_scope(mem_scope_context,region_queue_head(&region_cached_list)->region );
  mem_region_set_scope(mem_scope_cpu,region_queue_head(&region_cached_list)->region );
}

#else

# warning memory region allocator init cannot be use without fdt. define your memory region init in hw_user_init
void mem_region_init(struct device_s *root, void *blob)
{
  mem_region_set_scope(mem_scope_cluster,&mem_region_system);
  mem_region_set_scope(mem_scope_context,&mem_region_system);
  mem_region_set_scope(mem_scope_cpu,&mem_region_system);
}

#endif /*CONFIG_HEXO_DEVICE_TREE*/

struct mem_alloc_region_s *mem_region_create(uintptr_t start, uintptr_t end, bool_t cached)
{
  struct mem_alloc_region_s *region;

  if( cached )
    region = mem_alloc( sizeof( struct mem_alloc_region_s ), mem_scope_sys );
  else
    {
      region = (struct mem_alloc_region_s *)start;
      start += sizeof( struct mem_alloc_region_s );
    }
  mem_alloc_region_init( region, (void *)start, (void *)end );

  return region;
}

void mem_region_set_scope(enum mem_scope_e scope, struct mem_alloc_region_s *region)
{
  switch( scope )
    {
    case mem_scope_cluster:
      CPU_LOCAL_SET(local_uncached_region, region);
      break;
    case mem_scope_context:
    case mem_scope_cpu: 
      CPU_LOCAL_SET(local_cached_region, region);
      break;
    }
}

struct mem_alloc_region_s *mem_region_get_scope(enum mem_scope_e scope)
{
  switch( scope )
    {
    case mem_scope_cluster:
      return CPU_LOCAL_GET(local_uncached_region);
      break;
    case mem_scope_context:
    case mem_scope_cpu:
      return CPU_LOCAL_GET(local_cached_region);
      break;
    case mem_scope_sys:
    case mem_scope_default:
    default:
      break;
    }
  return &mem_region_system;
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

static inline
struct mem_region_s *get_local_item(enum mem_scope_e scope, region_queue_root_t **region_list)
{
  switch( scope )
    {
    case mem_scope_sys:
    case mem_scope_default:
      *region_list = NULL;
      return NULL;
      break;
    case mem_scope_cluster:
      *region_list = &region_uncached_list;
      return CPU_LOCAL_GET(uncached_region_queue);
      break;
    case mem_scope_context:
    case mem_scope_cpu:
      *region_list = &region_cached_list;
      return CPU_LOCAL_GET(cached_region_queue);
      break;
    }
}

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
  
  hdr = mem_alloc_region_pop(region->region, size);
  
  while(hdr == NULL)
    {
      region = region_queue_next(&region_list, region);
      if( region == local ) break;
      hdr = mem_alloc_region_pop(region->region, size);
    }
  
  return hdr != NULL
    ? (uint8_t*)hdr + mem_hdr_size
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
    + CONFIG_MUTEK_MEMALLOC_GUARD_SIZE
#endif
    : NULL;

}

#endif /*CONFIG_MUTEK_NUMA*/

#endif /*CONFIG_MUTEK_MEMALLOC_SMART*/
