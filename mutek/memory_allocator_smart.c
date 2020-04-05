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
              Dimitri Refauvelet <dimitri.refauvelet@lip6.fr> (c) 2009
*/

#include <mutek/mem_alloc.h>
#include <string.h>
#include <hexo/types.h>
#include <hexo/lock.h>
#include <hexo/bit.h>
#include <mutek/printk.h>

#include <gct_platform.h>
#include <gct/container_clist.h>

#ifdef CONFIG_HEXO_MMU
#include <hexo/mmu.h>
#endif

#ifdef CONFIG_SOCLIB_MEMCHECK
#include <arch/soclib/mem_checker.h>
#endif

/********************************************************/
/*********** Structure and global declaration ***********/

#define GCT_CONTAINER_ALGO_free_list CLIST
#define GCT_CONTAINER_ALGO_block_list CLIST

/** memory block header */
struct memory_allocator_header_s
{
  union
  {
    GCT_CONTAINER_ENTRY(free_list, free_entry);
    struct
    {
      struct memory_allocator_region_s	*region;    
      void *free_null_marker;
    };
  };

  GCT_CONTAINER_ENTRY(block_list, block_entry);

#ifdef CONFIG_MUTEK_MEMALLOC_CRC
  uint32_t crc;
#endif
};

#ifdef CONFIG_MUTEK_MEMALLOC_CRC
static const size_t mem_hdr_size_no_crc = sizeof (struct memory_allocator_header_s) - sizeof (uint32_t);
#else
static const size_t mem_hdr_size_no_crc = sizeof (struct memory_allocator_header_s);
#endif

static const size_t mem_hdr_size_align = pow2_m1_up((sizeof (struct memory_allocator_header_s) - 1) |
                                                    (CONFIG_MUTEK_MEMALLOC_ALIGN - 1)) + 1;

GCT_CONTAINER_TYPES(free_list, struct memory_allocator_header_s *, free_entry);
GCT_CONTAINER_TYPES(block_list, struct memory_allocator_header_s *, block_entry);

#if defined(CONFIG_MUTEK_MEMALLOC_GUARD)
# define MEMALLOC_SPLIT_SIZE	( mem_hdr_size_align + 16 + CONFIG_MUTEK_MEMALLOC_GUARD_SIZE * 2 )
#else
# define MEMALLOC_SPLIT_SIZE	( mem_hdr_size_align + 16 )
#endif
/*************************/

/** memory region handler */
struct memory_allocator_region_s
{
  lock_t lock;
  free_list_root_t free_root;
  block_list_root_t block_root;
  size_t size;
#ifdef CONFIG_MUTEK_MEMALLOC_STATS
  size_t alloc_blocks;
  size_t free_size;
  size_t free_blocks;
#endif
};

static const size_t region_hdr_size = align_pow2_up ( sizeof( struct memory_allocator_region_s ),
						       CONFIG_MUTEK_MEMALLOC_ALIGN);

struct memory_allocator_region_s *default_region;

/********************************************************/
/******************* GPCT function **********************/

GCT_CONTAINER_FCNS(block_list, static inline, block_list,
                   init, destroy, head, next, prev, push, pushback, insert_next, remove);

GCT_CONTAINER_FCNS(free_list, static inline, free_list,
                   init, destroy, push, remove, prev, next);

/********************************************************/

static inline
bool_t header_is_alloc(struct memory_allocator_header_s *hdr);

static inline
bool_t header_is_endblock(struct memory_allocator_header_s *hdr);


/********************************************************/
/*************** size and address translator ******************/

/*@this return the memory block header corresponding to the given user
  memory address*/

static inline
void *mem2hdr(void *mem)
{
  return (void*)((uintptr_t)mem - mem_hdr_size_align
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
		 - CONFIG_MUTEK_MEMALLOC_GUARD_SIZE
#endif
		 );
}

/*@this return the useable memory address corresponding to given
  memory block header*/

static inline
void *hdr2mem(void *hdr)
{
  return (void *)((uintptr_t)hdr + mem_hdr_size_align
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
		  + CONFIG_MUTEK_MEMALLOC_GUARD_SIZE
#endif  
		  );
}

/*@this converts usable memory block size to actual block size*/

static inline
size_t size_alloc2real(size_t size)
{
  return mem_hdr_size_align
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
    + CONFIG_MUTEK_MEMALLOC_GUARD_SIZE * 2
#endif
    + align_pow2_up(size, CONFIG_MUTEK_MEMALLOC_ALIGN);
}

/*@this converts actual block size to usable memory block size*/

static inline
size_t size_real2alloc(size_t size)
{
  return size
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
    - CONFIG_MUTEK_MEMALLOC_GUARD_SIZE * 2
#endif
    - mem_hdr_size_align;
}

/*@this return the real size of a memory block*/

static inline
size_t header_get_size( block_list_root_t *root, struct memory_allocator_header_s *hdr)
{
  assert( !header_is_endblock(hdr) );
  struct memory_allocator_header_s *next = block_list_next(root,hdr);
  return (size_t)( ( uintptr_t ) next - ( uintptr_t ) hdr);
}

/********************************************************/
/*************** Memory check function ******************/

/*@this set the header's crc field*/

#ifdef CONFIG_MUTEK_MEMALLOC_CRC
static uint32_t memory_allocator_crc(const uint8_t *data, size_t len)
{
  uint32_t crc = 0;

  while (len--)
    {
      uint32_t w = *data++;
      uint_fast8_t j;

      for (j = 0; j < 8; j++)
        {
          crc = (crc >> 1) ^ (0x04c11db7 & ~(((w ^ crc) & 1) - 1));
          w >>= 1;
        }
    }

  return crc;
}
#endif

static inline
void memory_allocator_crc_set(struct memory_allocator_header_s *hdr)
{
#ifdef CONFIG_MUTEK_MEMALLOC_CRC
  hdr->crc = memory_allocator_crc((uint8_t*)hdr, mem_hdr_size_no_crc);
#endif
}

/*@this check the header's crc field*/

static inline
void memory_allocator_crc_check(struct memory_allocator_header_s *hdr)
{
#ifdef CONFIG_MUTEK_MEMALLOC_CRC

  uint32_t crc = memory_allocator_crc((uint8_t*)hdr, mem_hdr_size_no_crc);
  if (hdr->crc != crc)
    {
      printk("Memory allocator error: Header crc check failed at %p. Expected 0x%08x, got 0x%08x\n",
             hdr, hdr->crc, crc);
      abort();
    }
#endif
}

/*@this apply function to a list, and update previous and next header's crc  */

#if defined (CONFIG_MUTEK_MEMALLOC_CRC)

 #define MEM_LIST_FUNCTION_REM(function, list_name, hdr) do{		\
  struct memory_allocator_header_s *_prev, *_next;			\
  _prev = list_name##_list_prev(&region->list_name##_root, hdr);	\
  _next = list_name##_list_next(&region->list_name##_root, hdr);	\
  list_name##_list_##function(&region->list_name##_root, hdr);	\
  if (_prev)memory_allocator_crc_set(_prev);					\
  if (_next)memory_allocator_crc_set(_next);				\
 }while(0)

 #define MEM_LIST_FUNCTION_INS(function, list_name, hdr, other) do{		\
  struct memory_allocator_header_s *_prev, *_next;			\
  list_name##_list_##function(&region->list_name##_root, other, hdr); \
  _prev = list_name##_list_prev(&region->list_name##_root, hdr);	\
  _next = list_name##_list_next(&region->list_name##_root, hdr);	\
  if (_prev)memory_allocator_crc_set(_prev);				\
  if (_next)memory_allocator_crc_set(_next);				\
 }while(0)

 #define MEM_LIST_FUNCTION_PUSH(function, list_name, hdr) do{		\
  struct memory_allocator_header_s *_prev, *_next;			\
  list_name##_list_##function(&region->list_name##_root, hdr);	\
  _prev = list_name##_list_prev(&region->list_name##_root, hdr);	\
  _next = list_name##_list_next(&region->list_name##_root, hdr);	\
  if (_prev)memory_allocator_crc_set(_prev);				\
  if (_next)memory_allocator_crc_set(_next);				\
 }while(0)


#else

 #define MEM_LIST_FUNCTION_REM(function, list_name, hdr) do{		\
  list_name##_list_##function(&region->list_name##_root, hdr);	\
 }while(0)

 #define MEM_LIST_FUNCTION_INS(function, list_name, hdr, other) do{	\
  list_name##_list_##function(&region->list_name##_root, other, hdr); \
 }while(0) 

 #define MEM_LIST_FUNCTION_PUSH(function, list_name, hdr) do{		\
  list_name##_list_##function(&region->list_name##_root, hdr);	\
 }while(0)

#endif


/*@this set two guard zone, at start and end point, in the given memory block*/

static inline
void memory_allocator_guard_set(size_t size, struct memory_allocator_header_s *hdr)
{
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
  memset( (void*)( (uintptr_t) hdr + mem_hdr_size_align ),
	  0x55,
	  CONFIG_MUTEK_MEMALLOC_GUARD_SIZE );
  memset( (void*)( (uintptr_t)hdr + size - CONFIG_MUTEK_MEMALLOC_GUARD_SIZE),
	  0xaa,
	  CONFIG_MUTEK_MEMALLOC_GUARD_SIZE );
#endif
}

/*@this check the memory block's guard zone*/

static inline
void memory_allocator_guard_check(size_t size, struct memory_allocator_header_s *hdr)
{
#ifdef CONFIG_MUTEK_MEMALLOC_GUARD
  uint8_t r;

  assert(header_is_alloc(hdr));

  r = memcstcmp( (void*)( (uintptr_t)hdr + mem_hdr_size_align),
		 0x55, CONFIG_MUTEK_MEMALLOC_GUARD_SIZE );
  if ( r )
    {
      printk("Memory allocator error: Header guard head zone check failed at %p\n", hdr);
      abort();
    }

  r = memcstcmp( (void*)( (uintptr_t)hdr + size - CONFIG_MUTEK_MEMALLOC_GUARD_SIZE),
		 0xaa, CONFIG_MUTEK_MEMALLOC_GUARD_SIZE );
  if ( r )
    {
      printk("Memory allocator error: Header guard tail zone check failed at %p\n", hdr);
      abort();
    }

#endif  
}

/*@this set the whole memory block with a special value*/

static inline
void memory_allocator_scramble_set_alloc(size_t size, struct memory_allocator_header_s *hdr)
{
#ifdef CONFIG_MUTEK_MEMALLOC_SCRAMBLE
  memset( hdr2mem(hdr), 0x5a , size_real2alloc(size) );
#endif
}

static inline
void memory_allocator_scramble_set_free(size_t size, struct memory_allocator_header_s *hdr)
{
#ifdef CONFIG_MUTEK_MEMALLOC_SCRAMBLE
  memset( hdr + 1, 0xa5 , size - sizeof(*hdr) );
#endif
}

/*@this check if the memory block isn't corrupt*/

static inline
void memory_allocator_scramble_check(size_t size, struct memory_allocator_header_s *hdr)
{
#if defined(CONFIG_MUTEK_MEMALLOC_SCRAMBLE) && CONFIG_MUTEK_MEMALLOC_SCRAMBLE_CHECK > 0
  int_fast8_t res = 0;

  assert(!header_is_alloc(hdr));

  size_t s = size - sizeof(*hdr);
  uint8_t *m = (void*)(hdr + 1);

  if (s > 2 * CONFIG_MUTEK_MEMALLOC_SCRAMBLE_CHECK)
    {
      res |= memcstcmp( m, 0xa5 , CONFIG_MUTEK_MEMALLOC_SCRAMBLE_CHECK );
      res |= memcstcmp( m + s - CONFIG_MUTEK_MEMALLOC_SCRAMBLE_CHECK, 0xa5,
                        CONFIG_MUTEK_MEMALLOC_SCRAMBLE_CHECK );
    }
  else
    {
      res = memcstcmp( m, 0xa5 , s );
    }

  if (res)
    {
      printk("Memory allocator error: free memory scramble check failed in block %p\n", hdr);
      abort();
    }
#endif
}

static inline
void disable_memchecker()
{
#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
#endif
}

static inline
void enable_memchecker()
{
#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_enable(SOCLIB_MC_CHECK_REGIONS);
#endif
}

static inline
void memchecker_set_alloc(size_t size, void *hdr)
{
#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_region_status(hdr2mem(hdr),
				 size_real2alloc(size),
				 SOCLIB_MC_REGION_ALLOC);
#endif
}

static inline
void memchecker_set_free(size_t size, void *hdr)
{
#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_region_status(hdr2mem(hdr),
				 size_real2alloc(size),
				 SOCLIB_MC_REGION_FREE);
#endif
}

static inline
bool_t header_is_alloc(struct memory_allocator_header_s *hdr)
{
  return ( hdr->free_null_marker == NULL );
}

static inline
void header_set_alloc(struct memory_allocator_header_s *hdr, struct memory_allocator_region_s *region )
{
  hdr->free_null_marker = NULL;
  hdr->region = region;
}

static inline
bool_t header_is_endblock(struct memory_allocator_header_s *hdr)
{
  return ( hdr->region == NULL );
}

static inline
void header_set_endblock(struct memory_allocator_header_s *hdr)
{
  hdr->region = NULL;
  hdr->free_null_marker = NULL;
}

/********************************************************/
/********* Non-algorithmic internal function ************/

static inline
void update_region_stats(struct memory_allocator_region_s *region,
		  ssize_t size, uint32_t free_block, uint32_t alloc_block)
{
#ifdef CONFIG_MUTEK_MEMALLOC_STATS
  region->free_size += size;
  region->free_blocks += free_block;
  region->alloc_blocks += alloc_block;
#endif
} 

static inline
void init_region_stats(struct memory_allocator_region_s *region,
		  ssize_t size, uint32_t free_block, uint32_t alloc_block)
{
#ifdef CONFIG_MUTEK_MEMALLOC_STATS
  region->free_size = size;
  region->free_blocks = free_block;
  region->alloc_blocks = alloc_block;
#endif
} 

static inline
struct memory_allocator_header_s *
memory_allocator_nolock_extend(struct memory_allocator_region_s *region, void *start, size_t size)
{
  struct memory_allocator_header_s *hdr = start;
  struct memory_allocator_header_s *hdr_end = start + size - mem_hdr_size_align;
  size_t hdr_size = (void*)hdr_end - (void *)hdr;

  assert( hdr != NULL );
  
  MEM_LIST_FUNCTION_PUSH(pushback, block, hdr);
  MEM_LIST_FUNCTION_PUSH(push, free, hdr);
  
  header_set_endblock(hdr_end);
  MEM_LIST_FUNCTION_PUSH(pushback, block, hdr_end);

  memory_allocator_scramble_set_free(hdr_size, hdr);
  memory_allocator_crc_set(hdr);
  memory_allocator_crc_set(hdr_end);

  region->size += size;

  update_region_stats(region, size, 1, 0);

  return hdr;
}

# ifdef CONFIG_HEXO_MMU
static inline struct memory_allocator_header_s *
mmu_region_nolock_extend(struct memory_allocator_region_s *region, size_t size)
{
  return memory_allocator_nolock_extend(region, vmem_ops.vpage_alloc(initial_ppage_region, size), size * CONFIG_HEXO_MMU_PAGESIZE);
}
# endif


/********************************************************/
/*********** algorithmic internal function **************/

#if defined(CONFIG_MUTEK_MEMALLOC_ALGO_FIRSTFIT)

/* FIRST FIT allocation algorithm */

static inline struct memory_allocator_header_s *
memory_allocator_candidate(struct memory_allocator_region_s *region, size_t size)
{
  GCT_FOREACH(free_list, &region->free_root, item,
  {
    if ( header_get_size(&region->block_root, item) >= size)
      return item;
  });

  return NULL;
}

#elif defined(CONFIG_MUTEK_MEMALLOC_ALGO_BESTFIT)

/* BEST FIT allocation algorithm */

static inline struct memory_allocator_header_s *
memory_allocator_candidate(struct memory_allocator_region_s *region, size_t size)
{
  struct memory_allocator_header_s	*best = NULL;
  size_t item_size, best_size;
  GCT_FOREACH(free_list, &region->free_root, item,
  {
    item_size = header_get_size(&region->block_root, item);
    if ( item_size >= size &&
	((best == NULL) || (best_size > item_size)))
      {
	best = item;
	best_size = item_size;
      }
  });

  return best;
}

#endif

static inline 
struct memory_allocator_header_s *get_hdr_for_rsv(struct memory_allocator_region_s *region, void *start, size_t size)
{
  GCT_FOREACH(free_list, &region->free_root, item,
  {
    if (((void *)(item + 1) <= start ) &&
	( ((void*)item + header_get_size(&region->block_root, item)) >= (start + size) ))
      return item;
  });

  return NULL;
}

/********************************************************/
/******************** API function **********************/

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

void *memory_allocator_resize(void *address, size_t size)
{
  struct memory_allocator_header_s *hdr = mem2hdr(address);

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  disable_memchecker();

  struct memory_allocator_region_s *region = hdr->region;

  lock_spin(&region->lock);
  
  assert( header_is_alloc(hdr) && !header_is_endblock(hdr) );

  size = size_alloc2real(size);
  
  size_t hdr_size = header_get_size(&region->block_root, hdr);

  memory_allocator_guard_check(hdr_size, hdr);

  ssize_t diff = (size - hdr_size);
  
  if (diff == 0)
    goto done;
  
  struct memory_allocator_header_s *next = block_list_next(&region->block_root, hdr);
  
  if ( next && !header_is_alloc(next) )
    {
      /* next block is free */
      size_t next_size = header_get_size(&region->block_root, next);

      memory_allocator_scramble_check(next_size, next);
      
      if (diff > 0 && next_size < (size_t)diff)
	{
          /* next block is not large enough to allocate more memory */
	  address = NULL;
	}
      else
	{
	  MEM_LIST_FUNCTION_REM(remove, free, next);
	  MEM_LIST_FUNCTION_REM(remove, block, next);
	  
	  memchecker_set_free(hdr_size, hdr);

	  if (diff < 0 || next_size >= diff + MEMALLOC_SPLIT_SIZE)
	    {
              /* keep free block but change its start adress */
	      next = (void*)((uintptr_t)hdr + size);
	      next_size -= diff;

	      MEM_LIST_FUNCTION_INS(insert_next, block, next, hdr);
	      MEM_LIST_FUNCTION_PUSH(push, free, next);

	      memory_allocator_crc_set(next);
              if (diff < 0)
                memory_allocator_scramble_set_free(sizeof(*hdr) - diff, next);

	      update_region_stats(region, -diff, 0, 0);
	    }
	  else
	    {
              /* use the whole free block */
              size = hdr_size + next_size;
              update_region_stats(region, -next_size, -1, 0);
	    }

          memchecker_set_alloc(size, hdr);
	  memory_allocator_crc_set(hdr);
          memory_allocator_guard_set(size, hdr);
          memory_allocator_scramble_set_alloc(size, hdr);
	}
    }
  else
    {
      /* next block is allocated or endblock */
      if (diff > 0)
	{
	  address = NULL;
	}
      else
	{
	  if (size + MEMALLOC_SPLIT_SIZE <= hdr_size)
	    {
              /* shrink allocated block */
              memchecker_set_free(hdr_size, hdr);

	      memory_allocator_guard_set(size, hdr);

	      next = (void*)((uintptr_t)hdr + size);
	      size_t next_size = -diff;
	      MEM_LIST_FUNCTION_INS(insert_next, block, next, hdr);
	      MEM_LIST_FUNCTION_PUSH(push, free, next);

	      memory_allocator_crc_set(next);
	      memory_allocator_scramble_set_free(next_size, next);

	      update_region_stats(region, next_size, 1, 0);
	      memchecker_set_alloc(size, hdr);
	    }
	}
    }

 done:
  enable_memchecker();
  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;

  return address;
}

void *memory_allocator_pop(struct memory_allocator_region_s *region, size_t size, size_t align)
{
  struct memory_allocator_header_s	*hdr;

  assert((align & (align - 1)) == 0); /* check align is a power of 2 */

  size = size_alloc2real(size);

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&region->lock);

  disable_memchecker();

  /* find suitable free block */
  hdr = memory_allocator_candidate(region, align > mem_hdr_size_align ? size + align : size);

#ifdef CONFIG_HEXO_MMU
  if (!hdr)
    hdr = mmu_region_nolock_extend(region, (size / CONFIG_HEXO_MMU_PAGESIZE) + 1);
#endif

  if (hdr)
    {
      assert ( !header_is_alloc(hdr) );
      size_t hdr_size = header_get_size(&region->block_root, hdr);

      assert ( size <= hdr_size );

      memory_allocator_scramble_check(hdr_size, hdr);
      memory_allocator_crc_check(hdr);

      /* may keep the free block and create a new allocated block inside due to alignment constraint */
      if (align > mem_hdr_size_align)
        {
          uint8_t *mem = (uint8_t*)hdr2mem(hdr);
          uint8_t *amem = (uint8_t*)address_align_up(mem, align);
          uintptr_t offset = amem - mem;

          if (offset)
            {
              assert(offset >= mem_hdr_size_align);

              struct memory_allocator_header_s *next = mem2hdr(amem);
              MEM_LIST_FUNCTION_INS(insert_next, block, next, hdr);

              memory_allocator_crc_set(hdr);
              memory_allocator_scramble_set_free(offset, hdr);
              update_region_stats(region, 0, 1, 0);

              hdr = next;
              hdr_size -= offset;

              goto keep_free;
            }
        }

      MEM_LIST_FUNCTION_REM(remove, free, hdr);
    keep_free:
      header_set_alloc(hdr, region);
      
      /* check if split is needed */
      if (hdr_size >= size + MEMALLOC_SPLIT_SIZE)
	{
	  struct memory_allocator_header_s	*next = (void*)((uintptr_t)hdr + size);

	  MEM_LIST_FUNCTION_INS(insert_next, block, next, hdr);
	  MEM_LIST_FUNCTION_PUSH(push, free, next);

          memory_allocator_crc_set(next);
          update_region_stats(region, -size, 0, 1);
	}
      else
        {
          /* allocate the whole free block */
          update_region_stats(region, -hdr_size, -1, 1);
          size = hdr_size;
        }

      memory_allocator_crc_set(hdr);
      memory_allocator_scramble_set_alloc(size, hdr);
      memory_allocator_guard_set(size, hdr);
      memchecker_set_alloc(size, hdr);
    }

  enable_memchecker();

  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;

  return hdr != NULL ? (uint8_t*)hdr2mem(hdr) : NULL;
}

void memory_allocator_push(void *address)
{
  struct memory_allocator_header_s	*next, *prev, *hdr = mem2hdr(address);

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  disable_memchecker();

  struct memory_allocator_region_s	*region = hdr->region;

  lock_spin(&region->lock);

  assert( header_is_alloc(hdr) && !header_is_endblock(hdr) );

  size_t size = header_get_size(&region->block_root, hdr);

  memory_allocator_guard_check(size, hdr);
  memory_allocator_crc_check(hdr);

  next = block_list_next(&region->block_root, hdr);
  if (next)
    memory_allocator_crc_check(next);

  prev = block_list_prev(&region->block_root, hdr);
  if (prev)  
    memory_allocator_crc_check(prev);

  memchecker_set_free(size, hdr);
  memory_allocator_scramble_set_free(size, hdr);

  if ( next && ! header_is_alloc(next) )
    {
      /* merge with next free block */
      MEM_LIST_FUNCTION_REM(remove, free, next);
      MEM_LIST_FUNCTION_REM(remove, block, next);
#ifdef CONFIG_MUTEK_MEMALLOC_SCRAMBLE
      memset( next, 0xa5 , sizeof(*next) );
#endif
      update_region_stats(region, 0, -1, 0);
    }

  if ( prev && ! header_is_alloc(prev) )
    {
      /* merge with previous free block */
      MEM_LIST_FUNCTION_REM(remove, block, hdr);
      memory_allocator_crc_set(prev);
#ifdef CONFIG_MUTEK_MEMALLOC_SCRAMBLE
      memset( hdr, 0xa5 , sizeof(*hdr) );
#endif
      hdr = prev;
    }
  else
    {
      MEM_LIST_FUNCTION_PUSH(push, free, hdr);
      memory_allocator_crc_set(hdr);
      update_region_stats(region, 0, 1, 0);
    }

  update_region_stats(region, size, 0, -1);

  size = header_get_size(&region->block_root, hdr);

  enable_memchecker();
  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;
}

size_t memory_allocator_getsize(void *ptr)
{
  size_t result;

  struct memory_allocator_header_s *hdr = mem2hdr(ptr);

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  disable_memchecker();
  lock_spin(& (hdr->region->lock) );

  assert( header_is_alloc(hdr) && !header_is_endblock(hdr) );

  size_t size = header_get_size(&hdr->region->block_root, hdr);

  memory_allocator_guard_check(size, hdr);
  memory_allocator_crc_check(hdr);

  result = size_real2alloc(size);

  lock_release(& (hdr->region->lock) );
  enable_memchecker();
  CPU_INTERRUPT_RESTORESTATE;

  return result;
}

void *memory_allocator_reserve(struct memory_allocator_region_s *region, void *start_, size_t size)
{
  struct memory_allocator_header_s	*hdr;

  uint8_t *start = start_;
  uint8_t *end = (uint8_t*)start + size;
  uint8_t *h = (void*)block_list_head(&region->block_root);

  if ((uint8_t*)hdr2mem(h) > start)
    return NULL;

  start = address_align_down(mem2hdr(start), mem_hdr_size_align);
  end = address_align_up(end + CONFIG_MUTEK_MEMALLOC_GUARD_SIZE, mem_hdr_size_align);

  assert(start < end);
  size = end - start;

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&region->lock);

  disable_memchecker();

  /* test if the reserve memory space is not already used and
     return the header which contains the reserve space */
  hdr = get_hdr_for_rsv(region, start, size);

  if (hdr)
    {
      assert( !header_is_alloc(hdr) );

      size_t hdr_size = header_get_size(&region->block_root, hdr);

      memory_allocator_scramble_check(hdr_size, hdr);
      memory_allocator_crc_check(hdr);

      /* keep previous free block ? */
      if (MEMALLOC_SPLIT_SIZE < (start - (uint8_t*)hdr))
	{
          struct memory_allocator_header_s *prev = hdr;
	  hdr = mem2hdr(start);

	  size_t prev_size = (uint8_t*)hdr - (uint8_t*)prev;
	  MEM_LIST_FUNCTION_INS(insert_next, block, hdr, prev);

          memory_allocator_crc_set(prev);
          memory_allocator_scramble_set_free(prev_size, prev);
          update_region_stats(region, 0, 1, 0);

          hdr_size -= prev_size;
        }
      else
        {
	  MEM_LIST_FUNCTION_REM(remove, free, hdr);
          start = (void*)hdr;
          size = end - start;
        }

      header_set_alloc(hdr, region);

      /* keep next free block ? */
      if (end + MEMALLOC_SPLIT_SIZE < (uint8_t*)hdr + hdr_size)
        {
          struct memory_allocator_header_s *next = (void*)((uintptr_t)hdr + size);
	      
          MEM_LIST_FUNCTION_INS(insert_next, block, next, hdr);
          MEM_LIST_FUNCTION_PUSH(push, free, next); 

          memory_allocator_crc_set(next);
          update_region_stats(region, -size, 0, 1);
        }
      else
        {
          update_region_stats(region, -hdr_size, -1, 1);
          size = hdr_size;
        }

      memory_allocator_crc_set(hdr);
      memory_allocator_scramble_set_alloc(size, hdr);
      memory_allocator_guard_set(size, hdr);
      memchecker_set_alloc(size, hdr);
    }

  enable_memchecker();

  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;

  return hdr != NULL ? (uint8_t*)hdr2mem(hdr) : NULL;
}

struct memory_allocator_region_s *
memory_allocator_init(struct memory_allocator_region_s *container_region,
			   void *start, void *end)
{
  struct memory_allocator_region_s *region;
  struct memory_allocator_header_s	*hdr, *hdr_end;

    start = address_align_up(start, CONFIG_MUTEK_MEMALLOC_ALIGN);
    end = address_align_down(end, CONFIG_MUTEK_MEMALLOC_ALIGN);

  if (container_region == NULL)
    {
      region = start;
      hdr = start + region_hdr_size;
    }
  else
    {
      region = memory_allocator_pop(container_region, sizeof (struct memory_allocator_region_s), 1);
      hdr = start;
    }

  hdr_end = end - mem_hdr_size_align;

  size_t size = (uintptr_t)hdr_end - (uintptr_t)hdr;
  
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  disable_memchecker();


  /* init region struct */

  lock_init(&region->lock);
  region->size = size;
  block_list_init(&region->block_root);
  free_list_init(&region->free_root);
  
  init_region_stats(region, size, 1, 0);
  
  /* push initials blocks */
  
  assert(size > mem_hdr_size_align);
  
  header_set_endblock(hdr_end);

  block_list_push(&region->block_root, hdr_end);  

  block_list_push(&region->block_root, hdr);
  free_list_push(&region->free_root, hdr);

  memory_allocator_crc_set(hdr_end);  
  memory_allocator_crc_set(hdr);

  size_t size = header_get_size(&region->block_root, hdr);
  memory_allocator_scramble_set_free(size, hdr);

  memchecker_set_free(size, hdr);
  enable_memchecker();
  CPU_INTERRUPT_RESTORESTATE;
  
  return region;
}

void memory_allocator_region_check(struct memory_allocator_region_s *region)
{
  size_t header_size = 0;
  size_t header_count = 0;
  bool_t last_is_free = 0;

#ifdef CONFIG_MUTEK_MEMALLOC_STATS
  size_t alloc_blocks = 0;
  size_t free_size = 0;
  size_t free_blocks = 0;
#endif

  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&region->lock);
  disable_memchecker();

  GCT_FOREACH(block_list, &region->block_root, item,
  {
    memory_allocator_crc_check(item);
    if (! header_is_endblock(item) )
      {
	size_t hdr_size = header_get_size(&region->block_root, item);
	header_size += hdr_size;
	header_count++;

        if (header_is_alloc(item))
          {
            last_is_free = 0;
#ifdef CONFIG_MUTEK_MEMALLOC_STATS
            alloc_blocks++;
#endif
            memory_allocator_guard_check(hdr_size, item);
          }
        else
          {
#ifdef CONFIG_MUTEK_MEMALLOC_STATS
            free_size += hdr_size;
            free_blocks++;
#endif
            if (last_is_free)
              printk("Memory allocator: Found contiguous free blocks at %p\n", item);
            last_is_free = 1;
            memory_allocator_scramble_check(hdr_size, item);
          }
      }
  });

#ifdef CONFIG_MUTEK_MEMALLOC_STATS
  assert(region->alloc_blocks == alloc_blocks);
  assert(region->free_size == free_size);
  assert(region->free_blocks == free_blocks);
#endif

  printk("Memory allocator: Check done on %d headers, with %d total size\n", header_count, header_size);

  enable_memchecker();
  lock_release(&region->lock);
  CPU_INTERRUPT_RESTORESTATE;
}

size_t memory_allocator_region_size(struct memory_allocator_region_s *region)
{
  return region->size;
}

void* memory_allocator_region_address(struct memory_allocator_region_s *region)
{
  return block_list_head(&region->block_root);
}

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

#ifdef CONFIG_MUTEK_PRINTK
void memory_allocator_dumpk(struct memory_allocator_region_s *region)
{
  GCT_FOREACH(block_list, &region->block_root, item,
  {
    if (header_is_endblock(item))
      {
        printk(" END\n");
      }
    else
      {
        logk(" %c at %p: %zu bytes",
               header_is_alloc(item) ? 'A' : 'F',
               item, header_get_size(&region->block_root, item));
      }
  });
}
#endif

#ifdef CONFIG_MUTEK_SHELL

#include <mutek/shell.h>

TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_blocks);
TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_blocks)
{
  struct memory_allocator_region_s *region = default_region;

  GCT_FOREACH(block_list, &region->block_root, item,
  {
    if (header_is_endblock(item))
      {
        termui_con_printf(con, " END\n");
      }
    else
      {
        termui_con_printf(con, " %c at %p: %zu bytes\n",
                          header_is_alloc(item) ? 'A' : 'F',
                          item, header_get_size(&region->block_root, item));
      }
  });
  return 0;
}

# ifdef CONFIG_MUTEK_MEMALLOC_STATS
TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_stats);
TERMUI_CON_COMMAND_PROTOTYPE(shell_mem_stats)
{
  struct memory_allocator_region_s *region = default_region;

  termui_con_printf(con,
                    "Allocated blocks: %zu\n"
                    "Free blocks:      %zu\n"
                    "Free bytes:       %zu\n",
                    region->alloc_blocks,
                    region->free_blocks,
                    region->free_size);
  return 0;
}
# endif
#endif

