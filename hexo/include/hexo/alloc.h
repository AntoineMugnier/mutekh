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

/**
 * @file
 * @module{Hexo}
 * @short Memory allocation stuff
 */

#ifndef ALLOC_H_
#define ALLOC_H_

#include <hexo/types.h>
#include <hexo/lock.h>
#include <hexo/endian.h>

#ifdef CONFIG_SOCLIB_MEMCHECK
#include <arch/mem_checker.h>
#endif

/***************** Memory allocatable region management ******************/

#ifdef CONFIG_HEXO_MEMALLOC_ALGO

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_clist.h>

#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
# define MEMALLOC_SIGNATURE	0x3a1b2ce1
#endif

/** memory block header */
struct mem_alloc_header_s
{
#ifdef CONFIG_HEXO_MEMALLOC_SIGNED
  uint32_t			signature;
#endif
  struct mem_alloc_region_s	*region;
  uint8_t			is_free;
  /* block size including header */
  uintptr_t			size;
  CONTAINER_ENTRY_TYPE(CLIST)	list_entry;
};

/** @internal @this give the size of the memory allocation header */
static const size_t	mem_hdr_size = ALIGN_VALUE_UP(sizeof (struct mem_alloc_header_s),
						      CONFIG_HEXO_MEMALLOC_ALIGN);

CONTAINER_TYPE(alloc_list, CLIST, struct mem_alloc_header_s, list_entry);
CONTAINER_FUNC(alloc_list, CLIST, static inline, alloc_list, list_entry);

#define MEMALLOC_SPLIT_SIZE	(2 * mem_hdr_size + 16)

/** @internal memory region handler */
struct mem_alloc_region_s
{
  lock_t		lock;
  alloc_list_root_t	root;
#ifdef CONFIG_HEXO_MEMALLOC_STATS
  size_t		alloc_blocks;
  size_t		free_size;
  size_t		free_blocks;
#endif
};





#else

static const size_t	mem_hdr_size = 0;

/** @internal memory region handler */
struct mem_alloc_region_s
{
  lock_t		lock;
  void			*next;
  void			*last;
};

#endif





/** @internal */
void *mem_alloc_region_pop(struct mem_alloc_region_s *region, size_t size);
/** @internal */
void mem_alloc_region_push(void *address);
/** @internal */
void mem_alloc_region_init(struct mem_alloc_region_s *region,
			   void *address, void *end);
/** @internal */
error_t mem_alloc_stats(struct mem_alloc_region_s *region,
			size_t *alloc_blocks,
			size_t *free_size,
			size_t *free_blocks);
/** @internal */
bool_t mem_alloc_region_guard_check(struct mem_alloc_region_s *region);





/***************** Memory allocation interface ******************/

/** set default allocatable region */
static inline void
mem_alloc_set_default(struct mem_alloc_region_s *region);

/** allocate a new memory block in given region */
static inline void *
mem_alloc(size_t size, struct mem_alloc_region_s *region)
{
  void *hdr;

  size = mem_hdr_size
#ifdef CONFIG_HEXO_MEMALLOC_GUARD
    + CONFIG_HEXO_MEMALLOC_GUARD_SIZE * 2
#endif
    + ALIGN_VALUE_UP(size, CONFIG_HEXO_MEMALLOC_ALIGN);

  hdr = mem_alloc_region_pop(region, size);

  return hdr != NULL
    ? (uint8_t*)hdr + mem_hdr_size
#ifdef CONFIG_HEXO_MEMALLOC_GUARD
    + CONFIG_HEXO_MEMALLOC_GUARD_SIZE
#endif
    : NULL;
}

/** free allocated memory block */
static inline void mem_free(void *ptr)
{
  struct mem_alloc_header_s *hdr = (void*)((uint8_t*)ptr
		      - mem_hdr_size
#ifdef CONFIG_HEXO_MEMALLOC_GUARD
		      - CONFIG_HEXO_MEMALLOC_GUARD_SIZE
#endif
		      );

  mem_alloc_region_push(hdr);
}


#ifdef CONFIG_HEXO_MEMALLOC_ALGO

static inline size_t mem_alloc_getsize(void *ptr)
{
  size_t result;

#ifdef CONFIG_SOCLIB_MEMCHECK
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  soclib_mem_check_disable(SOCLIB_MC_CHECK_REGIONS);
#endif

  struct mem_alloc_header_s *hdr = (void*)((uint8_t*)ptr
		      - mem_hdr_size
#ifdef CONFIG_HEXO_MEMALLOC_GUARD
		      - CONFIG_HEXO_MEMALLOC_GUARD_SIZE
#endif
		      );

  result = hdr->size
#ifdef CONFIG_HEXO_MEMALLOC_GUARD
    - CONFIG_HEXO_MEMALLOC_GUARD_SIZE * 2
#endif
    - mem_hdr_size;

#ifdef CONFIG_SOCLIB_MEMCHECK
  soclib_mem_check_enable(SOCLIB_MC_CHECK_REGIONS);
  CPU_INTERRUPT_RESTORESTATE;
#endif

  return result;
}

#endif

/** initialize memory subsystem. found in arch/name/mem_alloc.c */
void mem_init(void);

#ifdef CONFIG_HEXO_MEMALLOC_GUARD
/** @this checks memory allocation structures consistency */
static inline bool_t mem_guard_check(void) __attribute__((unused));
#endif

#include <arch/hexo/alloc.h>


#ifndef MEM_SCOPE_CPU
/** @this specifies processor local memory allocation, when applicable. */
# define MEM_SCOPE_CPU		MEM_SCOPE_SYS
# if defined(CONFIG_SMP) && defined(CONFIG_CPU_CACHE) && !defined(CONFIG_CPU_CACHE_COHERENCY)
#  warning No CPU local memory region is available, cache problems may occur
# endif
#endif

#ifndef MEM_SCOPE_CLUSTER
/** @this specifies cluster local memory allocation, when applicable. */
# define MEM_SCOPE_CLUSTER	MEM_SCOPE_SYS
#endif

#ifndef MEM_SCOPE_CONTEXT
/** @this specifies execution context local memory allocation. */
# define MEM_SCOPE_CONTEXT	MEM_SCOPE_SYS
#endif

#ifndef MEM_SCOPE_DEFAULT
/** @this specifies use of the current default allocation region.
    @see mem_alloc_set_default */
# define MEM_SCOPE_DEFAULT	MEM_SCOPE_SYS
#endif

#endif

