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

    %config CONFIG_HEXO_MEMALLOC_ALIGN
    desc Memory allocation block address alignment
    flags mandatory
    default 0x20
    %config end

    %config CONFIG_HEXO_MEMALLOC_DEBUG
    desc When enabled all allocated and freed memory blocks will filled be with 0x5a and 0xa5 bytes
    %config end

    %config CONFIG_HEXO_MEMALLOC_STATS
    desc keep stats about allocated blocks count and size
    depend CONFIG_HEXO_MEMALLOC_ALGO
    %config end

    %config CONFIG_HEXO_MEMALLOC_SIGNED
    desc When enabled all memory block headers will include a special magic value
    depend CONFIG_HEXO_MEMALLOC_ALGO
    %config end

*/


#ifndef ALLOC_H_
#define ALLOC_H_

#include <hexo/types.h>
#include <hexo/lock.h>
#include <hexo/endian.h>


/***************** Memory allocatable region management ******************/

#ifdef CONFIG_HEXO_MEMALLOC_ALGO

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_dlist.h>

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
  CONTAINER_ENTRY_TYPE(DLIST)	list_entry;
};

static const size_t	mem_hdr_size = ALIGN_VALUE_UP(sizeof (struct mem_alloc_header_s),
						      CONFIG_HEXO_MEMALLOC_ALIGN);

CONTAINER_TYPE(alloc_list, DLIST, struct mem_alloc_header_s, NOLOCK, NOOBJ, list_entry);

#define MEMALLOC_SPLIT_SIZE	(2 * sizeof (struct mem_alloc_header_s) + 16)

/** memory region handler */
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





#else /* CONFIG_HEXO_MEMALLOC_ALGO */

static const size_t	mem_hdr_size = 0;

struct mem_alloc_region_s
{
  lock_t		lock;
  void			*next;
  void			*last;
};

#endif /* CONFIG_HEXO_MEMALLOC_ALGO */






void *mem_alloc_region_pop(struct mem_alloc_region_s *region, size_t size);

void mem_alloc_region_push(void *address);

void mem_alloc_region_init(struct mem_alloc_region_s *region,
			   void *address, void *end);

error_t mem_alloc_stats(struct mem_alloc_region_s *region,
			size_t *alloc_blocks,
			size_t *free_size,
			size_t *free_blocks);






/***************** Memory allocation interface ******************/

/** set default allocatable region */
static inline void
mem_alloc_set_default(struct mem_alloc_region_s *region);

/** allocate a new memory block in given region */
static inline void *
mem_alloc(size_t size, struct mem_alloc_region_s *region)
{
  void *hdr;

  size = mem_hdr_size + ALIGN_VALUE_UP(size, CONFIG_HEXO_MEMALLOC_ALIGN);
  hdr = mem_alloc_region_pop(region, size);
  return hdr != NULL ? (uint8_t*)hdr + mem_hdr_size : NULL;
}

/** free allocated memory block */
static inline void mem_free(void *ptr)
{
  void *hdr = (void*)((uint8_t*)ptr - mem_hdr_size);

  mem_alloc_region_push(hdr);
}

/** initialize memory subsystem. found in arch/name/mem_alloc.c */
void mem_init(void);





#include <arch/hexo/alloc.h>






#ifndef MEM_SCOPE_CPU
# define MEM_SCOPE_CPU		MEM_SCOPE_SYS
# if defined(CONFIG_SMP) && defined(CONFIG_CPU_CACHE) && !defined(CONFIG_CPU_CACHE_COHERENCY)
#  warning No CPU local memory region is available, cache problems may occur
# endif
#endif

#ifndef MEM_SCOPE_CLUSTER
# define MEM_SCOPE_CLUSTER	MEM_SCOPE_SYS
#endif

#ifndef MEM_SCOPE_CONTEXT
# define MEM_SCOPE_CONTEXT	MEM_SCOPE_SYS
#endif

#ifndef MEM_SCOPE_DEFAULT
# define MEM_SCOPE_DEFAULT	MEM_SCOPE_SYS
#endif

#endif

