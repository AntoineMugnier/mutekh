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

#ifndef _MEMALLOC_PRIV_H_
#define _MEMALLOC_PRIV_H_

#include <hexo/endian.h>

#ifdef CONFIG_MUTEK_MEMALLOC_SMART

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

#else /*CONFIG_MUTEK_MEMALLOC_SMART*/ 

/***************** Memory allocatable region management ******************/

struct mem_alloc_region_s
{
  lock_t		lock;
  void			*next;
  void			*last;
};


#endif /*CONFIG_MUTEK_MEMALLOC_SMART*/

struct mem_alloc_region_s mem_region_system;

#endif
