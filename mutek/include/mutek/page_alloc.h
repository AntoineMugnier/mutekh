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

#ifndef VMEM_PALLOC_H_
#define VMEM_PALLOC_H_

/**
 * @file
 * @module{Mutek}
 * @short Physical pages allocator algorithms
 */

# ifndef CONFIG_VMEM_PHYS_ALLOC
#  warning CONFIG_VMEM_PHYS_ALLOC is disabled !!!
# else

#include <hexo/types.h>
#include <hexo/error.h>

/*
 *  physical page allocator
 */

struct vmem_page_region_s
{
  /** physical address of memory region */
  uintptr_t			paddr;
  /** memory region size */
  size_t			size;
  /** memory region page count */
  size_t			count;
  /** memory region free page count */
  size_t			free_count;
  /** first free page */
  uint_fast32_t			free_head;
  /** page allocation table */
  uint_fast32_t			*table;

  lock_t			lock;
};

/** Init a physical pages memory allocator region. */
error_t ppage_region_init(struct vmem_page_region_s *r, uintptr_t paddr, uintptr_t paddr_end);

/** Destroy a physical pages memory allocator region. */
void ppage_region_destroy(struct vmem_page_region_s *r);

/** Check if a physical address is in region range. */
bool_t ppage_inrange(struct vmem_page_region_s *r, uintptr_t paddr);

/** Allocate a free physical page in region and set paddr value. */
error_t ppage_alloc(struct vmem_page_region_s *r, uintptr_t *paddr);

/** Try to reserve all pages in pysical address range. All pages must be free. */
error_t ppage_reserve(struct vmem_page_region_s *r, uintptr_t paddr, uintptr_t paddr_end);

/** Get a new reference to an already allocated physical page. */
uintptr_t ppage_refnew(struct vmem_page_region_s *r, uintptr_t paddr);

/** Drop a reference to an allocated physical page, page is marked as free if counter reach 0. */
void ppage_refdrop(struct vmem_page_region_s *r, uintptr_t paddr);

# endif

#endif
