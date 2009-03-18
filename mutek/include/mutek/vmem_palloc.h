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

# ifndef CONFIG_VMEM_PHYS_ALLOC
#  warning CONFIG_VMEM_PHYS_ALLOC is disabled !!!
# else

#include <hexo/types.h>
#include <hexo/error.h>

/*
 *  physical page allocator
 */

struct vmem_ppage_desc_s;
struct vmem_page_region_s;

error_t vmem_ppage_region_init(uintptr_t paddr, uintptr_t paddr_end);
void vmem_ppage_region_destroy();

bool_t vmem_ppage_inrange(uintptr_t paddr);
error_t vmem_ppage_alloc(uintptr_t *paddr);
error_t vmem_ppage_reserve(uintptr_t paddr, uintptr_t paddr_end);
uintptr_t vmem_ppage_refnew(uintptr_t paddr);
void vmem_ppage_refdrop(uintptr_t paddr);

# endif

#endif
