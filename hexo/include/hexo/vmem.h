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

#ifndef VMEM_H_
#define VMEM_H_

/*

%config CONFIG_HEXO_VMEM
desc Enable virtual memory support
default undefined
require CONFIG_HEXO_VMEM_PAGESIZE
require CONFIG_ARCH_IBMPC
%config end

%config CONFIG_HEXO_VMEM_PAGESIZE
desc Virtual memory physical page size
parent CONFIG_HEXO_VMEM
default 4096
%config end

%config CONFIG_HEXO_VMEM_USERLIMIT
desc Virtual memory minimal user accessible address
parent CONFIG_HEXO_VMEM
default 0x40000000
%config end

*/


/***********************************************************************
 *  main virtual memory interface
 */

#include "types.h"
#include "error.h"

struct vmem_context_s;
struct vmem_page_s;
typedef uint8_t vmem_pageattr_t;

#define VMEM_PAGE_ATTR_R 0x01
#define VMEM_PAGE_ATTR_W 0x02
#define VMEM_PAGE_ATTR_X 0x04
#define VMEM_PAGE_ATTR_CACHED 0x08
#define VMEM_PAGE_ATTR_USERLEVEL 0x10
#define VMEM_PAGE_ATTR_DIRTY 0x20
#define VMEM_PAGE_ATTR_ACCESSED 0x40
#define VMEM_PAGE_ATTR_PRESENT 0x80

/* switch to virtual memory mode by enabling mmu, identity */
void vmem_enable(uintptr_t start, uintptr_t end);

/* create a memory context and initialize context object */
error_t vmem_context_init(struct vmem_context_s *ctx);

/* switch to a memory context */
void vmem_context_switch_to(struct vmem_context_s *ctx);

/* destroy a memory context */
void vmem_context_destroy(struct vmem_context_s *ctx);

/* get all page attributes */
vmem_pageattr_t
vmem_vpage_get_attr(struct vmem_context_s *ctx,
		   uintptr_t vaddr);

/* update all page attributes, may flush tlb */
error_t
vmem_vpage_update_attr(struct vmem_context_s *ctx, uintptr_t vaddr,
		       vmem_pageattr_t attr);

/* set page attributes (or), may flush tlb */
error_t
vmem_vpage_set_attr(struct vmem_context_s *ctx, uintptr_t vaddr,
		    vmem_pageattr_t attr);

/* clear page attributes (and not), may flush tlb */
error_t
vmem_vpage_clr_attr(struct vmem_context_s *ctx, uintptr_t vaddr,
		    vmem_pageattr_t attr);

/* get virtual page physical address */
uintptr_t vmem_vpage_get_paddr(struct vmem_context_s *ctx,
			       uintptr_t vaddr);

/* map a physical page to a virtual page, may flush tlb */
error_t vmem_vpage_map(struct vmem_context_s *ctx,
		       uintptr_t vaddr, uintptr_t paddr);

/* umap a virtual page, may flush tlb */
void vmem_vpage_umap(struct vmem_context_s *ctx,
		     uintptr_t vaddr);

/***********************************************************************
 *  physical page allocator
 */

struct vmem_ppage_desc_s;
struct vmem_page_region_s;

error_t vmem_ppage_region_init(uintptr_t paddr, uintptr_t paddr_end);
bool_t vmem_ppage_inrange(uintptr_t paddr);
void vmem_ppage_region_destroy();
error_t vmem_ppage_alloc(uintptr_t *paddr);
error_t vmem_ppage_reserve(uintptr_t paddr, uintptr_t paddr_end);
uintptr_t vmem_ppage_refnew(uintptr_t paddr);
void vmem_ppage_refdrop(uintptr_t paddr);

#endif

