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

%config CONFIG_HEXO_VMEM_INITIAL
desc Size of initial memory reserved for kernel use
default 0x00800000
%config end

*/


/***********************************************************************
 *  main virtual memory interface
 */

#include "types.h"
#include "error.h"
#include "local.h"

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

#define VMEM_PAGE_ATTR_RW 0x03
#define VMEM_PAGE_ATTR_RWX 0x07

extern CPU_LOCAL struct vmem_context_s *vmem_context_cur;

/* get current memory context */
static inline struct vmem_context_s * vmem_context_get(void)
{
  return CPU_LOCAL_GET(vmem_context_cur);
}

/* initialize virtual memory strucutres */
void vmem_global_init(void);

/* switch to virtual memory mode by enabling mmu */
void vmem_cpu_init(void);

/* create a memory context and initialize context object */
error_t vmem_context_init(struct vmem_context_s *ctx);

/* switch to a memory context */
void vmem_context_switch_to(struct vmem_context_s *ctx);

/* destroy a memory context */
void vmem_context_destroy(void);


/* update all page attributes */
error_t vmem_vpage_set(uintptr_t vaddr, uintptr_t paddr, vmem_pageattr_t attr);

/* set (logical or) and clear (logical nand) page attributes, virtual
   page must exist */
void vmem_vpage_mask_attr(uintptr_t vaddr, vmem_pageattr_t setmask, vmem_pageattr_t clrmask);

/* get all page attributes, return 0 if page does not exist */
vmem_pageattr_t vmem_vpage_get_attr(uintptr_t vaddr);

/* get virtual page physical address, virtual page must exist */
uintptr_t vmem_vpage_get_paddr(uintptr_t vaddr);




/* map a physical address range somewhere in kernel space and return
   its address or 0 on error, may flush tlb  */
uintptr_t vmem_vpage_io_map(uintptr_t paddr, size_t size);

/* allocate virtual page memory in kernel page space, may flush tlb */
void * vmem_vpage_kalloc();

/* free a kernel virtual page */
void vmem_vpage_kfree(void *vaddr);



/***********************************************************************
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

#include "cpu/hexo/vmem.h"

#endif

