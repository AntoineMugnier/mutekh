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

#ifndef MMU_H_
#define MMU_H_

#ifndef CONFIG_HEXO_MMU
//# warning Virtual memory support is not enabled in configuration file
#else

/***********************************************************************
 *  main virtual memory interface
 */

#include "types.h"
#include "error.h"
#include "local.h"

struct mmu_context_s;
//struct mmu_page_s;//FIXME: Sert à quoi?
typedef uint8_t mmu_pageattr_t;

#define MMU_PAGE_ATTR_R 0x01
#define MMU_PAGE_ATTR_W 0x02
#define MMU_PAGE_ATTR_X 0x04
#define MMU_PAGE_ATTR_NOCACHE 0x08
#define MMU_PAGE_ATTR_USERLEVEL 0x10
#define MMU_PAGE_ATTR_DIRTY 0x20
#define MMU_PAGE_ATTR_ACCESSED 0x40
#define MMU_PAGE_ATTR_PRESENT 0x80

#define MMU_PAGE_ATTR_RW 0x03
#define MMU_PAGE_ATTR_RWX 0x07
#define MMU_PAGE_ATTR_RX 0x05

extern CPU_LOCAL struct mmu_context_s *mmu_context_cur;

/* get current memory context */
static inline struct mmu_context_s * mmu_context_get(void)
{
  return CPU_LOCAL_GET(mmu_context_cur);
}

#define MMU_VPAGE_ALLOCATOR(n) void    *(n)(size_t)
#define MMU_PPAGE_ALLOCATOR(n) error_t *(n)(uintptr_t *paddr)

typedef MMU_VPAGE_ALLOCATOR(mmu_vpage_allocator_t);
typedef MMU_PPAGE_ALLOCATOR(mmu_ppage_allocator_t);

/* initialize virtual memory strucutres */
void mmu_global_init(mmu_vpage_allocator_t *va, mmu_ppage_allocator_t *pa);

/* switch to virtual memory mode by enabling mmu */
void mmu_cpu_init(void);

/* get kernel context */
struct mmu_context_s * mmu_get_kernel_context();

/* create a memory context and initialize context object */
error_t mmu_context_init(struct mmu_context_s *ctx);

/* switch to a memory context */
void mmu_context_switch_to(struct mmu_context_s *ctx);

/* destroy a memory context */
void mmu_context_destroy(void);


/* update all page attributes */
error_t mmu_vpage_set(uintptr_t vaddr, uintptr_t paddr, mmu_pageattr_t attr);

/* set (logical or) and clear (logical nand) page attributes, virtual
   page must exist */
void mmu_vpage_mask_attr(uintptr_t vaddr, mmu_pageattr_t setmask, mmu_pageattr_t clrmask);

/* get all page attributes, return 0 if page does not exist */
mmu_pageattr_t mmu_vpage_get_attr(uintptr_t vaddr);

/* get virtual page physical address, virtual page must exist */
uintptr_t mmu_vpage_get_paddr(uintptr_t vaddr);

bool_t mmu_is_user_vaddr(uintptr_t vaddr);

#if defined(CONFIG_HEXO_CPU_MMU)
# include <cpu/hexo/mmu.h>
#elif defined(CONFIG_HEXO_ARCH_MMU)
# include <arch/hexo/mmu.h>
#else
# error
#endif

#endif

#endif

