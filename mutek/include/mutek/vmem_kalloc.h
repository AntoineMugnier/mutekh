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

#ifndef VMEM_KALLOC_H_
#define VMEM_KALLOC_H_

#include <hexo/types.h>
#include <hexo/mmu.h>
#include <mutek/vmem_palloc.h>

# ifndef CONFIG_VMEM_KERNEL_ALLOC
#  warning CONFIG_VMEM_KERNEL_ALLOC is disabled !!!
# else

/*
 * Kernel virtual space management
 */

/* map a physical address range somewhere in kernel space and return
   its address or 0 on error, may flush tlb  */
uintptr_t vmem_vpage_io_map(uintptr_t paddr, size_t byte_size);

/* allocate virtual page memory in kernel page space, may flush tlb */
void * vmem_vpage_kalloc(size_t count);

/* free a kernel virtual page */
void vmem_vpage_kfree(void *vaddr, size_t count);

# endif

#endif

