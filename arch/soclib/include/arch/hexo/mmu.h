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

#if !defined(MMU_H_) || defined(CPU_SOCLIB_MMU_H_)
#error This file can not be included directly
#else

#define CPU_SOCLIB_MMU_H_
#include <hexo/endian.h>
#include <hexo/types.h>
#include <hexo/cpu.h>

//TODO: retirer les macro inutiles

#define MMU_USERLIMIT_PDE (CONFIG_HEXO_MMU_USER_START >> 22)//TODO: inutile?
#define MMU_INITIAL_PDE (CONFIG_HEXO_MMU_INITIAL >> 22)//TODO: inutile?

#define MMU_KERNEL_START_PDE 512
#define MMU_KERNEL_END_PDE 1020

#define MMU_KERNEL_START_ADDR (MMU_KERNEL_START_PDE * 0x400000)
#define MMU_KERNEL_END_ADDR (MMU_KERNEL_END_PDE * 0x400000)

#define MMU_USER_START_PDE 0
#define MMU_USER_END_PDE 511 

#define MMU_USER_START_ADDR (MMU_USER_START_PDE * 0x400000)
#define MMU_USER_END_ADDR 0x7FFFFFFF

#define MMU_MIRROR_PDE 1023
#define MMU_MIRROR_ADDR 0xffc00000

#define MMU_TTY_ADDR 0xff000000
#define MMU_ICU_ADDR 0xff400000
#define MMU_TIMER_ADDR 0xff800000


#define MMU_SOCLIB_ALIGN __attribute__ ((aligned (CONFIG_HEXO_MMU_PAGESIZE)))//

/* page table in directory */

struct cpu_vcache_pagetable_entry_s
{
  ENDIAN_BITFIELD(	uint32_t  entry_type:2,//toujours egal à 01
			uint32_t  not_used:6,
			uint32_t  address:24
			);
};

/* 4Mb page in directory */

struct cpu_vcache_page4m_entry_s
{
  ENDIAN_BITFIELD(	uint32_t  entry_type:2,
			uint32_t  not_used:10,
			uint32_t  address:14,
			uint32_t  non_cacheable:1,
			uint32_t  writable:1,
			uint32_t  executable:1,
			uint32_t  user:1,
			uint32_t  global:1,
			uint32_t  dirty:1
			);
};

/* 4k page in page table */

struct cpu_vcache_page4k_entry_s
{
  ENDIAN_BITFIELD(	uint32_t  entry_type:2,
			uint32_t	 not_used:4,
			uint32_t  address:20,
			uint32_t  non_cacheable:1,
			uint32_t  writable:1,
			uint32_t  executable:1,
			uint32_t  user:1,
			uint32_t  global:1,
			uint32_t  dirty:1
			);
};


union cpu_vcache_page_entry_s
{
  struct cpu_vcache_pagetable_entry_s pte;
  struct cpu_vcache_page4m_entry_s p4m;
  struct cpu_vcache_page4k_entry_s p4k;
  uint32_t val;
};

typedef union cpu_vcache_page_entry_s cpu_vcache_page_entry_t;

struct mmu_context_s
{
  cpu_vcache_page_entry_t	*pagedir; /* page directory */
  cpu_vcache_page_entry_t	*mirror; /* page table mirroring page directory */
  uintptr_t		pagedir_paddr; /* page directory physical address */
  uint_fast16_t		k_count; /* kernel entries count */
};


static inline void
mmu_vcache_set_pagedir(uint32_t paddr)
{
  cpu_mips_mtc2(paddr,0x0,0);
}

static inline union cpu_vcache_page_entry_s *
mmu_vcache_get_pagedir(void)
{
  uint_fast32_t *pd;
  *pd = cpu_mips_mfc2(0,0);
  asm volatile(	"and %0,$0xfffffc00, %0	\n"
		: "=r" (pd)
		);
  
  return pd;
}

static inline void//TODO: divise en deux fonction, une pour chaque cache
mmu_vcache_invalidate_page(uintptr_t vaddr)
{
  cpu_mips_mtc2(vaddr,4,0);
  cpu_mips_mtc2(vaddr,5,0);
}

#endif

