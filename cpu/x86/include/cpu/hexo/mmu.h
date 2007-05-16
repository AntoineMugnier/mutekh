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

#ifndef CPU_X86_MMU_HH_
#define CPU_X86_MMU_HH_

#include <hexo/types.h>

/* page table in directory */

struct cpu_x86_pagetable_entry_s
{
    uint32_t
		present:1,
		writable:1,
		userlevel:1,
		write_through:1,
		nocache:1,
		accessed:1,
		zero:2,
		global:1,
		unused:3,
		address:20;
};

/* 4Mb page in directory */

struct cpu_x86_page4m_entry_s
{
    uint32_t
		present:1,
		writable:1,
		userlevel:1,
		write_through:1,
		nocache:1,
		accessed:1,
		dirty:1,
		pagsize4mb:1,
		global:1,
		unused:3,
		patindex:1,
		reserved:9,
		address:10;
};

union cpu_x86_pagedir_entry_s
{
  struct cpu_x86_pagetable_entry_s pte;
  struct cpu_x86_page4m_entry_s p4m;
};

typedef union cpu_x86_pagedir_entry_s cpu_x86_pagedir_t[1024] __attribute__ ((aligned (4096)));

/* 4k page in page table */

struct cpu_x86_page4k_entry_s
{
    uint32_t
		present:1,
		writable:1,
		userlevel:1,
		write_through:1,
		nocache:1,
		accessed:1,
		dirty:1,
		pat_index:1,
		global:1,
		unused:3,
		address:20;
};

typedef struct cpu_x86_page4k_entry_s cpu_x86_pagetable_t[1024] __attribute__ ((aligned (4096)));

#endif

