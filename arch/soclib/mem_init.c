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


#include <mutek/mem_alloc.h>

extern __ldscript_symbol_t __system_uncached_heap_start, __system_uncached_heap_end;

void mem_init(void)
{
#ifdef CONFIG_HEXO_MMU
  uint32_t t = (uint32_t)(&__system_uncached_heap_start);
  
  mem_alloc_region_init(mem_region_get_local(mem_scope_sys),
			(void *)t,
			(void *)t+CONFIG_SOCLIB_VMEM_MALLOC_REGION_SIZE
			);
  
#else
  mem_alloc_region_init(mem_region_get_local(mem_scope_sys),
			&__system_uncached_heap_start,
			&__system_uncached_heap_end
			);
#endif
}
