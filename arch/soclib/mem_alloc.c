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


#include <hexo/alloc.h>
#include <hexo/segment.h>

#include "dsx_addresses.h"

struct mem_alloc_region_s mem_region_system;

#if defined(CONFIG_SMP)
struct mem_alloc_region_s mem_region_cpu;
#endif

#if defined(CONFIG_CLUSTER)
struct mem_alloc_region_s mem_region_cluster;
#endif

#if defined(CONFIG_SMP) || defined(CONFIG_CLUSTER)
struct mem_alloc_region_s *mem_region_default;
#endif

void mem_init(void)
{
#if defined(CONFIG_SMP)
  mem_alloc_region_init(&mem_region_cpu,
			(uint8_t*)DSX_SEGMENT_HEAP_C_ADDR,
			(uint8_t*)DSX_SEGMENT_HEAP_C_ADDR + DSX_SEGMENT_HEAP_C_SIZE
			);
#endif

  mem_alloc_region_init(&mem_region_system,
			(uint8_t*)DSX_SEGMENT_HEAP_U_ADDR,
			(uint8_t*)DSX_SEGMENT_HEAP_U_ADDR + DSX_SEGMENT_HEAP_U_SIZE
			);

#if defined(CONFIG_SMP) || defined(CONFIG_CLUSTER)
  mem_region_default = &mem_region_system;
#endif
}

