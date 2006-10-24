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
#include <hexo/lock.h>
#include <hexo/endian.h>

struct mem_alloc_region_s mem_region_ibmpc_ram;

static void *
mem_ibmpc_memsize_probe(void *start)
{
  volatile uint8_t	*x = ALIGN_ADDRESS_UP(start, 4096);
  size_t		step = 4096;

  while (1) {
    x += step;
    *x = 0x5a;
    *x = ~*x;

    if (*x == 0xa5)
      continue;

    x -= step;

    if (step == 1)
      break;

    step /= 2;
  }

  return (void*)x;
}

void mem_init(void)
{
  void	*mem_end = mem_ibmpc_memsize_probe(&__system_heap_start);
  void	*mem_start = (uint8_t*)&__system_heap_start;

  mem_end = ALIGN_ADDRESS_LOW(mem_end, CONFIG_HEXO_MEMALLOC_ALIGN);
  mem_start = ALIGN_ADDRESS_UP(mem_start, CONFIG_HEXO_MEMALLOC_ALIGN);

  mem_alloc_region_init(&mem_region_ibmpc_ram, mem_start, mem_end);
}

