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

static struct mem_alloc_region_s mem_ram;

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

static const size_t	hdr_size = ALIGN_VALUE_UP(sizeof (struct mem_alloc_header_s),
						  CONFIG_HEXO_MEMALLOC_ALIGN);

void * mem_alloc(size_t size, uint_fast8_t scope)
{
  struct mem_alloc_header_s	*hdr;

  size = hdr_size + ALIGN_VALUE_UP(size, CONFIG_HEXO_MEMALLOC_ALIGN);

  hdr = mem_alloc_region_pop(&mem_ram, size);

  return (uint8_t*)hdr + hdr_size;
}

void mem_free(void *ptr)
{
  struct mem_alloc_header_s	*hdr = (void*)((uint8_t*)ptr - hdr_size);

  mem_alloc_region_push(hdr);
}

void mem_init(void)
{
  void	*mem_end = mem_ibmpc_memsize_probe(&__system_heap_start);
  void	*mem_start = (uint8_t*)&__system_heap_start;

  mem_end = ALIGN_ADDRESS_LOW(mem_end, CONFIG_HEXO_MEMALLOC_ALIGN);
  mem_start = ALIGN_ADDRESS_UP(mem_start, CONFIG_HEXO_MEMALLOC_ALIGN);

  mem_alloc_region_init(&mem_ram, mem_start, mem_end);
}

error_t mem_stats(uint_fast8_t scope, size_t *alloc_blocks,
		  size_t *free_size, size_t *free_blocks)
{
#ifdef CONFIG_HEXO_MEMALLOC_STATS
  struct mem_alloc_region_s *region = &mem_ram;

  if (alloc_blocks)
    *alloc_blocks = region->alloc_blocks;

  if (free_size)
    *free_size = region->free_size;

  if (free_blocks)
    *free_blocks = region->free_blocks;

  return 0;
#else
  return -ENOTSUP;
#endif
}

