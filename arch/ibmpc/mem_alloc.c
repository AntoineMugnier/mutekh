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

static lock_t mem_lock = LOCK_INITIALIZER;

void * mem_alloc(size_t size, uint_fast8_t scope)
{
  static uint8_t	*addr = (void*)&__system_heap_start;
  void			*res;

  lock_spin(&mem_lock);

  res = addr;
  addr = (uint8_t*)ALIGN_VALUE((uintptr_t)addr + size, 4);

  lock_release(&mem_lock);

  return res;
}

void mem_free(void *ptr)
{
}

