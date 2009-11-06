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

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#include <mem_alloc.h>
#include <hexo/segment.h>
#include <hexo/lock.h>
#include <hexo/endian.h>

#include <arch/hexo/emu_syscalls.h>

void mem_init(void)
{
  void	*mem_start;
  void	*mem_end;

  mem_start = (void*)emu_do_syscall(EMU_SYSCALL_MMAP, 6, NULL, 
				    CONFIG_ARCH_EMU_MEMORY,
				    EMU_PROT_READ | EMU_PROT_WRITE | EMU_PROT_EXEC,
				    EMU_MAP_PRIVATE | EMU_MAP_ANONYMOUS, 0, 0);

  if (mem_start == EMU_MAP_FAILED)
    emu_do_syscall(EMU_SYSCALL_EXIT, 1);

  mem_end = (uint8_t*)mem_start + CONFIG_ARCH_EMU_MEMORY;

  mem_end = ALIGN_ADDRESS_LOW(mem_end, CONFIG_MUTEK_MEMALLOC_ALIGN);
  mem_start = ALIGN_ADDRESS_UP(mem_start, CONFIG_MUTEK_MEMALLOC_ALIGN);

  mem_alloc_region_init(mem_region_get_local(mem_scope_sys), mem_start, mem_end);
}

