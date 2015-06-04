/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Dimitri Refauvelet <dimitri.refauvelet@lip6.fr> (c) 2009
*/

#include <mutek/mem_alloc.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <assert.h>

extern inline void *
mem_alloc_align(size_t size, size_t align, enum mem_scope_e scope);

extern inline void *
mem_alloc_cpu(size_t size, enum mem_scope_e scope, cpu_id_t cpu_id);

#ifdef CONFIG_MUTEK_DEFAULT_MEMALLOC_INIT

void mutek_mem_alloc_init()
{
  __unused__ extern __ldscript_symbol_t __bss_end;
  uintptr_t start, end;

# if defined(CONFIG_LOAD_ROM) && CONFIG_STARTUP_HEAP_SIZE == 0
  /* Use the end of the LOAD_ROM_RW region when the startup heap size
     is 0.  The actual heap will be located after data and bss
     sections. This is only used when CONFIG_LOAD_ROM is defined. */
  start = ALIGN_VALUE_UP((uintptr_t)&__bss_end, CONFIG_MUTEK_MEMALLOC_ALIGN);
  end = CONFIG_LOAD_ROM_RW_ADDR + CONFIG_LOAD_ROM_RW_SIZE;
# elif defined(CONFIG_LOAD_ROM) && CONFIG_LOAD_ROM_RW_ADDR + CONFIG_LOAD_ROM_RW_SIZE == CONFIG_STARTUP_HEAP_ADDR
  /* Merge the end of the LOAD_ROM_RW region with the startup heap
     when they are contiguous. This is only used when CONFIG_LOAD_ROM
     is defined. */
  start = ALIGN_VALUE_UP((uintptr_t)&__bss_end, CONFIG_MUTEK_MEMALLOC_ALIGN);
  end = CONFIG_STARTUP_HEAP_ADDR + CONFIG_STARTUP_HEAP_SIZE;
# else
#  define LOAD_ROM_RW_EXTEND
  /* Rely on CONFIG_STARTUP_HEAP_ADDR and CONFIG_STARTUP_HEAP_SIZE only */
  start = CONFIG_STARTUP_HEAP_ADDR;
  end = CONFIG_STARTUP_HEAP_ADDR + CONFIG_STARTUP_HEAP_SIZE;
# endif

  assert(start != end);
  default_region = memory_allocator_init(NULL, (void*)start, (void*)end);

#if defined(CONFIG_LOAD_ROM) && defined(LOAD_ROM_RW_EXTEND)
  /* Add the end of the LOAD_ROM_RW region */
  start = ALIGN_VALUE_UP((uintptr_t)&__bss_end, CONFIG_MUTEK_MEMALLOC_ALIGN);
  uintptr_t size = ALIGN_VALUE_LOW(CONFIG_LOAD_ROM_RW_ADDR + CONFIG_LOAD_ROM_RW_SIZE - start, CONFIG_MUTEK_MEMALLOC_ALIGN);
  if (size >= 64)
    memory_allocator_extend(default_region, start, size);
#endif

  /* setup a guard value for the startup stack */
  cpu_mem_write_32(CONFIG_STARTUP_STACK_ADDR, 0x42ab8d64);
}

void mutek_startup_stack_reclaim()
{
  /* check the guard value of the startup stack */
  assert(cpu_mem_read_32(CONFIG_STARTUP_STACK_ADDR) == 0x42ab8d64 &&
         "startup stack overflowed");

  /* merge the startup stack space in the allocator default region */
  memory_allocator_extend(default_region, (void*)CONFIG_STARTUP_STACK_ADDR, CONFIG_STARTUP_STACK_SIZE);
}

#endif

