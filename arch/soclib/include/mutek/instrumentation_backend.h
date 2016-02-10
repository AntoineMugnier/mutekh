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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#ifndef MUTEK_INSTRUMENTATION_BACKEND_H_
#define MUTEK_INSTRUMENTATION_BACKEND_H_

# ifndef MUTEK_INSTRUMENTATION_H_
#  error This header may not be included directly
# endif

#include <arch/mem_checker.h>

#define INSTRUMENTATION_CHECK_SP SOCLIB_MC_CHECK_SP
#define INSTRUMENTATION_CHECK_MEM_INIT SOCLIB_MC_CHECK_INIT
#define INSTRUMENTATION_CHECK_MEM_REGIONS SOCLIB_MC_CHECK_REGIONS
#define INSTRUMENTATION_CHECK_IRQ_LOCK SOCLIB_MC_CHECK_IRQS_LOCK

ALWAYS_INLINE void instrumentation_check_disable(uint32_t to_disable)
{
  soclib_mem_check_disable(to_disable);
}

ALWAYS_INLINE void instrumentation_check_enable(uint32_t to_enable)
{
  soclib_mem_check_enable(to_enable);
}

#define INSTRUMENTATION_MEMORY_REGION_FREE SOCLIB_MC_REGION_FREE
#define INSTRUMENTATION_MEMORY_REGION_ALLOC SOCLIB_MC_REGION_ALLOC
#define INSTRUMENTATION_MEMORY_REGION_GLOBAL SOCLIB_MC_REGION_GLOBAL

ALWAYS_INLINE void instrumentation_memory_region_state_change(uintptr_t pointer, size_t size, uint32_t state)
{
  soclib_mem_check_region_status((void*)pointer, size, state);
}

ALWAYS_INLINE void instrumentation_memory_region_mark_initialized(uintptr_t pointer, size_t size)
{
  soclib_mem_mark_initialized((void*)pointer, size);
}


ALWAYS_INLINE void instrumentation_lock_create(uintptr_t lock_addr)
{
  soclib_mem_check_declare_lock((void*)lock_addr, 1);
}

ALWAYS_INLINE void instrumentation_lock_destroy(uintptr_t lock_addr)
{
  soclib_mem_check_declare_lock((void*)lock_addr, 0);
}

#define INSTRUMENTATION_CONTEXT_UNKNOWN SOCLIB_MC_CTX_ID_UNKNOWN

#define INSTRUMENTATION_CONTEXT_CURRENT SOCLIB_MC_CTX_ID_CURRENT

ALWAYS_INLINE void instrumentation_context_create(uint32_t id, uintptr_t stack_base, size_t stack_size)
{
  soclib_mem_check_create_ctx(id, (void *)stack_base, (void*)(stack_base + stack_size));
}

ALWAYS_INLINE void instrumentation_context_bootstrap(uint32_t id, uintptr_t stack_base, size_t stack_size)
{
  /* create a new memchecker context */
  cpu_mem_write_32(SOCLIB_MC_MAGIC, SOCLIB_MC_MAGIC_VAL);
  cpu_mem_write_32(SOCLIB_MC_R1, stack_base);
  cpu_mem_write_32(SOCLIB_MC_R2, stack_size);
  cpu_mem_write_32(SOCLIB_MC_CTX_CREATE, id);

  /* switch to new memchecker context */
  cpu_mem_write_32(SOCLIB_MC_CTX_SET, id);

  /* enable all memchecker checks */
  cpu_mem_write_32(SOCLIB_MC_ENABLE, SOCLIB_MC_CHECK_ALL);

  /* leave memchecker command mode */
  cpu_mem_write_32(SOCLIB_MC_MAGIC, 0);
}

ALWAYS_INLINE void instrumentation_context_invalidate(uint32_t id)
{
}

ALWAYS_INLINE void instrumentation_context_destroy(uint32_t id)
{
  soclib_mem_check_delete_ctx(id);
}

ALWAYS_INLINE void instrumentation_context_rename(uint32_t new_id, uint32_t old_id)
{
  soclib_mem_check_change_id(old_id, new_id);
}

ALWAYS_INLINE void instrumentation_context_running(uint32_t id)
{
}

ALWAYS_INLINE void instrumentation_context_waiting(uint32_t id)
{
}

ALWAYS_INLINE void instrumentation_context_stopped(uint32_t id)
{
}

ALWAYS_INLINE void instrumentation_sp_check_bypass_declare(uinptr_t from, uintptr_t to, bool_t append)
{
  soclib_mem_bypass_sp_check((void*)from, (void*)to);
}

ALWAYS_INLINE void instrumentation_irq_begin(uint32_t id)
{
}

ALWAYS_INLINE void instrumentation_irq_end(uint32_t id)
{
}

#endif
