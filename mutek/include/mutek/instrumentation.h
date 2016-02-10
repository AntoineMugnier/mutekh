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

#ifndef MUTEK_INSTRUMENTATION_H_
#define MUTEK_INSTRUMENTATION_H_

#include <hexo/types.h>

/**
   Backends must define:
   - INSTRUMENTATION_CHECK_SP
   - INSTRUMENTATION_CHECK_MEM_INIT
   - INSTRUMENTATION_CHECK_MEM_REGIONS
   - INSTRUMENTATION_CHECK_IRQ_LOCK
 */

#define INSTRUMENTATION_CHECK_ALL (INSTRUMENTATION_CHECK_SP                 \
                         | INSTRUMENTATION_CHECK_MEM_INIT         \
                         | INSTRUMENTATION_CHECK_MEM_REGIONS      \
                         | INSTRUMENTATION_CHECK_IRQ_LOCK)

ALWAYS_INLINE void instrumentation_pointer_name(uintptr_t ptr, const char *name);

ALWAYS_INLINE void instrumentation_check_disable(uint32_t to_disable);
ALWAYS_INLINE void instrumentation_check_enable(uint32_t to_enable);

/**
   Backends must define region states:
   - INSTRUMENTATION_MEMORY_REGION_FREE
   - INSTRUMENTATION_MEMORY_REGION_ALLOC
   - INSTRUMENTATION_MEMORY_REGION_GLOBAL
 */
ALWAYS_INLINE void instrumentation_memory_region_state_change(uintptr_t pointer, size_t size, uint32_t state);
ALWAYS_INLINE void instrumentation_memory_region_mark_initialized(uintptr_t pointer, size_t size);

ALWAYS_INLINE void instrumentation_lock_create(uintptr_t lock_addr);
ALWAYS_INLINE void instrumentation_lock_destroy(uintptr_t lock_addr);

/**
   Backends must define special context IDs:
   - INSTRUMENTATION_CONTEXT_UNKNOWN
   - INSTRUMENTATION_CONTEXT_CURRENT
 */
ALWAYS_INLINE void instrumentation_context_create(uint32_t id, uintptr_t stack_base, size_t stack_size);
ALWAYS_INLINE void instrumentation_context_bootstrap(uint32_t id, uintptr_t stack_base, size_t stack_size);
ALWAYS_INLINE void instrumentation_context_invalidate(uint32_t id);
ALWAYS_INLINE void instrumentation_context_destroy(uint32_t id);
ALWAYS_INLINE void instrumentation_context_rename(uint32_t new_id, uint32_t old_id);
ALWAYS_INLINE void instrumentation_context_running(uint32_t id);
ALWAYS_INLINE void instrumentation_context_waiting(uint32_t id);
ALWAYS_INLINE void instrumentation_context_stopped(uint32_t id);

ALWAYS_INLINE void instrumentation_scheduler_idle(void);

ALWAYS_INLINE void instrumentation_sp_check_bypass_declare(uintptr_t from, uintptr_t to, bool_t append);

ALWAYS_INLINE void instrumentation_irq_begin(uint32_t id);
ALWAYS_INLINE void instrumentation_irq_end(uint32_t id);


ALWAYS_INLINE void instrumentation_device_op_begin(uintptr_t dev, uintptr_t driver, uint32_t op_id);
ALWAYS_INLINE void instrumentation_device_op_end(void);

ALWAYS_INLINE void instrumentation_kroutine_init(uintptr_t kr, uint32_t policy);
ALWAYS_INLINE void instrumentation_kroutine_exec(uintptr_t kr);
ALWAYS_INLINE void instrumentation_kroutine_begin(uintptr_t kr);
ALWAYS_INLINE void instrumentation_kroutine_end(uintptr_t kr);

ALWAYS_INLINE void instrumentation_bytecode_vm_op(uintptr_t vm, uint16_t op);

#if defined(CONFIG_MUTEK_INSTRUMENTATION)
# include <mutek/instrumentation_backend.h>
#else

ALWAYS_INLINE void instrumentation_pointer_name(uintptr_t ptr, const char *name)
{
}

#define INSTRUMENTATION_CHECK_SP 0
#define INSTRUMENTATION_CHECK_MEM_INIT 0
#define INSTRUMENTATION_CHECK_MEM_REGIONS 0
#define INSTRUMENTATION_CHECK_IRQ_LOCK 0

ALWAYS_INLINE void instrumentation_check_disable(uint32_t to_disable)
{
}

ALWAYS_INLINE void instrumentation_check_enable(uint32_t to_enable)
{
}

#define INSTRUMENTATION_MEMORY_REGION_FREE 0
#define INSTRUMENTATION_MEMORY_REGION_ALLOC 0
#define INSTRUMENTATION_MEMORY_REGION_GLOBAL 0
ALWAYS_INLINE void instrumentation_memory_region_state_change(uintptr_t pointer, size_t size, uint32_t state)
{
}

ALWAYS_INLINE void instrumentation_memory_region_mark_initialized(uintptr_t pointer, size_t size)
{
}


ALWAYS_INLINE void instrumentation_lock_create(uintptr_t lock_addr)
{
}

ALWAYS_INLINE void instrumentation_lock_destroy(uintptr_t lock_addr)
{
}

#define INSTRUMENTATION_CONTEXT_UNKNOWN 0
#define INSTRUMENTATION_CONTEXT_CURRENT 0
ALWAYS_INLINE void instrumentation_context_create(uint32_t id, uintptr_t stack_base, size_t stack_size)
{
}

ALWAYS_INLINE void instrumentation_context_bootstrap(uint32_t id, uintptr_t stack_base, size_t stack_size)
{
}

ALWAYS_INLINE void instrumentation_context_invalidate(uint32_t id)
{
}

ALWAYS_INLINE void instrumentation_context_destroy(uint32_t id)
{
}

ALWAYS_INLINE void instrumentation_context_rename(uint32_t new_id, uint32_t old_id)
{
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

ALWAYS_INLINE void instrumentation_scheduler_idle(void)
{
}

ALWAYS_INLINE void instrumentation_sp_check_bypass_declare(uintptr_t from, uintptr_t to, bool_t append)
{
}

ALWAYS_INLINE void instrumentation_irq_begin(uint32_t id)
{
}

ALWAYS_INLINE void instrumentation_irq_end(uint32_t id)
{
}

ALWAYS_INLINE void instrumentation_device_op_begin(uintptr_t dev, uintptr_t driver, uint32_t op_id)
{
}

ALWAYS_INLINE void instrumentation_device_op_end(void)
{
}

ALWAYS_INLINE void instrumentation_kroutine_init(uintptr_t kr, uint32_t policy)
{
}

ALWAYS_INLINE void instrumentation_kroutine_exec(uintptr_t kr)
{
}

ALWAYS_INLINE void instrumentation_kroutine_begin(uintptr_t kr)
{
}

ALWAYS_INLINE void instrumentation_kroutine_end(uintptr_t kr)
{
}

ALWAYS_INLINE void instrumentation_bytecode_vm_op(uintptr_t vm, uint16_t op)
{
}

#endif

#endif
