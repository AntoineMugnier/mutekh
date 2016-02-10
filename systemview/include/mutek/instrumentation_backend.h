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

#include <systemview/log.h>

#define INSTRUMENTATION_CHECK_SP 0
#define INSTRUMENTATION_CHECK_MEM_INIT 0
#define INSTRUMENTATION_CHECK_MEM_REGIONS 0
#define INSTRUMENTATION_CHECK_IRQ_LOCK 0

ALWAYS_INLINE void instrumentation_pointer_name(uintptr_t ptr, const char *name)
{
  systemview_log_name_resource(ptr, name);
}

ALWAYS_INLINE void instrumentation_check_disable(uint32_t to_disable)
{
}

ALWAYS_INLINE void instrumentation_check_enable(uint32_t to_enable)
{
}

#define INSTRUMENTATION_MEMORY_REGION_FREE 0
#define INSTRUMENTATION_MEMORY_REGION_ALLOC 1
#define INSTRUMENTATION_MEMORY_REGION_GLOBAL 2

ALWAYS_INLINE void instrumentation_memory_region_state_change(uintptr_t pointer, size_t size, uint32_t state)
{
  switch (state) {
  case INSTRUMENTATION_MEMORY_REGION_FREE:
    systemview_log_1(SYSTEMVIEW_MUTEKH_MEM_FREE, SYSTEMVIEW_ID_SHRINK(pointer));
    break;

  case INSTRUMENTATION_MEMORY_REGION_ALLOC:
    systemview_log_2(SYSTEMVIEW_MUTEKH_MEM_ALLOC, SYSTEMVIEW_ID_SHRINK(pointer), size);
    break;

  default:
    break;
  }
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
  systemview_log_task_create(id);
  systemview_log_task_stack(id, stack_base, stack_size);
}

ALWAYS_INLINE void instrumentation_context_bootstrap(uint32_t id, uintptr_t stack_base, size_t stack_size)
{
  instrumentation_context_create(id, stack_base, stack_size);
  instrumentation_context_running(id);
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
  systemview_log_task_start_exec(id);
}

ALWAYS_INLINE void instrumentation_context_waiting(uint32_t id)
{
  systemview_log_task_start_ready(id);
}

ALWAYS_INLINE void instrumentation_context_stopped(uint32_t id)
{
  systemview_log_task_stop_ready(id, 0);
}

ALWAYS_INLINE void instrumentation_scheduler_idle(void)
{
  systemview_log_idle();
}

ALWAYS_INLINE void instrumentation_sp_check_bypass_declare(uintptr_t from, uintptr_t to, bool_t append)
{
}

ALWAYS_INLINE void instrumentation_irq_begin(uint32_t id)
{
  systemview_log_isr_enter(id);
}

ALWAYS_INLINE void instrumentation_irq_end(uint32_t id)
{
  systemview_log_isr_exit();
}

ALWAYS_INLINE void instrumentation_device_op_begin(uintptr_t dev, uintptr_t driver, uint32_t op_id)
{
  systemview_log_3(SYSTEMVIEW_MUTEKH_DEVICE_OP_BEGIN,
                   SYSTEMVIEW_ID_SHRINK(dev),
                   driver, op_id);
}

ALWAYS_INLINE void instrumentation_device_op_end(void)
{
  systemview_log(SYSTEMVIEW_MUTEKH_DEVICE_OP_END);
}

ALWAYS_INLINE void instrumentation_kroutine_init(uintptr_t kr, uint32_t policy)
{
  systemview_log_2(SYSTEMVIEW_MUTEKH_KROUTINE_INIT, SYSTEMVIEW_ID_SHRINK(kr), policy);
}

ALWAYS_INLINE void instrumentation_kroutine_exec(uintptr_t kr)
{
  systemview_log_1(SYSTEMVIEW_MUTEKH_KROUTINE_EXEC, SYSTEMVIEW_ID_SHRINK(kr));
}

ALWAYS_INLINE void instrumentation_kroutine_begin(uintptr_t kr)
{
  systemview_log_1(SYSTEMVIEW_MUTEKH_KROUTINE_BEGIN, SYSTEMVIEW_ID_SHRINK(kr));
}

ALWAYS_INLINE void instrumentation_kroutine_end(uintptr_t kr)
{
  systemview_log_1(SYSTEMVIEW_MUTEKH_KROUTINE_END, SYSTEMVIEW_ID_SHRINK(kr));
}

ALWAYS_INLINE void instrumentation_bytecode_vm_op(uintptr_t vm, uint16_t op)
{
  systemview_log_2(SYSTEMVIEW_MUTEKH_BYTECODE_VM_OP, SYSTEMVIEW_ID_SHRINK(vm), op);
}

#endif
