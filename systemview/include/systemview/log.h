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

#ifndef SYSTEMVIEW_ID_H_
#define SYSTEMVIEW_ID_H_

#include <systemview/event.h>
#include <hexo/decls.h>

void systemview_log(uint32_t id);
void systemview_log_1(uint32_t id, uint32_t a0);
void systemview_log_2(uint32_t id, uint32_t a0, uint32_t a1);
void systemview_log_3(uint32_t id, uint32_t a0, uint32_t a1, uint32_t a2);
void systemview_log_4(uint32_t id, uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3);

uint64_t systemview_timestamp_get(void);

#define SYSTEMVIEW_ID_SHRINK(id) ((((uintptr_t)(id)) - (uintptr_t)CONFIG_LOAD_ROM_RW_ADDR) / CONFIG_MUTEK_MEMALLOC_ALIGN)

ALWAYS_INLINE void systemview_log_isr_enter(uint32_t isr)
{
  systemview_log_1(SYSTEMVIEW_EVENT_ISR_ENTER, isr);
}

ALWAYS_INLINE void systemview_log_isr_exit(void)
{
  systemview_log(SYSTEMVIEW_EVENT_ISR_EXIT);
}

ALWAYS_INLINE void systemview_log_isr_to_scheduler(void)
{
  systemview_log(SYSTEMVIEW_EVENT_ISR_TO_SCHEDULER);
}

ALWAYS_INLINE void systemview_log_task_start_exec(uint32_t id)
{
  systemview_log_1(SYSTEMVIEW_EVENT_TASK_START_EXEC, SYSTEMVIEW_ID_SHRINK(id));
}

ALWAYS_INLINE void systemview_log_task_stop_exec(void)
{
  systemview_log(SYSTEMVIEW_EVENT_TASK_STOP_EXEC);
}

ALWAYS_INLINE void systemview_log_task_start_ready(uint32_t id)
{
  systemview_log_1(SYSTEMVIEW_EVENT_TASK_START_READY, SYSTEMVIEW_ID_SHRINK(id));
}

ALWAYS_INLINE void systemview_log_task_stop_ready(uint32_t id, uint32_t cause)
{
  systemview_log_2(SYSTEMVIEW_EVENT_TASK_STOP_READY, SYSTEMVIEW_ID_SHRINK(id), cause);
}

ALWAYS_INLINE void systemview_log_task_create(uint32_t id)
{
  systemview_log_1(SYSTEMVIEW_EVENT_TASK_CREATE, SYSTEMVIEW_ID_SHRINK(id));
}

void systemview_log_sysdesc(const char *desc);

ALWAYS_INLINE void systemview_log_user_start(uint32_t id)
{
  systemview_log_1(SYSTEMVIEW_EVENT_USER_START, id);
}

ALWAYS_INLINE void systemview_log_user_stop(uint32_t id)
{
  systemview_log_1(SYSTEMVIEW_EVENT_USER_STOP, id);
}

ALWAYS_INLINE void systemview_log_idle(void)
{
  systemview_log(SYSTEMVIEW_EVENT_IDLE);
}

ALWAYS_INLINE void systemview_log_timer_enter(uint32_t id)
{
  systemview_log_1(SYSTEMVIEW_EVENT_TIMER_ENTER, SYSTEMVIEW_ID_SHRINK(id));
}

ALWAYS_INLINE void systemview_log_timer_exit(void)
{
  systemview_log(SYSTEMVIEW_EVENT_TIMER_EXIT);
}

void systemview_log_module_desc(uint32_t id, uint32_t first_event, const char *desc);

ALWAYS_INLINE
void systemview_log_module_count(uint32_t count)
{
  systemview_log_1(SYSTEMVIEW_EVENT_NUMMODULES, count);
}

void systemview_log_task_info(uint32_t id, const char *name, uint32_t prio);

void systemview_log_name_resource(uint32_t id, const char *name);

ALWAYS_INLINE
void systemview_log_task_stack(uint32_t id, uint32_t stack, uint32_t stack_size)
{
  systemview_log_4(SYSTEMVIEW_EVENT_STACK_INFO, SYSTEMVIEW_ID_SHRINK(id), stack, stack_size, 0);
}

ALWAYS_INLINE
void systemview_log_start(void)
{
  systemview_log(SYSTEMVIEW_EVENT_TRACE_START);
}

#endif
