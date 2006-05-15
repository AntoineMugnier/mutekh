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

#ifndef TASK_H_
#define TASK_H_

#include "types.h"
#include "local.h"
#include "error.h"



/** cpu specific task structure */
struct cpu_task_s;

/** task descriptor structure */
struct task_s
{
  /* cpu specific task context pointer if any */
  struct cpu_task_s	*ctask;

  /* task local storage address */
  void			*tls;

  /* stack memory address */
  __reg_t			*stack;

  /* current stack pointer value */
  __reg_t			*stack_ptr;
};


/** task entry point function prototype */
#define TASK_ENTRY(n) void (n) (void *param)
/** task entry point function type */
typedef TASK_ENTRY(task_entry_t);

/** Switch task by saving/restoring all registers from/to task stack */
static void cpu_task_switch(struct task_s *old, struct task_s *new);

/** Jump to task from _non_ task context */
static void cpu_task_jumpto(struct task_s *new);

/** set new stack pointer and jump to a new function */
static void cpu_task_set_stack(uintptr_t stack, void *jumpto);

/** associate task context and cpu current execution state */
error_t cpu_task_bootstrap(struct task_s *task);

/** Prepare task execution by setting up original stack values */
error_t cpu_task_init(struct task_s *task, task_entry_t *entry, void *param);

/** cleanup task */
void cpu_task_destroy(struct task_s *task);



#include "cpu/mutek/task.h"

/** pointer to current task object */
extern TASK_LOCAL struct task_s *task_cur;

/** init a task object using current execution context */
error_t task_bootstrap(struct task_s *task);

/** init a task object allocating a new context */
error_t task_init(struct task_s *task, size_t stack_size, task_entry_t *entry, void *param);

/** free ressource associated with a task */
void task_destroy(struct task_s *task);

/** switch to a given task */
static inline void task_switch_to(struct task_s *task)
{
  struct task_s		*cur = TASK_LOCAL_GET(task_cur);

  cpu_task_switch(cur, task);
}

/** jump to a given task without saving current context */
static inline void
__attribute__((noreturn))
task_jump_to(struct task_s *task)
{
  cpu_task_jumpto(task);
}

/** return current task object */
static inline struct task_s * task_current(void)
{
  return TASK_LOCAL_GET(task_cur);
}

#endif

