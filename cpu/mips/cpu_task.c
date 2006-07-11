
#include <hexo/error.h>
#include <hexo/task.h>

error_t
cpu_task_bootstrap(struct task_s *task)
{
  /* set task local storage base pointer */
  CPU_LOCAL_SET(__cpu_task_data_base, task->tls);
  TASK_LOCAL_SET(__task_data_base, task->tls);

  return 0;
}



/* fake task entry point, pop entry funciton param and entry function
   address from stack and perform jump to real entry function.  We
   need to do this to pass an argument to the task entry function. */
void __mips_task_entry(void);

asm(
    ".set push			\n"
    ".set noreorder		\n"
    ".set noat			\n"
    "__mips_task_entry:		\n"
    "	lw	$4,	0($sp)	\n" /* entry function param */
    "	lw	$1,	4($sp)	\n" /* entry function address */
    "	jr	$1		\n"
    "	addiu	$sp,	2*4	\n"
    ".set pop			\n"
    );



/* task init function */

error_t
cpu_task_init(struct task_s *task, task_entry_t *entry, void *param)
{
  TASK_LOCAL_FOREIGN_SET(task->tls, __task_data_base, task->tls);

  /* push entry function address and param arg */
  *--task->stack_ptr = (uintptr_t)entry;
  *--task->stack_ptr = (uintptr_t)param;

  /* fake entry point */
  *--task->stack_ptr = (uintptr_t)&__mips_task_entry;

  /* status register */
  *--task->stack_ptr = 0x0000ff00;

  /* task local storage address */
  *--task->stack_ptr = (uintptr_t)task->tls;

  return 0;
}



void
cpu_task_destroy(struct task_s *task)
{
#if 0
  __reg_t		*stack = (__reg_t*)task->stack_ptr;
#endif
}

