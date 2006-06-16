
#include <mutek/error.h>
#include <mutek/task.h>

#include <cpu/mutek/pmode.h>

error_t
cpu_task_bootstrap(struct task_s *task)
{
  cpu_x86_segsel_t	tls_seg;

  /* get a new segment descriptor for tls */
  if (!(tls_seg = cpu_x86_segment_alloc((uintptr_t)task->tls, 0xffffffff, CPU_X86_SEG_DATA_UP_RW)))
    return -ENOMEM;

  /* load current tls segment */
  cpu_x86_dataseggs_use(tls_seg, 0);

  TASK_LOCAL_SET(__task_data_base, task->tls);

  return 0;
}

error_t
cpu_task_init(struct task_s *task, task_entry_t *entry, void *param)
{
  cpu_x86_segsel_t	tls_seg;

  /* get a new segment descriptor for tls */
  if (!(tls_seg = cpu_x86_segment_alloc((uintptr_t)task->tls, 0xffffffff, CPU_X86_SEG_DATA_UP_RW)))
    return -ENOMEM;

  TASK_LOCAL_FOREIGN_SET(task->tls, __task_data_base, task->tls);

  /* push param */
  *--task->stack_ptr = (uintptr_t)param;

  /* push task entry function return pointer */
  *--task->stack_ptr = (uintptr_t)0;

  /* push execution pointer */
  *--task->stack_ptr = (uintptr_t)entry;	/* EIP */

  /* push default flags */
  *--task->stack_ptr = 0x00000046;	/* EFLAGS */

  /* room for general purpose registers default values */
  task->stack_ptr -= 8;

  /* push tls segment index */
  *--task->stack_ptr = tls_seg << 3;	/* GS */

  return 0;
}

void
cpu_task_destroy(struct task_s *task)
{
  __reg_t		*stack = (__reg_t*)task->stack_ptr;

  /* free tls segment descriptor */
  cpu_x86_segdesc_free((uint16_t)stack[0]);
}

