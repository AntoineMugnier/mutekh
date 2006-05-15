
#include <mutek/error.h>
#include <mutek/task.h>
#include <mutek/local.h>
#include <mutek/segment.h>

/** pointer to current task */
TASK_LOCAL struct task_s *task_cur;

/** init a task object using current execution context */
error_t
task_bootstrap(struct task_s *task)
{
  error_t	res;

  /* allocate task local storage memory */
  if (!(task->tls = arch_taskdata_alloc()))
    return -ENOMEM;

  TASK_LOCAL_FOREIGN_SET(task->tls, task_cur, task);

  /* FIXME ? initial stack space will never be freed */
  task->stack = NULL;

  /* setup cpu specific task data */
  if ((res = cpu_task_bootstrap(task)))
    {
      arch_taskdata_free(task->tls);
      return res;
    }

  return 0;
}

/** init a task object allocating a new context */
error_t
task_init(struct task_s *task, size_t stack_size, task_entry_t *entry, void *param)
{
  error_t	res;

  /* allocate task local storage memory */
  if (!(task->tls = arch_taskdata_alloc()))
    return -ENOMEM;

  TASK_LOCAL_FOREIGN_SET(task->tls, task_cur, task);

  /* allocate task stack memory */
  if (!(task->stack = arch_taskstack_alloc(stack_size * sizeof(__reg_t))))
    {
      arch_taskdata_free(task->tls);
      return -ENOMEM;      
    }

  /* initial stack pointer address */
  task->stack_ptr = task->stack + stack_size - 1;

  /* setup cpu specific task data */
  if ((res = cpu_task_init(task, entry, param)))
    {
      arch_taskdata_free(task->tls);
      arch_taskstack_free(task->stack);
      return res;
    }

  return 0;
}

/** free ressource associated with a task */
void
task_destroy(struct task_s *task)
{
  cpu_task_destroy(task);
  arch_taskdata_free(task->tls);

  if (task->stack)
    arch_taskstack_free(task->stack);
}

