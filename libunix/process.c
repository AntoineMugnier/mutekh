#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/context.h>
#include <hexo/alloc.h>
#include <hexo/scheduler.h>
#include <hexo/interrupt.h>

#include <libunix/process.h>
#include <libunix/syscall.h>

struct unix_process_s *unix_create_process(context_entry_t entry, uint_fast16_t stack_sz)
{
  struct unix_process_s *ps = mem_alloc(sizeof(*ps), MEM_SCOPE_SYS);

  context_init(&ps->ctx.context, stack_sz, entry, NULL);
  cpu_syscall_sethandler_ctx(&ps->ctx.context, unix_syscall_handler);
  sched_context_init(&ps->ctx);

  return ps;
}

void unix_start_process(struct unix_process_s *ps)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_context_start(&ps->ctx);
  CPU_INTERRUPT_RESTORESTATE;
}
