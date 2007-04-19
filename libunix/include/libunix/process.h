#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/context.h>
#include <hexo/scheduler.h>

struct unix_process_s
{
  struct sched_context_s ctx;
  struct unix_process_s *parent;
  uint_fast8_t id;
};

struct unix_process_s *unix_create_process(context_entry_t entry, uint_fast16_t stack_sz);
void unix_start_process(struct unix_process_s *ps);
