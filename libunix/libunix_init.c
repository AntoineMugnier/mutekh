#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/context.h>
#include <stdlib.h>
#include <stdio.h>
#include <libunix/libunix.h>
#include <libunix/process.h>

static CONTEXT_ENTRY(pv_unix_init)
{
      puts("starting init process\n");
  while(1)
    {
      /* ... */
      sched_context_switch();
    }
}

error_t libunix_init(void)
{
  struct unix_process_s *ps_init = unix_create_process(pv_unix_init, 4096);
  unix_start_process(ps_init);

  return 0;
}
