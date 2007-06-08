#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/context.h>
#include <stdlib.h>
#include <stdio.h>
#include <libunix/libunix.h>
#include <libunix/process.h>

struct unix_process_s *ps_init;

//extern unix_ps_hash_root_t unix_ps_hash_g;

void libunix_init(void)
{
  unix_phash_init(&unix_ps_hash_g);

  ps_init = unix_create_process(NULL);
  unix_start_process(ps_init);
}

