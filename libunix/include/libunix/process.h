#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/context.h>
#include <hexo/scheduler.h>

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>

#include <gpct/cont_clist.h>
#include <gpct/cont_hashlist.h>

typedef uint_fast16_t unix_pid_t;

CONTAINER_TYPE(unix_ps_list, CLIST, struct unix_process_s
{
  CONTAINER_ENTRY_TYPE(CLIST)		list_entry;
  CONTAINER_ENTRY_TYPE(HASHLIST)	hash_entry;
  struct sched_context_s		ctx;
  unix_pid_t				pid;
  unix_ps_list_root_t			children;
  struct unix_process_s			*parent;
}					, NOLOCK, NOOBJ, list_entry);

CONTAINER_FUNC(static inline, unix_ps_list, CLIST, unix_plist, NOLOCK);

CONTAINER_TYPE(unix_ps_hash, HASHLIST, struct unix_process_s, NOLOCK, NOOBJ, hash_entry, 16);
CONTAINER_KEY_TYPE(unix_ps_hash, SCALAR, pid);

CONTAINER_FUNC(static inline, unix_ps_hash, HASHLIST, unix_phash, NOLOCK, pid);
CONTAINER_KEY_FUNC(static inline, unix_ps_hash, HASHLIST, unix_phash, NOLOCK, pid);

extern unix_ps_hash_root_t unix_ps_hash_g;

struct unix_process_s *unix_create_process(struct unix_process_s *parent, context_entry_t entry, uint_fast16_t stack_sz);
void unix_start_process(struct unix_process_s *ps);
