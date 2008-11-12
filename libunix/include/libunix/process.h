
#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/context.h>
#include <hexo/scheduler.h>
#include <hexo/vmem.h>

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
  struct vmem_context_s			vmem;
  unix_pid_t				pid;
  unix_ps_list_root_t			children;
  struct unix_process_s			*parent;

  uintptr_t				stack_vaddr_start;
  uintptr_t				stack_vaddr_end;
  uintptr_t				heap_vaddr_start;
  uintptr_t				heap_vaddr_end;
  uintptr_t				data_vaddr_start;
  uintptr_t				data_vaddr_end;
}, list_entry);

CONTAINER_FUNC(unix_ps_list, CLIST, static inline, unix_plist);

CONTAINER_TYPE(unix_ps_hash, HASHLIST, struct unix_process_s, hash_entry, 16);
CONTAINER_KEY_TYPE(unix_ps_hash, SCALAR, pid);

CONTAINER_FUNC(unix_ps_hash, HASHLIST, static inline, unix_phash, pid);
CONTAINER_KEY_FUNC(unix_ps_hash, HASHLIST, static inline, unix_phash, pid);

extern unix_ps_hash_root_t unix_ps_hash_g;

struct unix_process_s *unix_create_process(struct unix_process_s *parent);

void unix_start_process(struct unix_process_s *ps);

