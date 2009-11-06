
#include <stdio.h>

#include <hexo/error.h>
#include <hexo/types.h>
#include <hexo/context.h>
#include <mem_alloc.h>
#include <mutek/scheduler.h>
#include <hexo/interrupt.h>
#include <hexo/vmem.h>

#include <libunix/process.h>
#include <libunix/syscall.h>

unix_pid_t pv_get_next_pid()
{
  static unix_pid_t id = 1;

  while (!id || unix_phash_lookup(&unix_ps_hash_g, id))
    id++;

  return id;
}

unix_ps_hash_root_t unix_ps_hash_g;

CONTEXT_LOCAL struct unix_process_s *unix_process_current = NULL;

static CONTEXT_ENTRY(unix_context_entry)
{
  struct unix_process_s *ps = param;
  uintptr_t kstack = (uintptr_t)context_current()->stack_ptr;

  CONTEXT_LOCAL_SET(unix_process_current, ps);

  printk("YUHUUUUUU !!!!\n");

  vmem_context_switch_to(&ps->vmem);

  uint8_t *code = CONFIG_UNIX_START_VADDR;

  *code++ = 0xcd;
  *code++ = 0x80;
  *code++ = 0xcd;
  *code++ = 0x80;
#if 1
  *code++ = 0xeb;
  *code++ = 0xfe;
#else
  *code++ = 0x0f;
  *code++ = 0x0b;
#endif

  printk("YUHAAAAAA !!!!\n");

  sched_unlock();
  cpu_interrupt_enable();
  cpu_context_set_user(kstack, CONFIG_UNIX_STACK_VADDR - 4, CONFIG_UNIX_START_VADDR);
}

struct unix_process_s *unix_create_process(struct unix_process_s *parent)
{
    struct unix_process_s *ps;
    uintptr_t stack_page;

    if ((ps = mem_alloc(sizeof(*ps), mem_region_get_local(mem_scope_sys))) == NULL)
	goto err;

    if (ppage_alloc(&stack_page))
	goto err_ps;

#ifdef CONFIG_UNIX_DEBUG
    printk("Creating unix process ");
#endif

    /* setup scheduler context */
    context_init(&ps->ctx.context,
		 ps->stack_vaddr_start,
		 ps->stack_vaddr_start + CONFIG_UNIX_KSTACK_SIZE,
		 unix_context_entry, NULL);
    sched_context_init(&ps->ctx);

    /* setup virtual memory context */
    vmem_context_init(&ps->vmem);
    ps->ctx.context.vmem = &ps->vmem;

    /* setup syscall handler */
    cpu_syscall_sethandler_ctx(&ps->ctx.context, unix_syscall_handler);

    /* user stack */
    ps->stack_vaddr_start = CONFIG_UNIX_STACK_VADDR - CONFIG_HEXO_MMU_PAGESIZE;
    ps->stack_vaddr_end = CONFIG_UNIX_STACK_VADDR;

    vmem_vpage_set(ps->stack_vaddr_start, stack_page,
		   VMEM_PAGE_ATTR_RW | VMEM_PAGE_ATTR_USERLEVEL |
		   VMEM_PAGE_ATTR_PRESENT);

    vmem_vpage_set(CONFIG_UNIX_START_VADDR, stack_page,
		   VMEM_PAGE_ATTR_RW | VMEM_PAGE_ATTR_USERLEVEL |
		   VMEM_PAGE_ATTR_PRESENT);

    /* add in process list */
    ps->parent = parent;
    ps->pid = pv_get_next_pid();

#ifdef CONFIG_UNIX_DEBUG
    printk("(pid=%i)\n", ps->pid);
#endif

    unix_plist_init(&ps->children);
    unix_phash_push(&unix_ps_hash_g, ps);

    if (parent)
	unix_plist_push(&parent->children, ps);

    return ps;

err_ps:
    mem_free(ps);
err:
    return NULL;
}

void unix_start_process(struct unix_process_s *ps)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  sched_context_start(&ps->ctx);
  CPU_INTERRUPT_RESTORESTATE;
}

