
#include <hexo/error.h>
#include <hexo/context.h>

#include <cpu/hexo/pmode.h>

error_t
cpu_context_bootstrap(struct context_s *context)
{
  cpu_x86_segsel_t	tls_seg;

  /* get a new segment descriptor for tls */
  if (!(tls_seg = cpu_x86_segment_alloc((uintptr_t)context->tls, 0xffffffff, CPU_X86_SEG_DATA_UP_RW)))
    return -ENOMEM;

  /* load current tls segment */
  cpu_x86_dataseggs_use(tls_seg, 0);

  CONTEXT_LOCAL_SET(__context_data_base, context->tls);

  return 0;
}

error_t
cpu_context_init(struct context_s *context, context_entry_t *entry, void *param)
{
  cpu_x86_segsel_t	tls_seg;

  /* get a new segment descriptor for tls */
  if (!(tls_seg = cpu_x86_segment_alloc((uintptr_t)context->tls, 0xffffffff, CPU_X86_SEG_DATA_UP_RW)))
    return -ENOMEM;

  CONTEXT_LOCAL_FOREIGN_SET(context->tls, __context_data_base, context->tls);

  /* push param */
  *--context->stack_ptr = (uintptr_t)param;

  /* push context entry function return pointer */
  *--context->stack_ptr = (uintptr_t)0;

  /* push execution pointer */
  *--context->stack_ptr = (uintptr_t)entry;	/* EIP */

  /* push default flags */
  *--context->stack_ptr = 0x00000046;	/* EFLAGS */

  /* room for general purpose registers default values */
  context->stack_ptr -= 8;
#if 0
  context->stack_ptr[0] = 0; 	/* eax */
  context->stack_ptr[-1] = 0; 	/* ecx */
  context->stack_ptr[-2] = 0; 	/* edx */
  context->stack_ptr[-3] = 0; 	/* ebx */
  context->stack_ptr[-4] = 0; 	/* esp (skiped by popa) */
#endif
#if defined (CONFIG_COMPILE_DEBUG)
  /* frame pointer initial value set to zero for debugger */
  context->stack_ptr[-5] = 0; 	/* ebp */
#endif
#if 0
  context->stack_ptr[-6] = 0; 	/* esi */
  context->stack_ptr[-7] = 0; 	/* edi */
#endif

  /* push tls segment index */
  *--context->stack_ptr = tls_seg << 3;	/* GS */

  return 0;
}

void
cpu_context_destroy(struct context_s *context)
{
  reg_t		*stack = (reg_t*)context->stack_ptr;

  /* free tls segment descriptor */
  cpu_x86_segdesc_free((uint16_t)stack[0]);
}

