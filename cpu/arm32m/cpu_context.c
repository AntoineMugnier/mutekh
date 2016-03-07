/*
   This file is part of MutekH.
   
   MutekH is free software; you can redistribute it and/or modify it
   under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation; version 2.1 of the License.
   
   MutekH is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
   License for more details.
   
   You should have received a copy of the GNU Lesser General Public
   License along with MutekH; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
   02110-1301 USA.
  
   Copyright (c) 2013, Alexandre Becoulet <alexandre.becoulet@free.fr>
*/

#include <hexo/error.h>
#include <hexo/local.h>
#include <hexo/context.h>
#include <hexo/interrupt.h>
#include <mutek/scheduler.h>

#ifdef CONFIG_HEXO_CONTEXT_PREEMPT
CPU_LOCAL context_preempt_t *cpu_preempt_handler = (context_preempt_t*)1;
#endif

#ifdef CONFIG_HEXO_CONTEXT_IRQEN
CPU_LOCAL context_irqen_t *cpu_irqen_handler = NULL;
#endif

CONTEXT_LOCAL struct cpu_context_s arm_context_regs;

void cpu_context_entry();

error_t
cpu_context_bootstrap(struct context_s *context)
{
    asm volatile ("msr   psp, %0" :: "l" (context->tls));

#ifdef CONFIG_CPU_ARM32M_MPU_STACK_GUARD
    cpu_context_stack_guard(CONTEXT_LOCAL_GET(context_stack_start));
#endif

    return 0;
}

/* context init function */


error_t
cpu_context_init(struct context_s *context, context_entry_t *entry, void *param)
{
    struct cpu_context_s *regs = CONTEXT_LOCAL_TLS_ADDR(context->tls, arm_context_regs);

    regs->gpr[13] =
        CONTEXT_LOCAL_TLS_GET(context->tls, context_stack_end)
        - CONFIG_HEXO_STACK_ALIGN;
# ifdef CONFIG_COMPILE_FRAMEPTR
    regs->gpr[11] = regs->gpr[13];
# endif

    regs->sys.primask = 0x1;  /* irq disabled */
    regs->sys.exc_mode = 0;

    regs->gpr[15] = (uintptr_t)cpu_context_entry;
    regs->gpr[4] = (uintptr_t)entry;
    regs->gpr[5] = (uintptr_t)param;

    return 0;
}



void
cpu_context_destroy(struct context_s *context)
{
#if 0
    reg_t		*stack = (reg_t*)context->stack_ptr;
#endif
}

void cpu_exception_resume_pc(struct cpu_context_s *regs, uintptr_t pc)
{
    regs->gpr[15] = pc;
}

#ifdef CONFIG_HEXO_CONTEXT_IRQEN
void arm_interrupt_restore(void)
{
  context_irqen_t *func = CPU_LOCAL_GET(cpu_irqen_handler);

  reg_t ipsr;
  asm ("mrs  %0, ipsr" : "=l" (ipsr));

  if (func != NULL && !(ipsr & 0xff))
    func();
}
#endif

