/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Luc Delecroix <luc D delecroix A thalesgroup D com> (c) 2011
    Copyright Laurent Gantel <laurent D gantel A ensea D fr> (c) 2011
*/

#include <hexo/error.h>
#include <hexo/local.h>
#include <hexo/context.h>
#include <hexo/interrupt.h>
#include <mutek/scheduler.h>

CONTEXT_LOCAL struct cpu_context_s mbz_context_regs;

CPU_LOCAL void *__context_data_base;


/**
*	Context bootstrap function.
*/
error_t
cpu_context_bootstrap(struct context_s *context)
{
    /* set context local storage base pointer */
    CPU_LOCAL_SET(__context_data_base, context->tls);

    /* nothing is saved for this context */
    CONTEXT_LOCAL_ADDR(mbz_context_regs)->save_mask = 0;

    return 0;
}


/**
*	Context init function.
*/
error_t
cpu_context_init(struct context_s *context, context_entry_t *entry, void *param)
{

    struct cpu_context_s *regs = CONTEXT_LOCAL_TLS_ADDR(context->tls, mbz_context_regs);

    regs->gpr[5] 	=	(uintptr_t)param; 					/* R5 for passing parameters i.e. thread struct */
    regs->pc    	=	(uintptr_t)entry ;					/* pthread context entry						*/
    regs->save_mask	=  	CPU_MBZ_CONTEXT_RESTORE_CALLER ;	/* what is being saved and restored				*/
    regs->msr 		=	0; 									/* a voir 										*/
    regs->gpr[15] 	=	0xa5a5a5a5; 						/* can not return from context entry 			*/
    regs->gpr[1] 	=	CONTEXT_LOCAL_TLS_GET(context->tls, context_stack_end)
        				- CONFIG_HEXO_STACK_ALIGN;

# ifdef CONFIG_HEXO_FPU
    regs->fsr		=	0 ;
#endif

    return 0;
}


/**
*	Context destroy function.
*/
void
cpu_context_destroy(struct context_s *context)
{
#if 0
    reg_t		*stack = (reg_t*)context->stack_ptr;
#endif
}

