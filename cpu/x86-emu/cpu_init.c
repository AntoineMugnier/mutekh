/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/


#include <hexo/alloc.h>
#include <string.h>
#include <hexo/interrupt.h>
#include <hexo/init.h>
#include <hexo/iospace.h>
#include <hexo/lock.h>
#include <hexo/segment.h>
#include <arch/hexo/emu_syscalls.h>

CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_hw_handler;
CPU_LOCAL cpu_exception_handler_t  *cpu_interrupt_ex_handler;
CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_sys_handler;

/* cpu interrupts state */
volatile CPU_LOCAL bool_t cpu_irq_state = 0;

error_t
cpu_global_init(void)
{
  return 0;
}

struct cpu_cld_s
{
  /* CPU id */
  uint32_t			id;
  /* PID of the worker unix process */
  int32_t			worker_pid;
  /* PID of the unix process used to perform ptrace ops */
  int32_t			tracer_pid;
};

//static CPU_LOCAL struct cpu_cld_s	*cpu_cld;

#define TRACER_STACK_SIZE	65535

void tracer_entry(struct cpu_cld_s *cld)
{
  int32_t	status;

  emu_do_syscall(EMU_SYSCALL_PTRACE, 4,
		 EMU_PTRACE_ATTACH,
		 cld->worker_pid, 0, 0);

  emu_do_syscall(EMU_SYSCALL_WAITPID, 3,
		 cld->worker_pid, &status, 0);

  emu_do_syscall(EMU_SYSCALL_PTRACE, 4,
		 EMU_PTRACE_CONT,
		 cld->worker_pid, 0, 0);

  emu_do_syscall(EMU_SYSCALL_PTRACE, 4,
		 EMU_PTRACE_CONT,
		 cld->worker_pid, 0, 0);

  while (1);
}

struct cpu_cld_s *cpu_init(uint_fast8_t cpu_id)
{
  struct cpu_cld_s	*cld;
  reg_t			*tracer_stack;
  //void			*cls;

  if (!(cld = mem_alloc(sizeof (struct cpu_cld_s), MEM_SCOPE_SYS)))
    return NULL;

  /* allocate memory for tracer process stack */
  tracer_stack = (void*)emu_do_syscall(EMU_SYSCALL_MMAP, 6, NULL, TRACER_STACK_SIZE * sizeof(reg_t),
				       EMU_PROT_READ | EMU_PROT_WRITE,
				       EMU_MAP_PRIVATE | EMU_MAP_ANONYMOUS, 0, 0);

  tracer_stack += TRACER_STACK_SIZE;

  *--tracer_stack = (reg_t)cld;

  if (tracer_stack == EMU_MAP_FAILED)
    return NULL;

  cld->id = cpu_id;

  cld->worker_pid = emu_do_syscall(EMU_SYSCALL_GETPID, 0);

  /* FIXME registers may be clobbered by syscall */
  asm volatile (
		"	int $0x80		\n"
		"	test %0, %0		\n"
		"	jnz 1f			\n"
#ifdef CONFIG_COMPILE_FRAMEPTR
		"	xorl	%%ebp, %%ebp	\n"
#endif
		"	call	tracer_entry	\n"
		"1:				\n"
		: "=a" (cld->tracer_pid)
		: "a" (EMU_SYSCALL_CLONE)
		, "b" (EMU_CLONE_PARENT | EMU_CLONE_FS | EMU_CLONE_FILES | EMU_CLONE_VM)
		, "c" (tracer_stack)
		);

  if (cld->tracer_pid < 0)
    return NULL;

#if defined(CONFIG_DEBUG)
  /* enable alignment check */
  asm volatile("	pushf						\n"
	       "	orl	$0x40000, (%esp)			\n"
	       "	popf						\n");
#endif

  return cld;
}

void cpu_start_other_cpu(void)
{
}

uint_fast8_t cpu_id(void)
{
#ifdef CONFIG_SMP
  struct cpu_cld_s	*cld = CPU_LOCAL_GET(cpu_cld);

  return cld->id;
#else
  return 0;
#endif
}

