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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2010

*/

#include <hexo/interrupt.h>
#include <hexo/local.h>
#include <hexo/atomic.h>
#include <arch/hexo/emu_syscalls.h>

typedef void (*__sighandler_t)(__compiler_sint_t);

extern __compiler_sint_t cpu_pids[CONFIG_CPU_MAXCOUNT];
__compiler_sint_t irq_wait_fd[2];
volatile __sighandler_t irq_state;
CPU_LOCAL atomic_t irq_pending = ATOMIC_INITIALIZER(0);

extern CPU_LOCAL cpu_interrupt_handler_t  *cpu_interrupt_handler;

void emu_interrupts_process(__compiler_sint_t sig)
{
  cpu_interrupt_handler_t *hndl = CPU_LOCAL_GET(cpu_interrupt_handler);

  if (atomic_bit_testclr(CPU_LOCAL_ADDR(irq_pending), 0))
    hndl(0);
}

void emu_interrupts_post(cpu_id_t cpu)
{
  atomic_bit_set(CPU_LOCAL_ID_ADDR(cpu, irq_pending), 0);
  emu_do_syscall(EMU_SYSCALL_KILL, 2, cpu_pids[cpu], EMU_SIG_USR1);
}

void emu_interrupts_set(bool_t state)
{
  if (state)
    {
      if (irq_state == EMU_SIG_IGN)
	emu_do_syscall(EMU_SYSCALL_SIGNAL, 2, EMU_SIG_USR1, irq_state = emu_interrupts_process);
      emu_interrupts_process(0);
    }
  else
    {
      if (irq_state != EMU_SIG_IGN)
	emu_do_syscall(EMU_SYSCALL_SIGNAL, 2, EMU_SIG_USR1, irq_state = EMU_SIG_IGN);
    }
}

bool_t emu_interrupts_get(void)
{
  return irq_state != EMU_SIG_IGN;
}

void emu_interrupts_wait(void)
{
  char dummy;

  /* this will only return on unix signal */
  emu_do_syscall(EMU_SYSCALL_READ, 3, irq_wait_fd[0], &dummy, 1);
}

void emu_interrupts_init(void)
{
  emu_do_syscall(EMU_SYSCALL_PIPE, 1, irq_wait_fd);
  emu_interrupts_set(0);
}

