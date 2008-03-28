/*
 * This file is part of MutekH.
 * 
 * MutekH is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation; version 2.1 of the License.
 * 
 * MutekH is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with MutekH; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 * Copyright (c) UPMC, Lip6, SoC
 *         Nicolas Pouillon <nipo@ssji.net>, 2008
 */

#include <hexo/init.h>
#include <hexo/types.h>
#include <hexo/endian.h>

#include <string.h>
#include <stdio.h>

#include <srl.h>
#include <srl_private_types.h>


void hw_init();
void srl_console_init();
void srl_console_init_cpu(void *addr);
void srl_console_init_task(void *addr);

int_fast8_t mutek_main(int_fast8_t argc, char **argv)
{
	srl_console_init();

	hw_init();

	arch_start_other_cpu();

	mutek_main_smp();
	return 0;
}

static lock_t fault_lock;

static CPU_EXCEPTION_HANDLER(fault_handler)
{
  int_fast8_t		i;
#ifdef CPU_GPREG_NAMES
  const char		*reg_names[] = CPU_GPREG_NAMES;
#endif

  lock_spin(&fault_lock);

  printf("CPU Fault: cpuid(%u) faultid(%u)\n", cpu_id(), type);
  printf("Execution pointer: %p, Bad address (if any): %p\n", execptr, dataptr);
  puts("Registers:");

  for (i = 0; i < CPU_GPREG_COUNT; i++)
#ifdef CPU_GPREG_NAMES
    printf("%s=%p%c", reg_names[i], regtable[i], (i + 1) % 4 ? ' ' : '\n');
#else
    printf("%p%c", regtable[i], (i + 1) % 4 ? ' ' : '\n');
#endif

  puts("Stack top:");

  for (i = 0; i < 8; i++)
    printf("%p%c", stackptr[i], (i + 1) % 4 ? ' ' : '\n');

  lock_release(&fault_lock);

  while (1);
}

extern srl_cpudesc_s *cpu_desc_list[];

#ifdef CONFIG_PTHREAD
static void *srl_run_task( void* param )
{
	srl_task_s *task = param;
	srl_console_init_task(task->tty_addr);

	srl_log_printf(NONE, "Pthread Running %s on cpu %d\n", task->name, cpu_id());
	for (;;) {
		task->func( task->args );
	}
}
#else
static CONTEXT_ENTRY(srl_run_task)
{
	srl_task_s *task = param;
	srl_console_init_task(task->tty_addr);
	sched_unlock();
	cpu_interrupt_enable();

	srl_log_printf(NONE, "Sched Running %s on cpu %d\n", task->name, cpu_id());
	for (;;) {
		task->func( task->args );
	}
}
#endif

static void print_cpu_info()
{
	cpu_printf("CPU %i is up and running\n"
		   "DCache: %d bytes/line\n",
		   cpu_id(), cpu_dcache_line_size());
}

void mutek_main_smp(void)
{
  lock_init(&fault_lock);
  cpu_exception_sethandler(fault_handler);

  srl_cpudesc_s *cur = cpu_desc_list[cpu_id()];
  srl_console_init_cpu(cur->tty_addr);
  srl_console_init_task(cur->tty_addr);

  print_cpu_info();

  cpu_interrupt_disable();

  {
	  size_t i;

	  cpu_printf("Bootstrapping cpu %d: %d tasks\n", cpu_id(), cur->ntasks);
	  for ( i=0; i<cur->ntasks; ++i ) {
		  srl_task_s *task = cur->task_list[i];

		  if ( task->bootstrap ) {
			  task->bootstrap(task->args);
		  }

		  if ( task->func ) {
#ifdef CONFIG_PTHREAD
			  pthread_attr_t attr;
			  pthread_attr_init(&attr);
			  pthread_attr_affinity(&attr, cpu_id());
			  pthread_attr_stack(&attr, task->stack, task->stack_size);
			  pthread_create( &task->pthread, &attr, srl_run_task, task );
			  pthread_attr_destroy(&attr);
#else
			  context_init( &task->context.context,
							task->stack, task->stack_size,
							srl_run_task, task );
			  sched_context_init( &task->context );
			  sched_affinity_single( &task->context, cpu_id() );
			  sched_context_start( &task->context );
#endif
		  }
	  }
  }

  sched_lock();
  sched_context_exit();
}
