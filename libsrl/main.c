
#include <hexo/init.h>
#include <hexo/types.h>
#include <hexo/endian.h>
#include <drivers/device/char/tty-soclib/tty-soclib.h>

#include <hexo/device.h>
#include <hexo/driver.h>

#include <string.h>
#include <stdio.h>

#include <srl.h>
#include <srl_private_types.h>

#ifdef CONFIG_MUTEK_CONSOLE
struct device_s *tty_dev;
static struct device_s tty_con_dev;
extern __ldscript_symbol_t _dsx_tty_address;

lock_t srl_log_lock;

#ifndef SRL_VERBOSITY
#define SRL_VERBOSITY VERB_NONE
#endif

#endif

void hw_init();

int_fast8_t mutek_main(int_fast8_t argc, char **argv)
{
#ifdef CONFIG_MUTEK_CONSOLE
	device_init(&tty_con_dev);
	tty_con_dev.addr[0] = &_dsx_tty_address;
	tty_con_dev.irq = 1;
	tty_soclib_init(&tty_con_dev, NULL, NULL);
	tty_dev = &tty_con_dev;
#endif

	srl_log_printf(NONE, "HW Init... ");
	hw_init();
	srl_log_printf(NONE, "Done\n");

	lock_init(&srl_log_lock);

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

	srl_log_printf(NONE, "Pthread Running %s on cpu %d\n", task->name, cpu_id());
	for (;;) {
		task->func( task->args );
	}
}
#else
static CONTEXT_ENTRY(srl_run_task)
{
	srl_task_s *task = param;
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
	srl_log_printf(NONE, "CPU %i is up and running\n"
		   "DCache: %d bytes/line\n",
		   cpu_id(), cpu_dcache_line_size());
}

void mutek_main_smp(void)
{
  lock_init(&fault_lock);
  cpu_exception_sethandler(fault_handler);

  print_cpu_info();

  cpu_interrupt_disable();

  {
	  srl_cpudesc_s *cur = cpu_desc_list[cpu_id()];
	  size_t i;

	  srl_log_printf(NONE, "Bootstrapping cpu %d: %d tasks\n", cpu_id(), cur->ntasks);
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
