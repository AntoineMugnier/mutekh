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

#include <hexo/init.h>
#include <hexo/types.h>
#include <hexo/endian.h>

#include <hexo/interrupt.h>
#include <hexo/local.h>
#include <hexo/iospace.h>
#include <hexo/lock.h>
#include <hexo/context.h>
#include <hexo/cpu.h>
#include <mutek/scheduler.h>

#include <device/char.h>
#include <device/timer.h>

#include <hexo/device.h>
#include <device/driver.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutek/timer.h>
#include <mutek/printk.h>

#if defined(CONFIG_MUTEK_TIMERMS)
struct timer_s	timer_ms;
#endif

#if defined(CONFIG_DRIVER_TIMER)
DEVTIMER_CALLBACK(timer_callback)
{
	//  printk("timer callback\n");
# if defined(CONFIG_MUTEK_SCHEDULER_PREEMPT)
	sched_context_switch();
# endif

# if defined(CONFIG_MUTEK_TIMERMS)
	timer_inc_ticks(&timer_ms, 10);
# endif
}
#endif

#ifdef CONFIG_MUTEK_MAIN
#ifdef CONFIG_MUTEK_CONSOLE
struct device_s *console_dev;

static inline PRINTF_OUTPUT_FUNC(__printf_out_tty)
{
  while (len > 0)
    {
		ssize_t	res = dev_char_spin_write((struct device_s *)ctx, (uint8_t*)str, len);

      if (res < 0)
	break;
      len -= res;
      str += res;
    }
}

# if defined(CONFIG_DRIVER_UART)
	extern struct device_s uart_dev;
# elif defined(CONFIG_DRIVER_TTY)
	extern struct device_s tty_dev;
# endif
#endif

void arch_hw_init();

#if defined (CONFIG_MUTEK_SCHEDULER)
static struct sched_context_s main_ctx;
#endif

#if defined(CONFIG_LIBC_STREAM_STD)
void stdio_in_out_err_init();
#endif

int_fast8_t mutek_start(int_fast8_t argc, char **argv)  /* FIRST CPU only */
{
#if defined (CONFIG_MUTEK_SCHEDULER)
	context_bootstrap(&main_ctx.context);
	sched_context_init(&main_ctx);
#endif

	arch_hw_init();

#if defined(CONFIG_MUTEK_CONSOLE)
	assert(console_dev);
	printk_set_output(__printf_out_tty, console_dev);
#endif

#if defined(CONFIG_LIBC_STREAM_STD)
	stdio_in_out_err_init();
#endif

    cpu_interrupt_enable();

#if defined (CONFIG_MUTEK_TIMERMS)
	timer_init(&timer_ms.root);
	timer_ms.ticks = 0;
#endif

#if defined (CONFIG_MUTEK_TIMER)
	dev_timer_setperiod(&timer_dev, 0, 1193180 / 100);
	dev_timer_setcallback(&timer_dev, 0, timer_callback, 0);
#endif

  printk("MutekH is alive.\n");

  arch_start_other_cpu(); /* let other CPUs enter main_smp() */

  cpu_interrupt_disable();

  mutek_start_smp();

  return 0;
}

static lock_t fault_lock;

static CPU_EXCEPTION_HANDLER(fault_handler)
{
  int_fast8_t		i;
  reg_t			*sp = (reg_t*)stackptr;
#ifdef CPU_GPREG_NAMES
  const char		*reg_names[] = CPU_GPREG_NAMES;
#endif

#ifdef CPU_FAULT_NAMES
  static const char *const fault_names[CPU_FAULT_COUNT] = CPU_FAULT_NAMES;
  const char *name = type < CPU_FAULT_COUNT ? fault_names[type] : "unknown";
#else
  const char *name = "unknown";
#endif

  lock_spin(&fault_lock);

  printk("CPU Fault: cpuid(%u) faultid(%u-%s)\n", cpu_id(), type, name);
  printk("Execution pointer: %p, Bad address (if any): %p\n"
	 "Registers:"
		 , (void*)execptr, (void*)dataptr);

  for (i = 0; i < CPU_GPREG_COUNT; i++)
#ifdef CPU_GPREG_NAMES
	  printk("%s=%p%c", reg_names[i], (void*)(uintptr_t)regtable[i], (i + 1) % 4 ? ' ' : '\n');
#else
    printk("%p%c", (void*)(uintptr_t)regtable[i], (i + 1) % 4 ? ' ' : '\n');
#endif
#if defined(CONFIG_RTLD)
    printk("hwrena=%p tls=%p\n", regtable[CPU_GPREG_COUNT], regtable[CPU_GPREG_COUNT+1]);
#endif

  printk("Stack top (%p):\n", (void*)stackptr);

  for (i = 0; i < 12; i++)
	  printk("%p%c", (void*)(uintptr_t)sp[i], (i + 1) % 4 ? ' ' : '\n');

  lock_release(&fault_lock);

  while (1)
    ;
}

/** application main function */
void app_start();

void mutek_start_smp(void)  /* ALL CPUs execute this function */
{
  lock_init(&fault_lock);
  cpu_exception_sethandler(fault_handler);

  cpu_interrupt_enable();

  printk("CPU %i is up and running.\n", cpu_id());

#if defined(CONFIG_COMPILE_INSTRUMENT)
  //  hexo_instrument_trace(1);
  //  hexo_instrument_alloc_guard(1);
#endif

  if (cpu_isbootstrap())
    {
      app_start();
      cpu_interrupt_disable();
#if defined(CONFIG_MUTEK_SCHEDULER)
      context_destroy(&main_ctx.context);
      sched_lock();
      sched_context_exit();
#endif
    }
  else
    {
      cpu_interrupt_disable();

#if defined(CONFIG_MUTEK_SCHEDULER)
      sched_lock();
      sched_context_exit();
#endif
    }
}

#endif

