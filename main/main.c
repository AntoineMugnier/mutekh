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

#include <mutek/init.h>
#include <mutek/types.h>
#include <mutek/device.h>
#include <mutek/endian.h>

#include <mutek/interrupt.h>
#include <mutek/local.h>
#include <mutek/iospace.h>
#include <mutek/lock.h>
#include <mutek/task.h>

#include <mutek/drivers/uart-8250.h>
#include <mutek/drivers/tty-vga.h>
#include <mutek/drivers/tty-soclib.h>
#include <mutek/drivers/icu-8259.h>
#include <mutek/drivers/icu-soclib.h>
#include <mutek/drivers/fb-vga.h>

#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>

#ifdef CONFIG_TTY
lock_t tty_lock;
struct device_s tty_dev = {};
#endif

#ifdef CONFIG_FB
struct device_s fb_dev = {};
#endif

#ifdef CONFIG_TIMER
struct device_s timer_dev = {};
#endif

struct device_s icu_dev = {};

extern const uint8_t mutek_logo_320x200[320*200];

int_fast8_t mutek_main(int_fast8_t argc, char **argv)  /* FIRST CPU only */
{
  /********* ICU init ******************************** */

  /* ICU init */
#if defined(__ARCH__ibmpc__)
  icu_dev.addr[ICU_ADDR_MASTER] = 0x0020;
  icu_dev.addr[ICU_ADDR_SLAVE] = 0x00a0;
  icu_8259_init(&icu_dev);
#elif defined(__ARCH__soclib__)
  icu_dev.addr[ICU_ADDR_MASTER] = 0x10c00000;
  icu_soclib_init(&icu_dev);
#endif

  cpu_interrupt_enable();

  /********* TTY init ******************************** */

  /* TTY init */
#ifdef CONFIG_TTY
  lock_init(&tty_lock);
# if defined(__ARCH__ibmpc__)

#  ifdef CONFIG_TTY_UART
  tty_dev.addr[UART_8250_ADDR] = 0x03f8;
  uart_8250_init(&tty_dev);
#  else	/* CONFIG_TTY_UART */
  tty_dev.addr[VGA_TTY_ADDR_BUFFER] = 0x000b8000;
  tty_dev.addr[VGA_TTY_ADDR_CRTC] = 0x03d4;
  tty_dev.irq = 1;
  tty_vga_init(&tty_dev);
#  endif /* CONFIG_TTY_UART */

# elif defined(__ARCH__soclib__)
  tty_dev.addr[0] = 0xa0c00000;
  tty_dev.irq = 1;
  tty_soclib_init(&tty_dev);
# endif	/* defined(__ARCH__xxx__) */
  DEV_ICU_BIND(&icu_dev, &tty_dev);
#endif /* CONFIG_TTY */

#ifdef CONFIG_TIMER
# if defined(__ARCH__ibmpc__)
  timer_dev.addr[0] = 0x0040;
  timer_dev.irq = 0;
  timer_8253_init(&timer_dev);
# elif defined(__ARCH__soclib__)
  timer_dev.addr[0] = 0x20c00000;
  timer_dev.irq = 0;
  timer_soclib_init(&timer_dev);
# endif	/* defined(__ARCH__xxx__) */
  DEV_ICU_BIND(&icu_dev, &timer_dev);
#endif

  /********* FB init ********************************* */

#ifdef CONFIG_FB
# if defined(__ARCH__ibmpc__)
  fb_vga_init(&fb_dev);
  fb_vga_setmode(&fb_dev, 320, 200, 8, FB_PACK_INDEX);
  uint8_t *p = (void*)fb_vga_getbuffer(&fb_dev, 0);
  memcpy(p, mutek_logo_320x200, 64000);
# endif	/* defined(__ARCH__xxx__) */
#endif /* CONFIG_FB */

  puts("MutekH is alive.");

  //arch_start_other_cpu(); /* let other CPUs enter main_smp() */

  mutek_main_smp();

  return 0;
}

DEVTIMER_CALLBACK(timer_callback)
{
  printf("timer callback\n");
  pthread_yield();
}

static CPU_EXCEPTION_HANDLER(fault_handler)
{
# if defined(__ARCH__ibmpc__)
  static const char *cpu_x86_ex_name[32] =
    {
      [0] = "Divide Error",
      [1] = "Debug",
      [2] = "NMI Interrupt",
      [3] = "Break Point",
      [4] = "Overflow",
      [5] = "BOUND Range Exceeded",
      [6] = "Invalid Opcode",
      [7] = "Device Not Available",
      [8] = "Double Fault",
      [9] = "Coprocessor Segment Overrun",
      [10] = "Invalid TSS",
      [11] = "Segment Not Present",
      [12] = "Stack-Segment Fault",
      [13] = "General Protection Fault",
      [14] = "Page Fault",
      [15] = "",
      [16] = "x87 FPU Floating-Point Error",
      [17] = "Alignment Check",
      [18] = "Machine Check",
      [19] = "SIMD Floating-Point Exception",
      [20 ... 31] = "",
    };

  int_fast8_t		i;

  printf("CPU Fault (%x:%s)\n", type, cpu_x86_ex_name[type]);
  printf("Execution pointer: %p\n", execptr);
  puts("regs:");

  for (i = 0; i < 8; i++)
    printf("%p%c", regtable[i], (i + 1) % 4 ? ' ' : '\n');

#endif /* defined(__ARCH__ibmpc__) */
  while (1);
}

uint32_t a = 16;

void test_thread_free(void*arg)
{
  printf("free %p\n", arg);
}

pthread_t test_thread;

void *test_thread_main(void*arg)
{
  void	*mem;

  printf("test_thread %p\n", pthread_self());
  pthread_yield();

  pthread_cleanup_push(test_thread_free, mem = malloc(512));
  printf("alloc1 %p \n", mem);

  pthread_cleanup_push(test_thread_free, mem = malloc(512));
  printf("alloc2 %p \n", mem);

  while (1)
    pthread_testcancel();

  pthread_cleanup_pop(1);
  pthread_cleanup_pop(1);

  return 0x123;
}

/** application main function */
int_fast8_t main(int_fast8_t argc, char **argv);

void mutek_main_smp(void)  /* ALL CPUs execute this function */
{
  lock_spin(&tty_lock);
  printf("CPU %i is up and running.\n", cpu_id());
  lock_release(&tty_lock);

  cpu_interrupt_ex_sethandler(fault_handler);

  if (cpu_id() == 0)
    {
      char	buf[16];
      ssize_t	res;

      __pthread_bootstrap();
      puts("pthread init done");

      printf("abc %% %u %i %d %s %x %c\n", 0, -128, 123412345, "test", 32, '*');

      printf("abc %% _%u_ _%8u_ _%08u_ _%10u_\n", 128, 128, 128, 128);
      printf("abc %% _%x_ _%8x_ _%08x_ _%10x_\n", 128, 128, 128, 128);
      printf("abc %% _%u_ _%-8u_ _%-08u_ _%-10u_  _%-10u_\n", 128, 128, 128, 128, 128);
      printf("abc %% _%p_ _%s_ _%10s_ _%-10s_ _%010s_\n", "test", "test", "test", "test", "test");

      printf("abc %% %P\n", "abcdefghijklmnopq", 10);

      memset(buf, 0xff, sizeof(buf));

      res = snprintf(buf, 4, "abcdef");
      printf("snprintf(buf, 4, \"abcdef\"); %i _%P_\n", res, buf, 16);

      res = sprintf(buf, "abcdef");
      printf("sprintf(buf, \"abcdef\"); %i _%P_\n", res, buf, 16);

      res = snprintf(buf, 16, "ab%sef", "cd", 1);
      printf("; %i _%P_\n", res, buf, 16);

      printf("%08x %08x\n", endian_be32(16), endian_be32(a));

      //      __pthread_dump_runqueue();

      pthread_create(&test_thread, 0, test_thread_main, 0);

      printf("created thread %p\n", test_thread);

      pthread_cancel(test_thread);

      void *join_retval;

      __pthread_dump_runqueue();

      pthread_join(test_thread, &join_retval);

      __pthread_dump_runqueue();

      printf("joined %p", join_retval);

      /* application main function */
      //      main(0, 0);

      uint_fast8_t	i;

      for (i = 0; i < 3; i++)
	{
	  puts("main thread");
	  pthread_yield();
	}

      printf("plif\n");

#ifdef CONFIG_TIMER
      dev_timer_setperiod(&timer_dev, 0, 0xffff);
      dev_timer_setcallback(&timer_dev, 0, timer_callback, 0);
#endif

      //      main(0, 0);
      while (1)
	;
    }
}

