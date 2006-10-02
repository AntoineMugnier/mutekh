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

#include <../drivers/uart-8250/uart-8250.h>
#include <../drivers/tty-vga/tty-vga.h>
#include <../drivers/tty-soclib/tty-soclib.h>
#include <../drivers/icu-8259/icu-8259.h>
#include <../drivers/icu-soclib/icu-soclib.h>
#include <../drivers/timer-soclib/timer-soclib.h>
#include <../drivers/timer-8253/timer-8253.h>
#include <../drivers/fb-vga/fb-vga.h>
#include <../drivers/enum-pci/enum-pci.h>
#include <../drivers/enum-isapnp/enum-isapnp.h>
#include <../drivers/net-ne2000/net-ne2000.h>
#include <../drivers/net-3c900/net-3c900.h>

#include <hexo/device.h>
#include <hexo/driver.h>

#include <string.h>
#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>

#ifdef CONFIG_TTY
lock_t tty_lock;
struct device_s *tty_dev;
#endif

#ifdef CONFIG_FB
struct device_s fb_dev;
#endif

#ifdef CONFIG_TIMER
struct device_s timer_dev;
#endif

struct device_s tty_uart_dev;
struct device_s tty_con_dev;

struct device_s icu_dev;

struct device_s enum_pci;
struct device_s enum_isapnp;

extern const uint8_t mutek_logo_320x200[320*200];

DEVTIMER_CALLBACK(timer_callback)
{
  //  printf("timer callback\n");
  //  pthread_yield();
}

int_fast8_t mutek_main(int_fast8_t argc, char **argv)  /* FIRST CPU only */
{
  sched_global_init();
  sched_cpu_init();

  /********* ICU init ******************************** */

  /* ICU init */
  device_init(&icu_dev);
#if defined(__ARCH__ibmpc__)
  icu_dev.addr[ICU_ADDR_MASTER] = 0x0020;
  icu_dev.addr[ICU_ADDR_SLAVE] = 0x00a0;
  icu_8259_init(&icu_dev, NULL);
#elif defined(__ARCH__soclib__)
  icu_dev.addr[ICU_ADDR_MASTER] = 0x10c00000;
  icu_soclib_init(&icu_dev, NULL);
#endif

  /********* TTY init ******************************** */

#ifdef CONFIG_UART
  device_init(&tty_uart_dev);
# if defined(__ARCH__ibmpc__)
  tty_uart_dev.addr[UART_8250_ADDR] = 0x03f8;
  tty_uart_dev.irq = 4;
  uart_8250_init(&tty_uart_dev, &icu_dev);
  DEV_ICU_BIND(&icu_dev, &tty_uart_dev);
# endif
#endif

  /* TTY init */
#ifdef CONFIG_TTY
  lock_init(&tty_lock);
# if defined(__ARCH__ibmpc__)

#  ifdef CONFIG_TTY_UART
  tty_dev = &tty_uart_dev;
#  else	/* CONFIG_TTY_UART */
  device_init(&tty_con_dev);
  tty_con_dev.addr[VGA_TTY_ADDR_BUFFER] = 0x000b8000;
  tty_con_dev.addr[VGA_TTY_ADDR_CRTC] = 0x03d4;
  tty_con_dev.irq = 1;
  tty_vga_init(&tty_con_dev, &icu_dev);
  tty_dev = &tty_con_dev;
  DEV_ICU_BIND(&icu_dev, &tty_con_dev);
#  endif /* CONFIG_TTY_UART */

# elif defined(__ARCH__soclib__)
  device_init(&tty_con_dev);
  tty_con_dev.addr[0] = 0xa0c00000;
  tty_con_dev.irq = 1;
  tty_soclib_init(&tty_con_dev, &icu_dev);
  tty_dev = &tty_con_dev;
  DEV_ICU_BIND(&icu_dev, &tty_con_dev);
# endif	/* defined(__ARCH__xxx__) */
#endif /* CONFIG_TTY */

  /********* Timer init ******************************** */

#ifdef CONFIG_TIMER
  device_init(&timer_dev);
# if defined(__ARCH__ibmpc__)
  timer_dev.addr[0] = 0x0040;
  timer_dev.irq = 0;
  timer_8253_init(&timer_dev, &icu_dev);
# elif defined(__ARCH__soclib__)
  timer_dev.addr[0] = 0x20c00000;
  timer_dev.irq = 0;
  timer_soclib_init(&timer_dev, &icu_dev);
# endif	/* defined(__ARCH__xxx__) */
  DEV_ICU_BIND(&icu_dev, &timer_dev);

  dev_timer_setperiod(&timer_dev, 0, 0xffff);
  dev_timer_setcallback(&timer_dev, 0, timer_callback, 0);
#endif

  /********* FB init ********************************* */

#ifdef CONFIG_FB
  device_init(&fb_dev);
# if defined(__ARCH__ibmpc__)
  fb_vga_init(&fb_dev, &icu_dev);
  fb_vga_setmode(&fb_dev, 320, 200, 8, FB_PACK_INDEX);
  uint8_t *p = (void*)fb_vga_getbuffer(&fb_dev, 0);
  memcpy(p, mutek_logo_320x200, 64000);
# endif	/* defined(__ARCH__xxx__) */
#endif /* CONFIG_FB */

  puts("MutekH is alive.");

# if defined(__ARCH__ibmpc__)
  device_init(&enum_pci);
  enum_pci_init(&enum_pci, &icu_dev);

  device_init(&enum_isapnp);
  enum_isapnp_init(&enum_isapnp, &icu_dev);

  dev_enum_register(&enum_pci, &net_3c900_drv);
  dev_enum_register(&enum_pci, &net_ne2000_drv);

  struct device_s *isawd;

  isawd = malloc(sizeof (struct device_s));

  device_init(isawd);
  isawd->addr[0] = 0x320;
  isawd->irq = 3;
  net_ne2000_init(isawd, &icu_dev);

# endif

  arch_start_other_cpu(); /* let other CPUs enter main_smp() */

  mutek_main_smp();

  return 0;
}

static CPU_EXCEPTION_HANDLER(fault_handler)
{
  int_fast8_t		i;

  printf("CPU Fault: cpuid(%u) faultid(%u)\n", cpu_id(), type);
  printf("Execution pointer: %p\n", execptr);
  puts("regs:");

#if defined(__CPU__x86__)
  for (i = 0; i < 8; i++)
#elif defined(__CPU__mips__)
  for (i = 0; i < 32; i++)
#else
# error
#endif
    printf("%p%c", regtable[i], (i + 1) % 4 ? ' ' : '\n');

  while (1);
}

/** application main function */
int_fast8_t main(int_fast8_t argc, char **argv);

void mutek_main_smp(void)  /* ALL CPUs execute this function */
{
  if (!cpu_isbootstrap())
    {
      sched_cpu_init();
    }

  cpu_interrupt_ex_sethandler(fault_handler);

  lock_spin(&tty_lock);
  printf("CPU %i is up and running.\n", cpu_id());
  lock_release(&tty_lock);

  if (cpu_id() == 0)
    main(0, 0);

  sched_lock();
  sched_context_exit();
}

