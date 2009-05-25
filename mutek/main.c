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

#include <drivers/device/char/uart-8250/uart-8250.h>
#include <drivers/device/char/tty-vga/tty-vga.h>
#include <drivers/device/char/tty-soclib/tty-soclib.h>
#include <drivers/device/char/tty-emu/tty-emu.h>
#include <drivers/device/icu/8259/icu-8259.h>
#include <drivers/device/icu/soclib/icu-soclib.h>
#include <drivers/device/timer/soclib/timer-soclib.h>
#include <drivers/device/timer/8253/timer-8253.h>
#include <drivers/device/timer/emu/timer-emu.h>
#include <drivers/device/input/8042/input-8042.h>
#include <drivers/device/fb/vga/fb-vga.h>
#include <drivers/device/enum/pci/enum-pci.h>
#include <drivers/device/enum/isapnp/enum-isapnp.h>
#include <drivers/device/net/ne2000/net-ne2000.h>
#include <drivers/device/net/3c900/net-3c900.h>

#include <hexo/device.h>
#include <device/driver.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutek/timer.h>

#ifdef CONFIG_ARCH_IBMPC_DMA
#include <arch/dma-8237.h>
#endif

#ifdef CONFIG_ARCH_SOCLIB
#include <soclib_addresses.h>
#endif

#if defined(CONFIG_MUTEK_CONSOLE)
struct device_s *tty_dev;
#endif

#if defined(CONFIG_DRIVER_UART)
struct device_s uart_dev;
#endif

#if defined(CONFIG_DRIVER_FB)
struct device_s fb_dev;
#endif

#if defined(CONFIG_DRIVER_TIMER)
struct device_s timer_dev;
#endif

#if defined(CONFIG_DRIVER_KEYBOARD)
struct device_s keyboard_dev;
#endif

#if defined(CONFIG_DRIVER_ENUM_PCI)
struct device_s enum_pci;
#endif

#if defined(CONFIG_DRIVER_ENUM_ISAPNP)
struct device_s enum_isapnp;
#endif

#if defined(CONFIG_MUTEK_TIMERMS)
struct timer_s	timer_ms;
#endif

#if defined(CONFIG_MUTEK_LOGO)
extern const uint8_t mutek_logo_320x200[320*200];
#endif

#if defined(CONFIG_DRIVER_ICU)
struct device_s icu_dev;
#endif

#if defined(CONFIG_DRIVER_BLOCK)
struct device_s bd_dev;
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

static struct sched_context_s main_ctx;

int_fast8_t mutek_main(int_fast8_t argc, char **argv)  /* FIRST CPU only */
{
#if defined(CONFIG_MUTEK_SCHEDULER)
  context_bootstrap(&main_ctx.context);
  sched_context_init(&main_ctx);
#endif

  /********* ICU init ******************************** */

#if defined(CONFIG_DRIVER_ICU)

  device_init(&icu_dev);
# if defined(CONFIG_DRIVER_ICU_8259)
  icu_dev.addr[ICU_ADDR_MASTER] = 0x0020;
  icu_dev.addr[ICU_ADDR_SLAVE] = 0x00a0;
  icu_8259_init(&icu_dev, NULL, NULL);
# elif defined(CONFIG_DRIVER_ICU_SOCLIB)
  icu_dev.addr[ICU_ADDR_MASTER] = DSX_SEGMENT_ICU_ADDR;
  icu_soclib_init(&icu_dev, NULL, NULL);
#  warning CONFIG_DRIVER_ICU case not handled in mutek_main()
# endif
#endif

  cpu_interrupt_enable();

  /********* TTY init ******************************** */

#if defined(CONFIG_DRIVER_UART)
  device_init(&uart_dev);
# if defined(CONFIG_DRIVER_CHAR_UART8250)
  uart_dev.addr[UART_8250_ADDR] = 0x03f8;
  uart_dev.irq = 4;
  uart_8250_init(&uart_dev, &icu_dev, NULL);
  DEV_ICU_BIND(&icu_dev, &uart_dev);
# else
#  warning CONFIG_DRIVER_UART case not handled in mutek_main()
# endif
#endif

  /* TTY init */
#ifdef CONFIG_DRIVER_TTY
  static struct device_s tty_con_dev;

# if defined(CONFIG_DRIVER_CHAR_VGATTY)
  device_init(&tty_con_dev);
  tty_con_dev.addr[VGA_TTY_ADDR_BUFFER] = 0x000b8000;
  tty_con_dev.addr[VGA_TTY_ADDR_CRTC] = 0x03d4;
  tty_con_dev.irq = 1;
  tty_vga_init(&tty_con_dev, &icu_dev, NULL);
#  if defined(CONFIG_MUTEK_CONSOLE)
  tty_dev = &tty_con_dev;
#  endif
  DEV_ICU_BIND(&icu_dev, &tty_con_dev);
# elif defined(CONFIG_DRIVER_CHAR_SOCLIBTTY)
  device_init(&tty_con_dev);
  tty_con_dev.addr[0] = DSX_SEGMENT_TTY_ADDR;
  tty_con_dev.irq = 1;
  tty_soclib_init(&tty_con_dev, &icu_dev, NULL);
#  if defined(CONFIG_MUTEK_CONSOLE)
  tty_dev = &tty_con_dev;
#  endif
  DEV_ICU_BIND(&icu_dev, &tty_con_dev);
# elif defined(CONFIG_DRIVER_CHAR_EMUTTY)
  device_init(&tty_con_dev);
  tty_emu_init(&tty_con_dev, NULL, NULL);
#  if defined(CONFIG_MUTEK_CONSOLE)
  tty_dev = &tty_con_dev;
#  endif
# else
#  warning CONFIG_DRIVER_TTY case not handled in mutek_main()
# endif
#endif

#if defined(CONFIG_MUTEK_CONSOLE)
# if defined(CONFIG_DRIVER_TTY)
  tty_dev = &tty_con_dev;
# elif defined(CONFIG_DRIVER_UART)
  tty_dev = &uart_dev;
# endif
#endif

  /********* Block device init ************************* */
#if defined(CONFIG_DRIVER_BLOCK)
# if defined(CONFIG_DRIVER_BLOCK_SOCLIB)
  device_init(&bd_dev);
  bd_dev.addr[0] = DSX_SEGMENT_BD_ADDR;
  bd_dev.irq = 2; // 0 is timer and 1 is tty
  block_soclib_init(&bd_dev, &icu_dev, NULL);
  DEV_ICU_BIND(&icu_dev, &bd_dev);
# else
#  warning CONFIG_DRIVER_BLOCK case not handled in mutek_main()
# endif
#endif

  /********* Timer init ******************************** */

#if defined(CONFIG_DRIVER_TIMER)
  device_init(&timer_dev);
# if defined(CONFIG_DRIVER_TIMER_8253)
  timer_dev.addr[0] = 0x0040;
  timer_dev.irq = 0;
  timer_8253_init(&timer_dev, &icu_dev, NULL);
  dev_timer_setperiod(&timer_dev, 0, 1193180 / 100);
  DEV_ICU_BIND(&icu_dev, &timer_dev);
  dev_timer_setcallback(&timer_dev, 0, timer_callback, 0);
# elif defined(CONFIG_DRIVER_TIMER_SOCLIB)
  timer_dev.addr[0] = DSX_SEGMENT_TIMER_ADDR;
  timer_dev.irq = 0;
  timer_soclib_init(&timer_dev, &icu_dev, NULL);
  dev_timer_setperiod(&timer_dev, 0, 0xffff);
  DEV_ICU_BIND(&icu_dev, &timer_dev);
  dev_timer_setcallback(&timer_dev, 0, timer_callback, 0);
# elif defined(CONFIG_DRIVER_TIMER_EMU)
  timer_emu_init(&timer_dev, NULL, NULL);
  dev_timer_setperiod(&timer_dev, 0, 0xffff);
  dev_timer_setcallback(&timer_dev, 0, timer_callback, 0);
# else
#  warning CONFIG_DRIVER_TIMER case not handled in mutek_main()
# endif
#endif

#if defined (CONFIG_MUTEK_TIMERMS)
  timer_init(&timer_ms.root);
  timer_ms.ticks = 0;
#endif

#if defined(CONFIG_DRIVER_KEYBOARD)
  device_init(&keyboard_dev);
# if defined(CONFIG_DRIVER_INPUT_8042)
  keyboard_dev.addr[0] = 0x60;
  keyboard_dev.irq = 1;
  input_8042_init(&keyboard_dev, &icu_dev, NULL);
  DEV_ICU_BIND(&icu_dev, &keyboard_dev);
# else
#  warning CONFIG_DRIVER_KEYBOARD case not handled in mutek_main()
# endif
#endif

  /********* FB init ********************************* */

#if defined(CONFIG_DRIVER_FB)
  device_init(&fb_dev);
# if defined(CONFIG_DRIVER_FB_VGA)
  fb_vga_init(&fb_dev, &icu_dev, NULL);
  fb_vga_setmode(&fb_dev, 320, 200, 8, FB_PACK_INDEX);
#  if defined(CONFIG_MUTEK_LOGO)
  uint8_t *p = (void*)fb_vga_getbuffer(&fb_dev, 0);
  memcpy(p, mutek_logo_320x200, 64000);
#  endif
# elif defined(CONFIG_DRIVER_FB_SOCLIB)
  fb_dev.addr[0] = DSX_SEGMENT_FB_ADDR;
  fb_soclib_init(&fb_dev, NULL, NULL);
# else
#  warning CONFIG_DRIVER_FB case not handled in mutek_main()
# endif
#endif

  printk("MutekH is alive.\n");

#if defined(CONFIG_DRIVER_ENUM_PCI)
  device_init(&enum_pci);
  enum_pci_init(&enum_pci, &icu_dev, NULL);
#endif

#if defined(CONFIG_DRIVER_ENUM_ISAPNP)
  device_init(&enum_isapnp);
  enum_isapnp_init(&enum_isapnp, &icu_dev, NULL);
#endif

#if defined(CONFIG_DRIVER_ENUM_PCI) && defined(CONFIG_DRIVER_NET_3C900)
  dev_enum_register(&enum_pci, &net_3c900_drv);
#endif

#if defined(CONFIG_DRIVER_ENUM_PCI) && defined(CONFIG_DRIVER_NET_NE2000)
  dev_enum_register(&enum_pci, &net_ne2000_drv);
#endif

#if defined(CONFIG_DRIVER_NET_NE2000)
#if 0
  /* driver for the D-Link DE200-TP */
  static struct device_s net_dlink_200tp;

  device_init(&net_dlink_200tp);
  net_dlink_200tp.addr[0] = 0x320;
  net_dlink_200tp.irq = 5;
  net_ne2000_init(&net_dlink_200tp, &icu_dev, NULL);
#endif
#if 0
  /* driver for the UMC9008 */
  static struct device_s net_umc_9008;

  device_init(&net_umc_9008);
  net_umc_9008.addr[0] = 0x300;
  net_umc_9008.irq = 3;
  net_ne2000_init(&net_umc_9008, &icu_dev, NULL);
#endif
# endif

#ifdef CONFIG_ARCH_IBMPC_DMA
  dma_8237_init();
#endif

  arch_start_other_cpu(); /* let other CPUs enter main_smp() */

  mutek_main_smp();

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
	 , execptr, dataptr);

  for (i = 0; i < CPU_GPREG_COUNT; i++)
#ifdef CPU_GPREG_NAMES
    printk("%s=%p%c", reg_names[i], regtable[i], (i + 1) % 4 ? ' ' : '\n');
#else
    printk("%p%c", regtable[i], (i + 1) % 4 ? ' ' : '\n');
#endif
#if defined(CONFIG_LIBRTLD)
    printk("hwrena=%p tls=%p\n", regtable[CPU_GPREG_COUNT], regtable[CPU_GPREG_COUNT+1]);
#endif

  printk("Stack top (%p):\n", stackptr);

  for (i = 0; i < 12; i++)
    printk("%p%c", sp[i], (i + 1) % 4 ? ' ' : '\n');

  lock_release(&fault_lock);

  while (1)
    ;
}

/** application main function */
int_fast8_t main(size_t argc, char **argv);

void mutek_main_smp(void)  /* ALL CPUs execute this function */
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
      main(0, 0);
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

