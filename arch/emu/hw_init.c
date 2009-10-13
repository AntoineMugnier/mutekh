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

#include <drivers/device/char/tty-emu/tty-emu.h>
#include <drivers/device/timer/emu/timer-emu.h>

#include <hexo/device.h>
#include <device/driver.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutek/timer.h>
#include <mutek/printk.h>

#if defined(CONFIG_MUTEK_CONSOLE)
extern struct device_s *console_dev;
#endif

#if defined(CONFIG_DRIVER_TTY)
struct device_s tty_dev;
#endif

#if defined(CONFIG_DRIVER_TIMER)
struct device_s timer_dev;
#endif

void arch_hw_init()
{
	/* TTY init */
#ifdef CONFIG_DRIVER_TTY
	device_init(&tty_dev);
# if defined(CONFIG_DRIVER_CHAR_EMUTTY)
	tty_emu_init(&tty_dev, NULL);
# else
#  error CONFIG_DRIVER_TTY case not handled in hw_init()
# endif

	console_dev = &tty_dev;
#endif

	/********* Timer init ******************************** */

#if defined(CONFIG_DRIVER_TIMER)
	device_init(&timer_dev);
# if defined(CONFIG_DRIVER_TIMER_EMU)
	timer_emu_init(&timer_dev, NULL);
	dev_timer_setperiod(&timer_dev, 0, 0xffff);
	dev_timer_setcallback(&timer_dev, 0, timer_callback, 0);
# else
#  warning CONFIG_DRIVER_TIMER case not handled in hw_init()
# endif
#endif
}
