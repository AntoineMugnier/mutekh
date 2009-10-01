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

#include <drivers/device/icu/soclib-icu/icu-soclib.h>

#include <hexo/device.h>
#include <device/driver.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <mutek/timer.h>
#include <mutek/printk.h>

#include <soclib_addresses.h>

#if defined(CONFIG_DRIVER_ICU)
struct device_s icu_dev;
#endif

#if defined(CONFIG_DRIVER_FB)
struct device_s fb_dev;
#endif

#if defined(CONFIG_DRIVER_TTY)
struct device_s tty_dev;
#endif

#if defined(CONFIG_DRIVER_TIMER)
struct device_s timer_dev;
#endif

#if defined(CONFIG_DRIVER_BLOCK)
struct device_s block_dev;
#endif

#ifdef CONFIG_MUTEK_CONSOLE
extern struct device_s *console_dev;
#endif

# if defined(CONFIG_DRIVER_ICU_SOCLIB_XICU) && defined(CONFIG_SMP)
struct device_s xicu_dev[CONFIG_CPU_MAXCOUNT];
# endif

void arch_hw_init()
{
#if defined(CONFIG_DRIVER_ICU)
	device_init(&icu_dev);
# if defined(CONFIG_DRIVER_ICU_SOCLIB)
	icu_dev.addr[ICU_ADDR_MASTER] = DSX_SEGMENT_ICU_ADDR;
	icu_dev.irq = 0;
	icu_dev.icudev = CPU_LOCAL_ADDR(cpu_icu_dev);
	icu_soclib_init(&icu_dev, NULL);
# elif defined(CONFIG_DRIVER_ICU_SOCLIB_XICU)
	icu_dev.addr[XICU_ADDR_MASTER] = DSX_SEGMENT_XICU_ADDR;
	icu_dev.addr[XICU_OUT_INDEX] = cpu_id();
	icu_dev.irq = 0;
	icu_dev.icudev = CPU_LOCAL_ADDR(cpu_icu_dev);
	{
		struct soclib_xicu_param_s params = {
			.output_line_no = cpu_id(),
		};
		xicu_soclib_init(&icu_dev, &params);
	}
# else
#  error CONFIG_DRIVER_ICU case not handled in arch_hw_init()
# endif
#endif

	/* TTY init */
#ifdef CONFIG_DRIVER_TTY
	device_init(&tty_dev);
# if defined(CONFIG_DRIVER_CHAR_SOCLIBTTY)
	tty_dev.addr[0] = DSX_SEGMENT_TTY_ADDR;
#  if defined(CONFIG_DRIVER_ICU_SOCLIB)
	tty_dev.irq = 1;
#  elif defined(CONFIG_DRIVER_ICU_SOCLIB_XICU)
	tty_dev.irq = 0;
#  endif
	tty_dev.icudev = &icu_dev;
	tty_soclib_init(&tty_dev, NULL);
#  ifdef CONFIG_MUTEK_CONSOLE
	console_dev = &tty_dev;
#  endif
# else
#  warning CONFIG_DRIVER_TTY case not handled in arch_hw_init()
# endif
#endif

	/********* Block device init ************************* */
#if defined(CONFIG_DRIVER_BLOCK)
# if defined(CONFIG_DRIVER_BLOCK_SOCLIB)
	device_init(&bd_dev);
	bd_dev.addr[0] = DSX_SEGMENT_BD_ADDR;
	bd_dev.irq = 2; // 0 is timer and 1 is tty
	bd_dev.icudev = &icu_dev;
	block_soclib_init(&bd_dev, NULL);
# else
#  warning CONFIG_DRIVER_BLOCK case not handled in arch_hw_init()
# endif
#endif

	/********* Timer init ******************************** */

#if defined(CONFIG_DRIVER_TIMER)
	device_init(&timer_dev);
# if defined(CONFIG_DRIVER_TIMER_SOCLIB)
	timer_dev.addr[0] = DSX_SEGMENT_TIMER_ADDR;
	timer_dev.irq = 0;
	timer_dev.icudev = &icu_dev;
	timer_soclib_init(&timer_dev, NULL);
# else
#  warning CONFIG_DRIVER_TIMER case not handled in arch_hw_init()
# endif
#endif

	/********* FB init ********************************* */

#if defined(CONFIG_DRIVER_FB)
	device_init(&fb_dev);
# if defined(CONFIG_DRIVER_FB_SOCLIB)
	fb_dev.addr[0] = DSX_SEGMENT_FB_ADDR;
	fb_soclib_init(&fb_dev, NULL);
# else
#  warning CONFIG_DRIVER_FB case not handled in arch_hw_init()
# endif
# if defined(CONFIG_MUTEK_LOGO)
	uint8_t *p = (void*)fb_vga_getbuffer(&fb_dev, 0);
	memcpy(p, mutek_logo_320x200, 64000);
# endif
#endif
}


/*


# if defined(CONFIG_DRIVER_ICU_SOCLIB_XICU)
	  {
		  struct device_s *icu_dev = &icu_dev_2[cpu_id()];
		  device_init(icu_dev);
		  icu_dev->addr[XICU_ADDR_MASTER] = DSX_SEGMENT_XICU_ADDR;
		  icu_dev->addr[XICU_OUT_INDEX] = cpu_id();
		  icu_dev->irq = 0;
		  icu_dev->icudev = CPU_LOCAL_ADDR(cpu_icu_dev);
		  struct soclib_xicu_param_s params = {
			  .output_line_no = cpu_id(),
		  };
		  xicu_soclib_init(icu_dev, &params);
	  }
# endif

*/
