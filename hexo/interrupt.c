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

	Copyright (c) Nicolas Pouillon, <nipo@ssji.net>, 2009
*/

#include <hexo/interrupt.h>

#include <device/icu.h>

#include <device/device.h>
#include <device/driver.h>

CPU_LOCAL cpu_interrupt_handler_t *cpu_interrupt_handler;
CPU_LOCAL void *cpu_interrupt_handler_arg;

void cpu_interrupt_set_handler_device(struct device_s *dev)
{
	printk("Setting CPU IRQ handler for cpuid %d: %p drv: %p\n",
		   cpu_id(), dev, dev->drv);
	assert(dev && dev->drv);

	CPU_LOCAL_SET(cpu_interrupt_handler, dev->drv->f_irq);
	CPU_LOCAL_SET(cpu_interrupt_handler_arg, dev);
}

void cpu_interrupt_set_handler_func(cpu_interrupt_handler_t *handler,
									void *priv)
{
	CPU_LOCAL_SET(cpu_interrupt_handler, handler);
	CPU_LOCAL_SET(cpu_interrupt_handler_arg, priv);
}
