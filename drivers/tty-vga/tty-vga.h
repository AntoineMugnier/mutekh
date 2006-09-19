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

#ifndef DRIVER_TTY_VGA_H_
#define DRIVER_TTY_VGA_H_

#include <hexo/device/char.h>
#include <hexo/device.h>

/* devices addresses slots */

#define VGA_TTY_ADDR_BUFFER	0
#define VGA_TTY_ADDR_CRTC	1

/* tty device functions */

DEV_IRQ(tty_vga_irq);
DEV_INIT(tty_vga_init);
DEV_CLEANUP(tty_vga_cleanup);
DEVCHAR_READ(tty_vga_read);
DEVCHAR_WRITE(tty_vga_write);

#endif

