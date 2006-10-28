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

#ifndef DRIVER_INPUT_8042_H_
#define DRIVER_INPUT_8042_H_

#include <hexo/device/input.h>
#include <hexo/device.h>

/* devices addresses slots */

#define UART_8042_ADDR	0

/* input device functions */

DEV_IRQ(input_8042_irq);
DEV_INIT(input_8042_init);
DEV_CLEANUP(input_8042_cleanup);
DEVINPUT_INFO(input_8042_info);
DEVINPUT_READ(input_8042_read);
DEVINPUT_WRITE(input_8042_write);
DEVINPUT_SETCALLBACK(input_8042_setcallback);

#endif

