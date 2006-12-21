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

    Copyright Matthieu Bucchianeri <matthieu.bucchianeri@epita.fr> (c) 2006

*/

#ifndef DRIVER_UART_MC9S12NE64_H_
#define DRIVER_UART_MC9S12NEy64_H_

#include <hexo/device/char.h>
#include <hexo/device.h>

/* tty device functions */

DEV_IRQ(uart_mc9s12ne64_irq);
DEV_INIT(uart_mc9s12ne64_init);
DEV_CLEANUP(uart_mc9s12ne64_cleanup);
DEVCHAR_READ(uart_mc9s12ne64_read);
DEVCHAR_WRITE(uart_mc9s12ne64_write);

#endif

