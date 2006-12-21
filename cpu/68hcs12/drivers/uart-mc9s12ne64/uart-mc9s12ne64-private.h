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

#ifndef UART_MC9S12NE64_PRIVATE_H_
#define UART_MC9S12NE64_PRIVATE_H_

#include <hexo/types.h>
#include <hexo/lock.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_ring.h>
#include <hexo/gpct_lock_hexo.h>

CONTAINER_TYPE(tty_fifo, RING, uint8_t, HEXO_SPIN_IRQ, NOOBJ, 128);
CONTAINER_FUNC(static inline, tty_fifo, RING, tty_fifo, HEXO_SPIN_IRQ);
CONTAINER_FUNC(static inline, tty_fifo, RING, tty_fifo_noirq, HEXO_SPIN);
CONTAINER_FUNC(static inline, tty_fifo, RING, tty_fifo_nolock, NOLOCK);

struct uart_mc9s12ne64_context_s
{
  /* tty input char fifo */
  tty_fifo_root_t			read_fifo;

  lock_t				lock;
};

#endif

