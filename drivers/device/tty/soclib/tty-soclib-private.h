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


#ifndef TTY_VGA_PRIVATE_H_
#define TTY_VGA_PRIVATE_H_

#include <hexo/types.h>
#include <hexo/device.h>

#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_ring.h>
#include <hexo/gpct_lock_hexo.h>


/**************************************************************/

/*
 * Private vgz tty device context
 */

#define CONTAINER_LOCK_tty_fifo HEXO_SPIN_IRQ

CONTAINER_TYPE(tty_fifo, RING, uint8_t, 32);
CONTAINER_FUNC_LOCK(tty_fifo, RING, static inline, tty_fifo, HEXO_SPIN_IRQ);
CONTAINER_FUNC_LOCK(tty_fifo, RING, static inline, tty_fifo_noirq, HEXO_SPIN);
CONTAINER_FUNC_NOLOCK(tty_fifo, RING, static inline, tty_fifo_nolock);

struct tty_soclib_context_s
{
  /* tty input char fifo */
  tty_fifo_root_t		read_fifo;
};

#endif

