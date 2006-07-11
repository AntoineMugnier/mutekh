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


#include <hexo/types.h>
#include <hexo/device.h>
#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include "tty-soclib.h"

#include "tty-soclib-private.h"

#define TTY_SOCLIB_REG_WRITE	0
#define TTY_SOCLIB_REG_STATUS	4
#define TTY_SOCLIB_REG_READ	8

/* 
 * device read operation
 */

DEVCHAR_READ(tty_soclib_read)
{
  struct tty_soclib_context_s	*pv = dev->drv_pv;
  size_t			res;

  lock_spin_irq(&pv->lock);

  res = tty_read_fifo_poplist(&pv->read_fifo, data, size);

  lock_release_irq(&pv->lock);

  return res;
}

/* 
 * device write operation
 */

DEVCHAR_WRITE(tty_soclib_write)
{
  size_t		i;

  for (i = 0; i < size; i++)
    cpu_mem_write_32(dev->addr[0] + TTY_SOCLIB_REG_WRITE, data[i]);

  return size;
}

/* 
 * device close operation
 */

DEV_CLEANUP(tty_soclib_cleanup)
{
  struct tty_soclib_context_s	*pv = dev->drv_pv;

  lock_destroy(&pv->lock);

  mem_free(pv);
}

/*
 * device irq
 */

DEV_IRQ(tty_soclib_irq)
{
  struct tty_soclib_context_s	*pv = dev->drv_pv;
  uint8_t	c;

  lock_spin(&pv->lock);

  /* get character from tty */
  c = cpu_mem_read_8(dev->addr[0] + TTY_SOCLIB_REG_READ);

  /* add character to driver fifo */
  tty_read_fifo_push(&pv->read_fifo, c);

  lock_release(&pv->lock);

  printf("tty input %02x\n", c);

  return 0;
}

/* 
 * device open operation
 */

DEV_INIT(tty_soclib_init)
{
  struct tty_soclib_context_s	*pv;

#ifndef CONFIG_STATIC_DRIVERS
  dev->f_cleanup	= tty_soclib_cleanup;
  dev->f_irq		= tty_soclib_irq;
  dev->chr.f_read	= tty_soclib_read;
  dev->chr.f_write	= tty_soclib_write;
#endif

  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  lock_init(&pv->lock);

  dev->drv_pv = pv;

  /* init tty input fifo */
  tty_read_fifo_init(&pv->read_fifo);

  return 0;
}

