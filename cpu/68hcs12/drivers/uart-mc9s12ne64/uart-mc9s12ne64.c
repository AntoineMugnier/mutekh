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

#include <MC9S12NE64.h>
#include <hexo/types.h>

#include <device/char.h>
#include <hexo/device.h>
#include <device/driver.h>

#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include "uart-mc9s12ne64.h"

#include "uart-mc9s12ne64-private.h"

/**************************************************************/

/*
 * device read operation
 */

DEVCHAR_READ(uart_mc9s12ne64_read)
{
  struct uart_mc9s12ne64_context_s	*pv = dev->drv_pv;
  size_t				res;

  res = tty_fifo_pop_array(&pv->read_fifo, data, size);

  return res;
}

/*
 * device write operation
 */

DEVCHAR_WRITE(uart_mc9s12ne64_write)
{
  struct uart_mc9s12ne64_context_s	*pv = dev->drv_pv;
  uint_fast16_t				i;

  lock_spin_irq(&pv->lock);

  for (i = 0; i < size; i++)
    /* process each char */
    {
      if (!SCI0SR1_TDRE)
	break;

      SCI0DRL = data[i];
  DDRG = 0x03;
  PTG = 0x02;

    }

  lock_release_irq(&pv->lock);

  return i;
}

/*
 * device close operation
 */

DEV_CLEANUP(uart_mc9s12ne64_cleanup)
{
  struct uart_mc9s12ne64_context_s	*pv = dev->drv_pv;

  tty_fifo_destroy(&pv->read_fifo);

  lock_destroy(&pv->lock);

  mem_free(pv);
}

/*
 * IRQ handler
 */

DEV_IRQ(uart_mc9s12ne64_irq)
{
  struct uart_mc9s12ne64_context_s	*pv = dev->drv_pv;

  DDRG = 0x03;
  PTG = 0x01;

  if (!SCI0SR1_RDRF)
    return 0;

  while (SCI0SR1_RDRF)
    tty_fifo_noirq_pushback(&pv->read_fifo, SCI0DRL);

  return 1;
}

/*
 * device open operation
 */

#ifndef CONFIG_STATIC_DRIVERS
const struct driver_s	uart_mc9s12ne64_drv =
{
  .f_init		= uart_mc9s12ne64_init,
  .f_cleanup		= uart_mc9s12ne64_cleanup,
  .f_irq		= uart_mc9s12ne64_irq,
  .f.chr = {
    .f_read		= uart_mc9s12ne64_read,
    .f_write		= uart_mc9s12ne64_write,
  }
};
#endif

DEV_INIT(uart_mc9s12ne64_init)
{
  struct uart_mc9s12ne64_context_s	*pv;

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &uart_mc9s12ne64_drv;
#endif

  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  lock_init(&pv->lock);

  dev->drv_pv = pv;

  /* init tty input fifo */
  tty_fifo_init(&pv->read_fifo);

  /* init SCI0 */
  SCI0BD_IREN = 0;
  SCI0BD_TNP = 0;
  /* XXX baud rate */
  _SCI0BD.Overlap_STR.SCI0BDLSTR.Byte = 13;


  SCI0CR1 = 0;
  SCI0CR2 = SCI0CR2_RE_MASK | SCI0CR2_TE_MASK | SCI0CR2_RIE_MASK;
  return 0;
}

