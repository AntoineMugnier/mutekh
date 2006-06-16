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


#include <mutek/types.h>
#include <mutek/device.h>
#include <mutek/iospace.h>
#include <mutek/alloc.h>
#include <mutek/lock.h>
#include <mutek/interrupt.h>

#include "uart-8250.h"

#include "uart-8250-private.h"

#include "8250.h"

/**************************************************************/

/* 
 * device read operation
 */

DEVCHAR_READ(uart_8250_read)
{
  struct uart_8250_context_s	*pv = dev->drv_pv;
  size_t			res;

  lock_spin_irq(&pv->lock);

  res = tty_read_fifo_poplist(&pv->read_fifo, data, size);

  lock_release_irq(&pv->lock);

  return res;
}

/* 
 * device write operation
 */

DEVCHAR_WRITE(uart_8250_write)
{
  struct uart_8250_context_s	*pv = dev->drv_pv;
  uint_fast16_t			i;

  lock_spin_irq(&pv->lock);

  for (i = 0; i < size; i++)
    /* process each char */
    {
      while (!(cpu_io_read_8(dev->addr[0] + UART_8250_LSR) & UART_8250_LSR_TXEMPTY))
	;

      cpu_io_write_8(dev->addr[0] + UART_8250_THR, data[i]);
    }

  lock_release_irq(&pv->lock);

  return i;
}

/* 
 * device close operation
 */

DEV_CLEANUP(uart_8250_cleanup)
{
  struct uart_8250_context_s	*pv = dev->drv_pv;

  lock_destroy(&pv->lock);

  mem_free(pv);
}

/*
 * IRQ handler
 */

DEV_IRQ(uart_8250_irq)
{
  struct uart_8250_context_s*pv = dev->drv_pv;
  __bool_t			res = 0;

  lock_spin(&pv->lock);

  lock_release(&pv->lock);

  return res;
}

/* 
 * device open operation
 */

DEV_INIT(uart_8250_init)
{
  struct uart_8250_context_s	*pv;

#ifndef CONFIG_STATIC_DRIVERS
  dev->f_cleanup	= uart_8250_cleanup;
  dev->f_irq		= uart_8250_irq;
  dev->chr.f_read	= uart_8250_read;
  dev->chr.f_write	= uart_8250_write;
#endif

  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  lock_init(&pv->lock);

  dev->drv_pv = pv;

  /* init tty input fifo */
  tty_read_fifo_init(&pv->read_fifo);

  pv->line_mode = UART_8250_LCR_8BITS | UART_8250_LCR_PARNO | UART_8250_LCR_1STOP;
  pv->line_speed = 0x000c;

  cpu_io_write_8(dev->addr[0] + UART_8250_FCR, UART_8250_FCR_FIFO);

  cpu_io_write_8(dev->addr[0] + UART_8250_MCR, 0);

  cpu_io_write_8(dev->addr[0] + UART_8250_LCR, UART_8250_LCR_DLAB);
  cpu_io_write_8(dev->addr[0] + UART_8250_DLL, pv->line_speed & 0xff);
  cpu_io_write_8(dev->addr[0] + UART_8250_DLM, pv->line_speed >> 8);

  cpu_io_write_8(dev->addr[0] + UART_8250_LCR, pv->line_mode);

  cpu_io_write_8(dev->addr[0] + UART_8250_IER, 0);

  return 0;
}

