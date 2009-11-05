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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009

*/

#include "uart-us6089c.h"

#include "uart-us6089c-private.h"

#include <device/icu.h>
#include <hexo/types.h>
#include <device/device.h>
#include <device/driver.h>
#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/interrupt.h>

#define BR    38400 			/* Baud Rate */
#define MCK  47923200
#define BRD  (MCK/16/BR)	/* Baud Rate Divisor */

static void try_send(struct device_s *dev, bool_t continuous)
{
	struct uart_us6089c_context_s *pv = dev->drv_pv;
	volatile struct us6089c_reg_s *registers = (void*)dev->addr[0];
	struct dev_char_rq_s *txrq = dev_char_queue_head(&pv->write_q);

	while ((registers->US_CSR & US6089C_TXRDY) && txrq)
	{
		assert( txrq->size );
		registers->US_THR = (uint32_t)*(txrq->data);
			
		++(txrq->data);
		--(txrq->size);

		if (txrq->callback(dev, txrq, 1) || txrq->size == 0)
		{
			dev_char_queue_remove(&pv->write_q, txrq);

			// Take the next request
			txrq = dev_char_queue_head(&pv->write_q);
			if (!txrq)
			{
				registers->US_IDR = US6089C_TXRDY;
				break;
			}
		}
		if ( !continuous )
			break;
	}
}

static void try_recv(struct device_s *dev, bool_t continuous)
{
	struct uart_us6089c_context_s *pv = dev->drv_pv;
	volatile struct us6089c_reg_s *registers = (void*)dev->addr[0];
	struct dev_char_rq_s *rxrq = dev_char_queue_head(&pv->read_q);

	while ((registers->US_CSR & US6089C_RXRDY) && rxrq)
	{
		assert( rxrq->size );
		uint32_t d = registers->US_RHR;

		*rxrq->data = d;

		++(rxrq->data);
		--(rxrq->size);

		if (rxrq->callback(dev, rxrq, 1) || rxrq->size == 0)
		{
			dev_char_queue_remove(&pv->read_q, rxrq);

			// Take the next request
			rxrq = dev_char_queue_head(&pv->read_q);
			if ( !rxrq )
			{
				registers->US_IDR = US6089C_RXRDY;
				break;
			}
		}
		if ( !continuous )
			break;
	}
	if (!rxrq && (registers->US_CSR & US6089C_RXRDY)) {
		volatile uint32_t d = registers->US_RHR;
		(void)d;
	}
}

DEVCHAR_REQUEST(uart_us6089c_request)
{
	struct uart_us6089c_context_s	*pv = dev->drv_pv;
	volatile struct us6089c_reg_s *registers = (void*)dev->addr[0];

	if (rq->size == 0) {
		if (rq->callback)
			rq->callback(dev, rq, 0);
		return;
	}

	LOCK_SPIN_IRQ(&dev->lock);

	switch (rq->type)
    {
    case DEV_CHAR_READ:
		dev_char_queue_pushback(&pv->read_q, rq);
		registers->US_IER = US6089C_RXRDY;
		try_recv(dev, 0);
		break;

    case DEV_CHAR_WRITE:
		dev_char_queue_pushback(&pv->write_q, rq);
		registers->US_IER = US6089C_TXRDY;
		try_send(dev, 0);
		break;
    }

	LOCK_RELEASE_IRQ(&dev->lock);
}

/* 
 * device close operation
 */

DEV_CLEANUP(uart_us6089c_cleanup)
{
	struct uart_us6089c_context_s	*pv = dev->drv_pv;

	DEV_ICU_UNBIND(dev->icudev, dev, dev->irq);

	dev_char_queue_destroy(&pv->write_q);
	dev_char_queue_destroy(&pv->read_q);

	mem_free(pv);
}

/*
 * device irq
 */

DEV_IRQ(uart_us6089c_irq)
{
	lock_spin(&dev->lock);

	try_send(dev, 1);
	try_recv(dev, 1);

	lock_release(&dev->lock);

	return 1;
}

/* 
 * device open operation
 */

const struct driver_s	uart_us6089c_drv =
{
    .class      = device_class_char,
    .f_init     = uart_us6089c_init,
    .f_cleanup  = uart_us6089c_cleanup,
    .f_irq      = uart_us6089c_irq,
    .f.chr = {
        .f_request = uart_us6089c_request,
    }
};

DEV_INIT(uart_us6089c_init)
{
	struct uart_us6089c_context_s	*pv;
	volatile struct us6089c_reg_s *registers = (void*)dev->addr[0];

	dev->drv = &uart_us6089c_drv;

	/* alocate private driver data */
	pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

	if (!pv)
		return -1;

	// set up the USART0 register
	registers->US_CR =
		US6089C_RSTRX
		| US6089C_RSTTX
		| US6089C_RXDIS
		| US6089C_TXDIS;

	// set char size
	registers->US_MR = 0
		| US6089C_PAR_NONE
		| (0x3 << 6)
		| 2 // Hardware handshaking
		;

	// no interupt
	registers->US_IDR = 0xFFFF;

	// configure to 9600 bauds
	registers->US_BRGR = BRD;
	registers->US_RTOR = 0;
	registers->US_TTGR = 0;
	registers->US_FIDI = 0;
	registers->US_IF = 0;

	dev_icu_sethndl(dev->icudev, dev->irq, uart_us6089c_irq, dev);
	dev_icu_set_flags(dev->icudev, dev->irq, 0x4);
	dev_icu_enable(dev->icudev, dev->irq, 1);

	// enable receiver and transmitter
	registers->US_CR = US6089C_RXEN | US6089C_TXEN;

	dev->drv_pv = pv;

	dev_char_queue_init(&pv->read_q);
	dev_char_queue_init(&pv->write_q);

	return 0;
}

