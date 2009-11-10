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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2009

*/


#include <hexo/types.h>

#include <device/i2c.h>
#include <device/icu.h>
#include <device/device.h>
#include <device/driver.h>

#include <hexo/iospace.h>
#include <mutek/mem_alloc.h>
#include <hexo/endian.h>
#include <mutek/printk.h>

#include <assert.h>

#include "i2c-twi6061a.h"
#include "i2c-twi6061a-private.h"

#define MCK 48000000

static REQ_HNDL(i2c_twi6061a_wait_txcomp)
{
	struct i2c_twi6061a_context_s *pv = dev->drv_pv;
	volatile struct twi6061a_reg_s *registers = (void*)dev->addr[0];
	struct dev_i2c_rq_s *rq = dev_i2c_queue_head(&pv->queue);

	dev_i2c_queue_remove(&pv->queue, rq);
	rq->callback(rq->pvdata, rq, 0);

	registers->TWI_IDR = TWI6061A_TXCOMP | TWI6061A_NACK;
	pv->handler = NULL;
	return 1;
}

static REQ_HNDL(i2c_twi6061a_read)
{
	struct i2c_twi6061a_context_s *pv = dev->drv_pv;
	volatile struct twi6061a_reg_s *registers = (void*)dev->addr[0];

	*pv->data = registers->TWI_RHR;
	pv->data++;
	pv->count--;
	if ( pv->count == 1 ) {
		registers->TWI_CR = TWI6061A_STOP;
	}
	if ( pv->count == 0 ) {
		registers->TWI_IER = TWI6061A_TXCOMP;
		registers->TWI_IDR = TWI6061A_RXRDY;
		pv->handler = i2c_twi6061a_wait_txcomp;
	}
	return 0;
}

static REQ_HNDL(i2c_twi6061a_write)
{
	struct i2c_twi6061a_context_s *pv = dev->drv_pv;
	volatile struct twi6061a_reg_s *registers = (void*)dev->addr[0];

	registers->TWI_THR = *pv->data;
	pv->data++;
	pv->count--;
	if ( pv->count == 0 ) {
		registers->TWI_IDR = TWI6061A_TXRDY;
		registers->TWI_IER = TWI6061A_TXCOMP;
		pv->handler = i2c_twi6061a_wait_txcomp;
	}
	return 0;
}

#if 0
static
void i2c_twi6061a_handle_req(struct device_s *dev, struct dev_i2c_rq_s *rq)
{
	struct i2c_twi6061a_context_s *pv = dev->drv_pv;
	volatile struct twi6061a_reg_s *registers = (void*)dev->addr[0];

	uint32_t mmr = (0x7f & rq->dev_addr) << 16;

	if ( rq->internal_address & DEV_I2C_IADDR_MASK ) {
		mmr |= ((rq->internal_address & DEV_I2C_IADDR_MASK) >> 24) << 8;
		registers->TWI_IADR = rq->internal_address & 0xffffff;
	}

	pv->data = rq->data;
	pv->count = rq->size;

	switch ( rq->type ) {
	case DEV_I2C_READ: {
		uint32_t cr = TWI6061A_START;
		uint32_t ier = TWI6061A_RXRDY | TWI6061A_NACK;

		registers->TWI_MMR = mmr | TWI6061A_MREAD;
		pv->handler = i2c_twi6061a_read;

		if ( pv->count == 1 )
			cr |= TWI6061A_STOP;

		registers->TWI_CR = cr;
		registers->TWI_IER = ier;

		break;
	}
	case DEV_I2C_WRITE:
		pv->handler = i2c_twi6061a_write;
		registers->TWI_MMR = mmr;
		registers->TWI_CR = TWI6061A_START;
		registers->TWI_IER = TWI6061A_TXRDY | TWI6061A_NACK;
		i2c_twi6061a_write(dev);
		break;
	}

//	printk("i2c st, st: %x, co: %d\n", registers->TWI_SR, pv->count);
}
#else
static
void i2c_twi6061a_handle_req(struct device_s *dev, struct dev_i2c_rq_s *rq)
{
	struct i2c_twi6061a_context_s *pv = dev->drv_pv;
	volatile struct twi6061a_reg_s *registers = (void*)dev->addr[0];

	uint32_t mmr = (0x7f & rq->dev_addr) << 16;

	if ( rq->internal_address & DEV_I2C_IADDR_MASK ) {
		mmr |= ((rq->internal_address & DEV_I2C_IADDR_MASK) >> 24) << 8;
	}

	pv->data = rq->data;
	pv->count = rq->size;

	switch ( rq->type ) {
	case DEV_I2C_READ: {
		uint32_t cr = TWI6061A_START;
		uint32_t ier = TWI6061A_RXRDY | TWI6061A_NACK;

		registers->TWI_MMR = mmr | TWI6061A_MREAD;
		registers->TWI_IADR = rq->internal_address & 0xffffff;
		pv->handler = i2c_twi6061a_read;

		if ( pv->count == 1 )
			cr |= TWI6061A_STOP;

		registers->TWI_CR = cr;

		while ( pv->count ) {
			while( !(registers->TWI_SR & TWI6061A_RXRDY) )
				if ( registers->TWI_SR & (TWI6061A_NACK|TWI6061A_TXCOMP|TWI6061A_UNRE|TWI6061A_OVRE) ) {
					rq->callback(rq->pvdata, rq, registers->TWI_SR & (TWI6061A_NACK|TWI6061A_TXCOMP|TWI6061A_UNRE|TWI6061A_OVRE));
					dev_i2c_queue_remove(&pv->queue, rq);
					return;
				}
			*pv->data = registers->TWI_RHR;
			pv->data++;
			pv->count--;
			if ( pv->count == 1 ) {
				registers->TWI_CR = TWI6061A_STOP;
			}
			if ( pv->count == 0 ) {
				break;
			}
		}			
		while( !(registers->TWI_SR & TWI6061A_TXCOMP) )
			if ( registers->TWI_SR & (TWI6061A_NACK|TWI6061A_UNRE|TWI6061A_OVRE) ) {
				rq->callback(rq->pvdata, rq, (registers->TWI_SR & (TWI6061A_NACK|TWI6061A_UNRE|TWI6061A_OVRE)) | 0x80 );
				dev_i2c_queue_remove(&pv->queue, rq);
				return;
			}

		rq->callback(rq->pvdata, rq, 0);
		dev_i2c_queue_remove(&pv->queue, rq);

		break;
	}
	case DEV_I2C_WRITE:
		pv->handler = i2c_twi6061a_write;
		registers->TWI_MMR = mmr;
		registers->TWI_CR = TWI6061A_START;
		registers->TWI_IER = TWI6061A_TXRDY | TWI6061A_NACK;
		i2c_twi6061a_write(dev);
		break;
	}

//	printk("i2c st, st: %x, co: %d\n", registers->TWI_SR, pv->count);
}
#endif

DEVI2C_REQUEST(i2c_twi6061a_request)
{
    struct i2c_twi6061a_context_s *pv = dev->drv_pv;

	CPU_INTERRUPT_SAVESTATE_DISABLE;

	bool_t queue_empty = !dev_i2c_queue_head(&pv->queue);

	dev_i2c_queue_pushback(&pv->queue, rq);

	if ( ! queue_empty )
		return;
	
	i2c_twi6061a_handle_req(dev, rq);
	
	CPU_INTERRUPT_RESTORESTATE;
}

DEV_IRQ(i2c_twi6061a_irq)
{
    struct i2c_twi6061a_context_s *pv = dev->drv_pv;
	struct dev_i2c_rq_s *rq = dev_i2c_queue_head(&pv->queue);
	volatile struct twi6061a_reg_s *registers = (void*)dev->addr[0];

	assert(rq);
	assert(pv->handler);

//	printk("i2c irq, st: %x, co: %d\n", registers->TWI_SR, pv->count);

	if ( registers->TWI_SR & TWI6061A_NACK ) {
		dev_i2c_queue_remove(&pv->queue, rq);
		rq->callback(rq->pvdata, rq, EIO);

		registers->TWI_IDR = TWI6061A_TXCOMP | TWI6061A_RXRDY | TWI6061A_TXRDY | TWI6061A_NACK;
		pv->handler = NULL;

		rq = dev_i2c_queue_head(&pv->queue);
		if ( rq )
			i2c_twi6061a_handle_req(dev, rq);
	} else if ( pv->handler(dev) ) {
		rq = dev_i2c_queue_head(&pv->queue);
		if ( rq )
			i2c_twi6061a_handle_req(dev, rq);
	}

	return 0;
}

DEVI2C_SET_BAUDRATE(i2c_twi6061a_set_baudrate)
{
	volatile struct twi6061a_reg_s *registers = (void*)dev->addr[0];

	// We define half periods, in MCK cycles
	br = MCK/2/br;

	if ( !br )
		return EINVAL;

	int_fast8_t exp = __builtin_ffs(br)-7;

	if ( exp < 0 )
		exp = 0;

	if ( exp > 7 )
		return EINVAL;

	br >>= 7;

	assert( (br & ~0xff) == 0 );

	registers->TWI_CWGR = (exp << 16) | (br << 8) |  br;

	return MCK/2/((br<<exp) + 3);
}

const struct driver_s   i2c_twi6061a_drv =
{
    .class      = device_class_i2c,
    .f_init     = i2c_twi6061a_init,
    .f_cleanup  = i2c_twi6061a_cleanup,
    .f_irq      = i2c_twi6061a_irq,
	.f.i2c = {
		.f_request = i2c_twi6061a_request,
		.f_set_baudrate = i2c_twi6061a_set_baudrate,
	},
};

DEV_INIT(i2c_twi6061a_init)
{
	struct i2c_twi6061a_context_s   *pv;
	volatile struct twi6061a_reg_s *registers = (void*)dev->addr[0];

	dev->drv = &i2c_twi6061a_drv;

	/* allocate private driver data */
	pv = mem_alloc(sizeof(*pv), (mem_scope_sys));

	if (!pv)
		return -1;

	dev->drv_pv = pv;

	registers->TWI_CR = TWI6061A_MSDIS;
	{ uint_fast8_t i; for (i=0; i<200; ++i) asm volatile("nop"); }
	registers->TWI_CR = TWI6061A_SWRST;
	{ uint_fast8_t i; for (i=0; i<200; ++i) asm volatile("nop"); }

	registers->TWI_CWGR = 0
		// MCK/(2^7) == 187KHz
		| (7 << 16)
		;

	registers->TWI_CR = TWI6061A_MSEN;

	dev_i2c_queue_init(&pv->queue);

	dev_icu_sethndl(dev->icudev, dev->irq, i2c_twi6061a_irq, dev);
	dev_icu_set_flags(dev->icudev, dev->irq, 0x2);
	dev_icu_enable(dev->icudev, dev->irq, 1);

	return 0;
}

DEV_CLEANUP(i2c_twi6061a_cleanup)
{
    struct i2c_twi6061a_context_s *pv = dev->drv_pv;

	DEV_ICU_UNBIND(dev->icudev, dev, dev->irq);

    mem_free(pv);
}

