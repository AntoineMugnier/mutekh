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

#ifndef I2C_TWI6061A_PRIVATE_H_
#define I2C_TWI6061A_PRIVATE_H_

#include <hexo/types.h>

#define REQ_HNDL(c) bool_t (c)(struct device_s *dev)
typedef REQ_HNDL(req_handler_t);

struct i2c_twi6061a_context_s
{
	dev_i2c_queue_root_t queue;
	req_handler_t *handler;
	uint8_t *data;
	size_t count;
};

struct twi6061a_reg_s {
	uint32_t	 TWI_CR;
	uint32_t	 TWI_MMR;
	uint32_t	 Reserved0[1];
	uint32_t	 TWI_IADR;
	uint32_t	 TWI_CWGR;
	uint32_t	 Reserved1[3];
	uint32_t	 TWI_SR;
	uint32_t	 TWI_IER;
	uint32_t	 TWI_IDR;
	uint32_t	 TWI_IMR;
	uint32_t	 TWI_RHR;
	uint32_t	 TWI_THR;
};

#define TWI6061A_START       ((uint32_t)1 <<  0)
#define TWI6061A_STOP        ((uint32_t)1 <<  1)
#define TWI6061A_MSEN        ((uint32_t)1 <<  2)
#define TWI6061A_MSDIS       ((uint32_t)1 <<  3)
#define TWI6061A_SWRST       ((uint32_t)1 <<  7)

#define TWI6061A_TXCOMP      ((uint32_t)1 <<  0)
#define TWI6061A_RXRDY       ((uint32_t)1 <<  1)
#define TWI6061A_TXRDY       ((uint32_t)1 <<  2)
#define TWI6061A_OVRE        ((uint32_t)1 <<  6)
#define TWI6061A_UNRE        ((uint32_t)1 <<  7)
#define TWI6061A_NACK        ((uint32_t)1 <<  8)

#define TWI6061A_MREAD       ((uint32_t)1 << 12)

#endif
