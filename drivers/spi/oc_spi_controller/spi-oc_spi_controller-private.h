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

    Copyright (c) Eric Guthmuller 2010

*/

#ifndef SPI_OC_SPI_CONTROLLER_PRIVATE_H_
#define SPI_OC_SPI_CONTROLLER_PRIVATE_H_

#include <hexo/types.h>

#define CMD_HANDLER(x) void x(struct device_s *dev)
typedef CMD_HANDLER(cmd_handler_f);

struct spi_oc_spi_controller_context_s
{
	dev_spi_queue_root_t queue;
	uint32_t cs_bits;
	cmd_handler_f *tx_handler;
	cmd_handler_f *rx_handler;
	size_t cur_cmd;
	devspi_wait_value_callback_t *wait_cb;
	size_t count;
	uint_fast8_t increment;
        uint_fast8_t *bits_per_word;
        uint_fast16_t *dividers;
        uint_fast8_t *modes;
	bool_t *keep_cs;
	uint_fast16_t constant;

	uintptr_t tx_ptr;
	uintptr_t rx_ptr;

	uint16_t pad_byte;

	uint_fast8_t lun_count;

	bool_t abort;
};

#define SPI_OC_RX(x)  (0x4*x)
#define SPI_OC_TX(x)  (0x4*x)
#define SPI_OC_CTRL    0x10
#define SPI_OC_DIVIDER 0x14
#define SPI_OC_SS      0x18

#define SPI_OC_CHAR_LEN      ((uint32_t)0x7F << 0)
#define SPI_OC_GO_BSY        ((uint32_t)1 << 8)
#define SPI_OC_RX_NEG        ((uint32_t)1 << 9)
#define SPI_OC_TX_NEG        ((uint32_t)1 << 10)
#define SPI_OC_LSB           ((uint32_t)1 << 11)
#define SPI_OC_IE            ((uint32_t)1 << 12)
#define SPI_OC_ASS           ((uint32_t)1 << 13)

#define SPI_OC_DIVIDER_M     ((uint32_t)0xFFFF << 0)
#define SPI_OC_SS_M          ((uint32_t)0xFF << 0)

#endif
