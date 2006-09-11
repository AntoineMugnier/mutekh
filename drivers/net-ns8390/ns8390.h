/*
 *  GRUB  --  GRand Unified Bootloader
 *  Copyright (C) 2001,2002  Free Software Foundation, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/**************************************************************************

Author: Martin Renters
  Date: May/94

 This code is based heavily on David Greenman's if_ed.c driver

 Copyright (C) 1993-1994, David Greenman, Martin Renters.
  This software may be used, modified, copied, distributed, and sold, in
  both source and binary form provided that the above copyright and these
  terms are retained. Under no circumstances are the authors responsible for
  the proper functioning of this software, nor do the authors assume any
  responsibility for damages incurred with its use.

**************************************************************************/

#ifndef _8390_H_
#define _8390_H_

/*
 * Memory size constants.
 */

#define MEM_8192	32
#define MEM_16384	64
#define MEM_32768	128

/*
 * Offset of registers.
 */

#define NE_ASIC_OFFSET	0x10
#define NE_RESET	0x0F		/* Used to reset card */
#define NE_DATA		0x00		/* Used to read/write NIC mem */

/*
 * Commands and arguments.
 */

#define D8390_P0_COMMAND	0x00
#define D8390_P0_PSTART		0x01
#define D8390_P0_PSTOP		0x02
#define D8390_P0_BOUND		0x03
#define D8390_P0_TSR		0x04
#define	D8390_P0_TPSR		0x04
#define D8390_P0_TBCR0		0x05
#define D8390_P0_TBCR1		0x06
#define D8390_P0_ISR		0x07
#define D8390_P0_RSAR0		0x08
#define D8390_P0_RSAR1		0x09
#define D8390_P0_RBCR0		0x0A
#define D8390_P0_RBCR1		0x0B
#define D8390_P0_RSR		0x0C
#define D8390_P0_RCR		0x0C
#define D8390_P0_TCR		0x0D
#define D8390_P0_DCR		0x0E
#define D8390_P0_IMR		0x0F
#define D8390_P1_COMMAND	0x00
#define D8390_P1_PAR0		0x01
#define D8390_P1_PAR1		0x02
#define D8390_P1_PAR2		0x03
#define D8390_P1_PAR3		0x04
#define D8390_P1_PAR4		0x05
#define D8390_P1_PAR5		0x06
#define D8390_P1_CURR		0x07
#define D8390_P1_MAR0		0x08

#define D8390_COMMAND_PS0	0x0		/* Page 0 select */
#define D8390_COMMAND_PS1	0x40		/* Page 1 select */
#define D8390_COMMAND_PS2	0x80		/* Page 2 select */
#define	D8390_COMMAND_RD2	0x20		/* Remote DMA control */
#define D8390_COMMAND_RD1	0x10
#define D8390_COMMAND_RD0	0x08
#define D8390_COMMAND_TXP	0x04		/* transmit packet */
#define D8390_COMMAND_STA	0x02		/* start */
#define D8390_COMMAND_STP	0x01		/* stop */

#define D8390_RCR_MON		0x20		/* monitor mode */

#define D8390_DCR_FT1		0x40
#define D8390_DCR_LS		0x08		/* Loopback select */
#define D8390_DCR_WTS		0x01		/* Word transfer select */

#define D8390_ISR_PRX		0x01		/* successful recv */
#define D8390_ISR_PTX		0x02		/* successful xmit */
#define D8390_ISR_RXE		0x04		/* receive error */
#define D8390_ISR_TXE		0x08		/* transmit error */
#define D8390_ISR_OVW		0x10		/* Overflow */
#define D8390_ISR_CNT		0x20		/* Counter overflow */
#define D8390_ISR_RDC		0x40		/* Remote DMA complete */
#define D8390_ISR_RST		0x80		/* reset */

#define D8390_RSTAT_PRX		0x01		/* successful recv */
#define D8390_RSTAT_CRC		0x02		/* CRC error */
#define D8390_RSTAT_FAE		0x04		/* Frame alignment error */
#define D8390_RSTAT_OVER	0x08		/* FIFO overrun */

#define D8390_TXBUF_SIZE	6
#define D8390_RXBUF_END		32
#define D8390_PAGE_SIZE         256

/*
 * Each packet in the card's memory is prefixed by this structure.
 */

struct		net_ns8380_header_s
{
  uint8_t	status;
  uint8_t	next;
  uint16_t	size;
} __attribute__ ((packed));

/*
 * Function to access the hardware.
 */

void	net_ns8390_pio_read(struct net_ns8390_context_s		*pv,
			    uint_fast16_t			src,
			    uint8_t				*dst,
			    size_t				size);

void	net_ns8390_pio_write(struct net_ns8390_context_s	*pv,
			     uint8_t				*src,
			     uint_fast16_t			dst,
			     size_t				size);

void	net_ns8390_reset(struct net_ns8390_context_s		*pv);

error_t	net_ns8390_probe(struct net_ns8390_context_s	*pv,
			 uint_fast16_t			base);

size_t	net_ns8390_read(struct net_ns8390_context_s	*pv,
			uint8_t				**data);

void	net_ns8390_write(struct net_ns8390_context_s	*pv,
			 uint8_t			*data,
			 size_t				size);

#endif
