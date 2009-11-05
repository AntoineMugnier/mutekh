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


#ifndef UART_US6089C_PRIVATE_H_
#define UART_US6089C_PRIVATE_H_

#include <hexo/types.h>
#include <device/device.h>

#include <hexo/gpct_platform_hexo.h>

/**************************************************************/

struct uart_us6089c_context_s
{
  dev_char_queue_root_t		read_q;
  dev_char_queue_root_t		write_q;
};


struct us6089c_reg_s {
	uint32_t	 US_CR; 	// Control Register
	uint32_t	 US_MR; 	// Mode Register
	uint32_t	 US_IER; 	// Interrupt Enable Register
	uint32_t	 US_IDR; 	// Interrupt Disable Register
	uint32_t	 US_IMR; 	// Interrupt Mask Register
	uint32_t	 US_CSR; 	// Channel Status Register
	uint32_t	 US_RHR; 	// Receiver Holding Register
	uint32_t	 US_THR; 	// Transmitter Holding Register
	uint32_t	 US_BRGR; 	// Baud Rate Generator Register
	uint32_t	 US_RTOR; 	// Receiver Time-out Register
	uint32_t	 US_TTGR; 	// Transmitter Time-guard Register
	uint32_t	 Reserved0[5]; 	// 
	uint32_t	 US_FIDI; 	// FI_DI_Ratio Register
	uint32_t	 US_NER; 	// Nb Errors Register
	uint32_t	 Reserved1[1]; 	// 
	uint32_t	 US_IF; 	// IRDA_FILTER Register
	uint32_t	 Reserved2[44]; 	// 
	uint32_t	 US_RPR; 	// Receive Pointer Register
	uint32_t	 US_RCR; 	// Receive Counter Register
	uint32_t	 US_TPR; 	// Transmit Pointer Register
	uint32_t	 US_TCR; 	// Transmit Counter Register
	uint32_t	 US_RNPR; 	// Receive Next Pointer Register
	uint32_t	 US_RNCR; 	// Receive Next Counter Register
	uint32_t	 US_TNPR; 	// Transmit Next Pointer Register
	uint32_t	 US_TNCR; 	// Transmit Next Counter Register
	uint32_t	 US_PTCR; 	// PDC Transfer Control Register
	uint32_t	 US_PTSR; 	// PDC Transfer Status Register
};

#define US6089C_RXRDY        ((uint32_t)1 <<  0)
#define US6089C_TXRDY        ((uint32_t)1 <<  1)
#define US6089C_ENDRX        ((uint32_t)1 <<  3)
#define US6089C_ENDTX        ((uint32_t)1 <<  4)
#define US6089C_OVRE         ((uint32_t)1 <<  5)
#define US6089C_FRAME        ((uint32_t)1 <<  6)
#define US6089C_PARE         ((uint32_t)1 <<  7)
#define US6089C_TXEMPTY      ((uint32_t)1 <<  9)
#define US6089C_TXBUFE       ((uint32_t)1 << 11)
#define US6089C_RXBUFF       ((uint32_t)1 << 12)
#define US6089C_COMM_TX      ((uint32_t)1 << 30)
#define US6089C_COMM_RX      ((uint32_t)1 << 31)

#define US6089C_RSTRX        ((uint32_t)1 <<  2)
#define US6089C_RSTTX        ((uint32_t)1 <<  3)
#define US6089C_RXEN         ((uint32_t)1 <<  4)
#define US6089C_RXDIS        ((uint32_t)1 <<  5)
#define US6089C_TXEN         ((uint32_t)1 <<  6)
#define US6089C_TXDIS        ((uint32_t)1 <<  7)
#define US6089C_RSTSTA       ((uint32_t)1 <<  8)

#define US6089C_PAR_EVEN       ((uint32_t)0 <<  9)
#define US6089C_PAR_ODD        ((uint32_t)1 <<  9)
#define US6089C_PAR_SPACE      ((uint32_t)2 <<  9)
#define US6089C_PAR_MARK       ((uint32_t)3 <<  9)
#define US6089C_PAR_NONE       ((uint32_t)4 <<  9)
#define US6089C_PAR_MULTI_DROP ((uint32_t)6 <<  9)

#endif

