/*
  This file is part of MutekH.

  MutekH is free software; you can redistribute it and/or modify it
  under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation; version 2.1 of the
  License.
    
  MutekH is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
    
  You should have received a copy of the GNU Lesser General Public
  License along with MutekH; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
  02110-1301 USA

  Copyright Nicolas Pouillon, <nipo@ssji.net>, 2009
*/

#include <hexo/types.h>
#include <hexo/cpu.h>
#include <hexo/iospace.h>
#include <mutek/printk.h>
#include <arch/lm3s/lm3s8962.h>

static PRINTF_OUTPUT_FUNC(early_console_out)
{
	uintptr_t out = (uintptr_t)ctx;
	size_t i;
	for ( i=0; i<len; ++i ){
          while(LM3S_BASE_UART0->UART_FR & (1<<5)); // spin if fifo full
          LM3S_BASE_UART0->UART_DR = str[i];
          //cpu_mem_write_32(out, str[i]);
        }
}

void simple_early_console()
{
  // 115200 bauds for 50MHz operating frequency
  //LM3S_BASE_UART0->UART_IBRDR = 27;
  //LM3S_BASE_UART0->UART_FBRDR = 9;

  // 115200 bauds for 12MHz operating frequency
  LM3S_BASE_UART0->UART_IBRDR = 6;
  LM3S_BASE_UART0->UART_FBRDR = 33;

  LM3S_BASE_UART0->UART_LCRHR = 0x00000060;
  LM3S_BASE_UART0->UART_CTLR  = (1<<8) |
                                   (1<<0);
  LM3S_BASE_UART0->UART_DR = 'O';
  printk_set_output(early_console_out, (void*)(LM3S_BASE_UART0->UART_DR));
}
 
