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
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2012
*/

#include <hexo/types.h>
#include <hexo/cpu.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>
#include <mutek/printk.h>

#ifdef CONFIG_ATMEL_PRINTK_AVR32_SIM
static PRINTK_HANDLER(printk_out)
{
#ifdef CONFIG_ATMEL_PRINTK_AVR32_SIM
  asm volatile("mov r12, 1  \n"
               "mov r11, %0  \n"
               "mov r10, %1  \n"
               "mov r8, 4\n"
               "breakpoint   \n"
               :: "r" (str), "r" (len)
               : "r8", "r10", "r11", "r12"
               );
#endif
}

void atmel_printk_avr32sim_init()
{
  static struct printk_backend_s backend;
  printk_register(&backend, printk_out);
}
#endif

/*********************************************************************************/

#ifdef CONFIG_ATMEL_PRINTK_USART

#define AVR32_USART_THR                0x0000001c

#define AVR32_USART_CSR                0x00000014
#define AVR32_USART_CSR_TXRDY_MASK     0x00000002

#define AVR32_USART_CR                 0x00000000
#define AVR32_USART_CR_RSTTX_MASK      0x00000008
#define AVR32_USART_CR_TXEN_MASK       0x00000040

#define AVR32_USART_MR                 0x00000004
#define AVR32_USART_MR_CHRL_8          0x00000003
#define AVR32_USART_MR_CHRL_OFFSET     6
#define AVR32_USART_MR_PAR_NONE        0x00000004
#define AVR32_USART_MR_PAR_OFFSET      9
#define AVR32_USART_MR_NBSTOP_1        0x00000000
#define AVR32_USART_MR_NBSTOP_OFFSET   12

static PRINTK_HANDLER(printk_out)
{
  size_t i;

  for (i = 0; i < len; i++)
    {
      while (!(cpu_mem_read_32(CONFIG_MUTEK_PRINTK_ADDR + AVR32_USART_CSR) & AVR32_USART_CSR_TXRDY_MASK))
        ;
      cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + AVR32_USART_THR, str[i]);
    }
}

void atmel_printk_usart_init()
{
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + AVR32_USART_CR, AVR32_USART_CR_RSTTX_MASK);
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + AVR32_USART_MR,
                   (AVR32_USART_MR_CHRL_8 << AVR32_USART_MR_CHRL_OFFSET)     |
                   (AVR32_USART_MR_PAR_NONE << AVR32_USART_MR_PAR_OFFSET)    |
                   (AVR32_USART_MR_NBSTOP_1 << AVR32_USART_MR_NBSTOP_OFFSET));
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + AVR32_USART_CR, AVR32_USART_CR_TXEN_MASK);
  static struct printk_backend_s backend;
  printk_register(&backend, printk_out);
}
#endif

