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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2015
*/

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <mutek/printk.h>

#include <arch/pic32/uart.h>
#include <arch/pic32/gpio.h>
#include <arch/pic32/clk.h>
#include <arch/pic32/devaddr.h>
#include <arch/pic32/freq.h>


static void early_console_out_char(char c)
{
  while ((cpu_mem_read_32(CONFIG_MUTEK_PRINTK_ADDR + PIC32_UART_STATUS_ADDR)
           & PIC32_UART_STATUS_UTXBF))
    ;

  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + PIC32_UART_TX_ADDR, c);
}

static PRINTF_OUTPUT_FUNC(early_console_out)
{
  uint_fast8_t i;

  for (i = 0; i < len; i++)
  {
    if (str[i] == '\n')
      early_console_out_char('\r');
    early_console_out_char(str[i]);
  }
}

void pic32_uart_printk_init()
{
  uint32_t x = 0;

  uint8_t nuart = (CONFIG_MUTEK_PRINTK_ADDR >> 9) & 0x7;
  uint32_t bank = CONFIG_DRIVER_PIC32_UART_PRINTK_PIN/16;
  uint32_t pin = CONFIG_DRIVER_PIC32_UART_PRINTK_PIN%16;

  /* Configure TX pin */

  switch (nuart)
    {
    case 0:    /* uart1 */
    case 2:    /* uart3 */
      x = 1;
      break;
    case 1:    /* uart2 */
    case 3:    /* uart4 */
      x = 2;
      break;
    case 4:    /* uart5 */
      x = 3;
      break;
    case 5:    /* uart6 */
      x = 4;
      break;
    default:
      return;
    }

  cpu_mem_write_32(PIC32_GPIO_ADDR + PIC32_GPIO_TRIS_ADDR(bank), PIC32_GPIO_TRIS_DIR(pin, OUTPUT));
  cpu_mem_write_32(PIC32_GPIO_ADDR + PIC32_GPIO_RP_ADDR(CONFIG_DRIVER_PIC32_UART_PRINTK_PIN), x);
  cpu_mem_write_32(PIC32_GPIO_ADDR + PIC32_GPIO_ANSEL_ADDR(bank), 0);

  /* configure baud rate. */
  x = ((PIC32_PB2CLK_FREQ/CONFIG_DRIVER_PIC32_UART_PRINTK_BAUDRATE)/16) - 1;

  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + PIC32_UART_BAUD_ADDR, x);

  /* Enable uart and transmitter */
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + PIC32_UART_MODE_ADDR, PIC32_UART_MODE_ON);
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + PIC32_UART_STATUS_ADDR, PIC32_UART_STATUS_UTXEN);


  printk_set_output(early_console_out, NULL);
}

