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
#include <hexo/bit.h>

#include <mutek/printk.h>
#include <mutek/startup.h>

#include <arch/pic32/uart.h>
#include <arch/pic32/gpio.h>
#include <arch/pic32/clk.h>
#include <arch/pic32/devaddr.h>
#include <arch/pic32/freq.h>

#include <arch/pic32/pin.h>


static void printk_out_char(char c)
{
  while ((cpu_mem_read_32(CONFIG_MUTEK_PRINTK_ADDR + PIC32_UART_STATUS_ADDR)
           & PIC32_UART_STATUS_UTXBF))
    ;

  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + PIC32_UART_TX_ADDR, c);
}

static PRINTK_HANDLER(printk_out)
{
  size_t i;

  for (i = 0; i < len; i++)
  {
    if (str[i] == '\n')
      printk_out_char('\r');
    printk_out_char(str[i]);
  }
}

void pic32_uart_printk_init(void)
{
  // UART number, 0-based, unlike PIC32 Datasheet
  uint8_t nuart = (CONFIG_MUTEK_PRINTK_ADDR >> 9) & 0x7;
  uint32_t bank = CONFIG_DRIVER_PIC32_UART_PRINTK_PIN/16;
  uint32_t pin = CONFIG_DRIVER_PIC32_UART_PRINTK_PIN%16;
  uint32_t remap = nuart < 2 ? (nuart + 1) : (nuart - 1);
  uint32_t rate = ((PIC32_PB2CLK_FREQ / CONFIG_DRIVER_PIC32_UART_PRINTK_BAUDRATE) / 16) - 1;
  static struct printk_backend_s backend;
  
  cpu_mem_write_32(PIC32_GPIO_ADDR + PIC32_GPIO_TRIS_ADDR(bank) + PIC32_CLR_OFF,
                   bit(pin));
  cpu_mem_write_32(PIC32_GPIO_ADDR + PIC32_GPIO_RP_ADDR(CONFIG_DRIVER_PIC32_UART_PRINTK_PIN), remap);
  cpu_mem_write_32(PIC32_GPIO_ADDR + PIC32_GPIO_ANSEL_ADDR(bank) + PIC32_CLR_OFF,
                   bit(pin));

  /* configure baud rate. */
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + PIC32_UART_BAUD_ADDR, rate);

  /* Enable uart and transmitter */
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + PIC32_UART_MODE_ADDR, PIC32_UART_MODE_ON);
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + PIC32_UART_STATUS_ADDR, PIC32_UART_STATUS_UTXEN);

  printk_register(&backend, printk_out);
}

