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

  Copyright (c) 2013 Jeremie Brunel <jeremie.brunel@telecom-paristech.fr>
  Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
  Copyright (c) 2011 Institut Telecom / Telecom ParisTech

*/

#include <mutek/printk.h>
#include <mutek/startup.h>

#include <hexo/types.h>
#include <hexo/cpu.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

extern volatile char toto;

static void printk_out_char(uint8_t c)
{
  while ((cpu_mem_read_32(CONFIG_MUTEK_PRINTK_ADDR + 0x18) & 0x20))
    ;

  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR, endian_le32((uint32_t)c));
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

  while (endian_le32(cpu_mem_read_32(CONFIG_MUTEK_PRINTK_ADDR + 0x18)) & 0x8)
    ;
}

void bcm283x_printk_init()
{
  /* before init disable uart */
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + 0x30, 0);

  /* set baudrate
     ibrd = uart_clk / (16 * baud_rate)
     fbrd = rnd((64 * (uart_clk % (16 * baud_rate)) / (16 * baud_rate)))
     uart_clk = 3000000
     baud_rate = 115200
     ibrd = 1
     fbrd = 40
   */
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + 0x24, 1);
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + 0x28, 40);

  /* set uart to be 8 bits, 1 stop bit, no parity, fifo enabled */
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + 0x2c, (3 << 5) | (1 << 4));

  /* enable uart */
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + 0x30, (1 << 8) | (1 << 9) | (1));

  static struct printk_backend_s backend;
  printk_register(&backend, printk_out);
}

