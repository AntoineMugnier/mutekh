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

static void early_console_out_char(uintptr_t addr, uint8_t c)
{
  /* wait tx fifo not full */
  while ((endian_le32(cpu_mem_read_32(addr + 0x2c)) & 0x10))
    ;

  cpu_mem_write_32(addr + 0x30, endian_le32((uint32_t)c));
}

static PRINTF_OUTPUT_FUNC(early_console_out)
{
  uintptr_t addr = (uintptr_t)ctx;
  size_t i;

  /* enable TX if needed */
  if ((endian_le32(cpu_mem_read_32(addr + 0)) & 0x30) != 0x10)
    cpu_mem_write_32(addr + 0, endian_le32(0x110));

  for (i = 0; i < len; i++)
  {
    if (str[i] == '\n')
      early_console_out_char(addr, '\r');

    early_console_out_char(addr, str[i]);
  }

  /* wait tx active or tx fifo not empty */
  while ((endian_le32(cpu_mem_read_32(addr + 0x2c)) & 0x808) ^ 0x008)
    ;
}

void zynq_early_console_init()
{
  uintptr_t addr = CONFIG_ZYNQ_EARLY_CONSOLE_UART_ADDR;

  printk_set_output(early_console_out, (void*)addr);
}

