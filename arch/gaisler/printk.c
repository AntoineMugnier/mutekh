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

  Copyright (c) 2011 Alexandre Becoulet <alexandre.becoulet@telecom-paristech.fr>
  Copyright (c) 2011 Institut Telecom / Telecom ParisTech

*/

#include <mutek/printk.h>
#include <mutek/startup.h>

#include <hexo/types.h>
#include <hexo/cpu.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

static void printk_out_char(uintptr_t addr, uint8_t c)
{
  /* wait for transmit fifo empty */
  while (!(cpu_mem_read_32(addr + 4) & endian_be32(0x4)))
    ;

  cpu_mem_write_32(addr, endian_be32(c));

  /* wait for transmit register empty */
  while (!(cpu_mem_read_32(addr + 4) & endian_be32(0x2)))
    ;
}

static PRINTF_OUTPUT_FUNC(printk_out)
{
  uintptr_t addr = (uintptr_t)ctx;
  size_t i;

  for (i = 0; i < len; i++)
    {
      if (str[i] == '\n')
	printk_out_char(addr, '\r');

      printk_out_char(addr, str[i]);
    }
}

void gaisler_printk_init()
{
  uintptr_t addr = CONFIG_MUTEK_PRINTK_ADDR;

#ifndef CONFIG_GAISLER_PRINTK_DEBUG
  /* uart scaler FIXME */
  cpu_mem_write_32(addr + 12, endian_be32(CONFIG_GAISLER_PRINTK_SCALER));
  /* uart control */
  cpu_mem_write_32(addr + 8, endian_be32(0x3));
  /* clear uart status */
  cpu_mem_write_32(addr + 4, 0);
#endif

  printk_set_output(printk_out, (void*)addr);
}
