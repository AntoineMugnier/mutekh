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

static void printk_out_char(uint8_t c)
{
  /* wait tx fifo not full */
  while ((endian_le32(cpu_mem_read_32(CONFIG_MUTEK_PRINTK_ADDR + 0x2c)) & 0x10))
    ;

  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + 0x30, endian_le32((uint32_t)c));
}

static PRINTK_HANDLER(printk_out)
{
  size_t i;

  /* enable TX if needed */
  if ((endian_le32(cpu_mem_read_32(CONFIG_MUTEK_PRINTK_ADDR + 0)) & 0x30) != 0x10)
    cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + 0, endian_le32(0x110));

  for (i = 0; i < len; i++)
  {
    if (str[i] == '\n')
      printk_out_char('\r');

    printk_out_char(str[i]);
  }

  /* wait tx active or tx fifo not empty */
  while ((endian_le32(cpu_mem_read_32(CONFIG_MUTEK_PRINTK_ADDR + 0x2c)) & 0x808) ^ 0x008)
    ;
}

void zynq_printk_init()
{
  struct printk_backend_s backend;
  printk_register(&backend, printk_out);
}

