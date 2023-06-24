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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2023
*/

#include <mutek/printk.h>
#include <mutek/startup.h>

#include <hexo/types.h>
#include <hexo/cpu.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include "pl011.h"

static void pl011_printk_out_char(uint8_t c)
{
  /* wait tx fifo not full */
  while ((endian_le32(cpu_mem_read_32(CONFIG_MUTEK_PRINTK_ADDR + PL011_FR_ADDR)) & PL011_FR_TXFF))
    ;

  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + PL011_DR_ADDR, endian_le32((uint32_t)c));
}

static PRINTK_HANDLER(pl011_printk_out)
{
  for (size_t i = 0; i < len; i++)
  {
    if (str[i] == '\n')
      pl011_printk_out_char('\r');
    pl011_printk_out_char(str[i]);
  }
}

void pl011_printk_init(void)
{
  uint32_t div_f12_11 = (CONFIG_DRIVER_ARM_PL011_PRINTK_DIVISOR * 16 * 64 * 2);
  uint32_t div_f12_10 = (div_f12_11 + 1) / 2;

  uint32_t lcrh = 0;
  lcrh |= PL011_LCRH_FEN;
  lcrh |= PL011_LCRH_WLEN(PL011_LCRH_WLEN_8_BITS);
  cpu_mem_write_32(pv->addr + PL011_LCRH_ADDR, endian_le32(lcrh));
  cpu_mem_write_32(pv->addr + PL011_IBRD_ADDR, endian_le32(div_f12_10 >> 6));
  cpu_mem_write_32(pv->addr + PL011_FBRD_ADDR, endian_le32(div_f12_10 & 0x3f));
  cpu_mem_write_32(pv->addr + PL011_CR_ADDR, endian_le32(PL011_CR_TXE | PL011_CR_UARTEN));

  static struct printk_backend_s pl011_printk_backend;
  printk_register(&pl011_printk_backend, pl011_printk_out);
}

