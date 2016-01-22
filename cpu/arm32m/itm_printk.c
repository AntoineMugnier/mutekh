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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2016
*/

#include <mutek/startup.h>

#include <hexo/types.h>
#include <hexo/endian.h>
#include <string.h>

#include <mutek/printk.h>

#include <hexo/iospace.h>

#include <cpu/arm32m/itm.h>
#include <cpu/arm32m/coredebug.h>

#include <assert.h>

static void arm32m_itm_putc(char c)
{
  while (!cpu_mem_read_32(ITM_STIM_ADDR(CONFIG_CPU_ARM32M_ITM_PRINTK_PORT)))
    ;

  cpu_mem_write_8(ITM_STIM_ADDR(CONFIG_CPU_ARM32M_ITM_PRINTK_PORT), c);
}

static PRINTK_HANDLER(itm_printk_out)
{
  uint32_t i;

  if (!(cpu_mem_read_32(ITM_TER_ADDR) & ITM_TER_STIMENA(CONFIG_CPU_ARM32M_ITM_PRINTK_PORT)))
    return;

  if (!(cpu_mem_read_32(COREDEBUG_DEMCR_ADDR) & COREDEBUG_DEMCR_TRACEENA))
    return;

  for (i = 0; i < len; ++i)
    arm32m_itm_putc(str[i]);
}

void arm32m_itm_printk_init(void)
{
  static struct printk_backend_s itm_printk_backend;

  printk_register(&itm_printk_backend, itm_printk_out);
}
