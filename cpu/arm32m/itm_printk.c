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
#include <cpu/arm32m/tpiu.h>
#include <cpu/arm32m/dwt.h>
#include <cpu/arm32m/etm.h>
#include <cpu/arm32m/coredebug.h>

#include <assert.h>

static void arm32m_itm_putc(char c)
{
  while (!cpu_mem_read_32(ITM_STIM_ADDR(CONFIG_CPU_ARM32M_ITM_PRINTK_PORT)))
    ;
  
  cpu_mem_write_8(ITM_STIM_ADDR(CONFIG_CPU_ARM32M_ITM_PRINTK_PORT), c);
}

static PRINTF_OUTPUT_FUNC(itm_printk_out)
{
  uint32_t i;

  assert((cpu_mem_read_32(ITM_TER_ADDR) & ITM_TER_STIMENA(CONFIG_CPU_ARM32M_ITM_PRINTK_PORT))
         && (cpu_mem_read_32(COREDEBUG_DEMCR_ADDR) & COREDEBUG_DEMCR_TRACEENA));

  for (i = 0; i < len; ++i) {
    if (str[i] == '\n')
      arm32m_itm_putc('\r');
    arm32m_itm_putc(str[i]);
  }
}

void arm32m_itm_printk_init(void)
{
  cpu_mem_write_32(ITM_TER_ADDR,
                   cpu_mem_read_32(ITM_TER_ADDR)
                   | ITM_TER_STIMENA(CONFIG_CPU_ARM32M_ITM_PRINTK_PORT));

  printk_set_output(itm_printk_out, NULL);
}
