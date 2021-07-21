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

#include <cpu/arm32m/etm.h>
#include <cpu/arm32m/itm.h>
#include <cpu/arm32m/tpiu.h>
#include <cpu/arm32m/dwt.h>
#include <cpu/arm32m/coredebug.h>

#include <assert.h>

static int arm32m_itm_putc(int chan, char c)
{
  uint32_t timeout = 1000;

  if (!(cpu_mem_read_32(ITM_TCR_ADDR) & ITM_TCR_ITMENA))
    return 1;

  if (!(cpu_mem_read_32(ITM_TER_ADDR) & ITM_TER_STIMENA(chan)))
    return 1;

  while (cpu_mem_read_32(ITM_STIM_ADDR(chan)) == 0)
    if (!--timeout)
      return 1;

  cpu_mem_write_8(ITM_STIM_ADDR(chan), c);

  return 0;
}

static PRINTK_HANDLER(itm_printk_out)
{
  size_t i;

  for (i = 0; i < len; i++)
    {
      if (str[i] == '\n' && arm32m_itm_putc(CONFIG_CPU_ARM32M_ITM_PRINTK_PORT, '\r'))
        break;

      if (arm32m_itm_putc(CONFIG_CPU_ARM32M_ITM_PRINTK_PORT, str[i]))
        break;
    }
}

void arm32m_itm_printk_init(void)
{
  static struct printk_backend_s itm_printk_backend;

  cpu_mem_write_32(ITM_LAR_ADDR, ITM_LAR_ACCESS_ENABLE);

#if defined(CONFIG_CPU_ARM32M_ITM)
  cpu_mem_write_32(ITM_TCR_ADDR, 0
# if defined(CONFIG_CPU_ARM32M_DWT)
    | ITM_TCR_DWTENA // Mandatory for sync
# endif
    | ITM_TCR_SWOENA
    | ITM_TCR_ITMENA
# if defined(CONFIG_CPU_ARM32M_DWT_SYNC)
    | ITM_TCR_SYNCENA // Mandatory for sync
# endif
    | ITM_TCR_TSENA
# if defined(CONFIG_CPU_ARM32M_TRACE_FORMATTING)
    | ITM_TCR_TRACEBUSID(CONFIG_CPU_ARM32M_ITM_BUSID)
# endif
    //| ITM_TCR_GTSFREQ(ON_EMPTY)
    );
#endif
  cpu_mem_write_32(ITM_TER_ADDR, 1 << CONFIG_CPU_ARM32M_ITM_PRINTK_PORT);

  printk_register(&itm_printk_backend, itm_printk_out);
}
