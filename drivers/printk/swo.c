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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#include <mutek/printk.h>
#include <hexo/iospace.h>

#if 0

#define DEMCR     ((uintptr_t)0xE000EDFC)
#define TRCENA       0x01000000

#define ITM_STIM ((uintptr_t)0xE0000000)

#define ITM_ENA  ((uintptr_t)0xE0000E00)

#define ITM_TCR  ((uintptr_t)0xE0000E80)
#define ITM_TCR_SWOENA  0x10
#define ITM_TCR_ITMENA  0x01
#define ITM_TCR_SYNCENA  0x04

#define ITM_LOCK ((uintptr_t)0xE0000FB0)
#define ITM_LOCK_MAGIC 0xC5ACCE55

static */
void swo_putc(char c)
{
  if (!(cpu_mem_read_32(ITM_TCR) & 1))
    return;

  if (!(cpu_mem_read_32(ITM_ENA) & 1))
    return;

  while (!cpu_mem_read_32(ITM_STIM))
    ;

  cpu_mem_write_8(ITM_STIM, c);
}

static void swo_enable()
{
  cpu_mem_write_32(ITM_LOCK, ITM_LOCK_MAGIC);
  cpu_mem_write_32(DEMCR, cpu_mem_read_32(DEMCR)
                   | TRCENA
                   );

  cpu_mem_write_32(ITM_TCR, cpu_mem_read_32(ITM_TCR)
                   | ITM_TCR_ITMENA
                   );

  cpu_mem_write_32(ITM_TCR, cpu_mem_read_32(ITM_TCR)
                   | ITM_TCR_SWOENA
                   | ITM_TCR_ITMENA
                   );

  cpu_mem_write_32(ITM_ENA, cpu_mem_read_32(ITM_ENA)
                   | 1
                   );
}

#else

#define ITM_TRACE_ENABLE    0xe0000e00
#define ITM_TRACE_PRIVILEGE 0xe0000e40
#define ITM_TRACE_CONTROL   0xe0000e80
#define ITM_TRACE_CONTROL_BUSY (1 << 23)

#define ITM_LOCK_ACCESS     0xe0000fb0

#define ITM_LOCK_ACCESS_MAGIC 0xc5acce55

#define ITM_STIMULUS(x)     (0xe0000000 + 4 * (x))

#define DHCSR         0xe000edf0
#define DHCSR_C_DEBUG 1

#define DEMCR         0xe000edfc
#define DEMCR_TRCENA  (1 << 24)

#define TPIU_ACPR  (*(volatile uint32_t*)0xE0040010) // Async Clock
                                                         // presacler register
#define TPIU_SPPR  0xe00400f0
#define TPIU_SPPR_TRACE_PORT     0
#define TPIU_SPPR_SWO_MANCHESTER 1
#define TPIU_SPPR_SWO_NRZ        2

#define DWT_CTRL   (*(volatile uint32_t*)0xE0001000) // DWT Control Register
#define FFCR       (*(volatile uint32_t*)0xE0040304) // Formatter and flush
#define ITM_STIM_U32 (*(volatile uint32_t*)0xE0000000) // STIM word acces

void swo_putc(char c)
{
  if (!(cpu_mem_read_32(DHCSR) & DHCSR_C_DEBUG)
      || !(cpu_mem_read_32(DEMCR) & DEMCR_TRCENA)
      || !(cpu_mem_read_32(ITM_TRACE_ENABLE) & 1)
      || (cpu_mem_read_32(ITM_TRACE_CONTROL) & ITM_TRACE_CONTROL_BUSY))
    return;

  while (!cpu_mem_read_32(ITM_STIMULUS(0)))
    ;
  cpu_mem_write_8(ITM_STIMULUS(0), c);
}

static void swo_enable()
{
  cpu_mem_write_32(DEMCR, cpu_mem_read_32(DEMCR) | DEMCR_TRCENA);
  cpu_mem_write_32(ITM_LOCK_ACCESS, ITM_LOCK_ACCESS_MAGIC);

  cpu_mem_write_32(ITM_TRACE_ENABLE, cpu_mem_read_32(ITM_TRACE_ENABLE) | 1);
  cpu_mem_write_32(ITM_TRACE_CONTROL, 0);

  cpu_mem_write_32(TPIU_SPPR, TPIU_SPPR_SWO_NRZ);

  TPIU_ACPR  = TargetDiv - 1;
  ITM_TPR    = 0x00000000;
  DWT_CTRL   = 0x400003FE;
  FFCR       = 0x00000100;

  //
  // Enable ITM and stimulus port
  //

  ITM_TCR     = 0x1000D;
  ITM_ENA     = StimulusRegs | (1 << _ITMPort);
}

#endif

static PRINTF_OUTPUT_FUNC(swo_printk_out)
{
  for (size_t i = 0; i < len; ++i)
    swo_putc(str[i]);
}

void swo_printk_init(void)
{
  swo_enable();
  printk_set_output(swo_printk_out, NULL);
}
