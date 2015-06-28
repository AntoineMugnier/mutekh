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

#define ITM_ENA (*(volatile uint32_t*)0xE0000E00) // ITM Enable
#define ITM_TPR (*(volatile uint32_t*)0xE0000E40) // Trace Privilege Register
#define ITM_TCR (*(volatile uint32_t*)0xE0000E80) // ITM Trace Control Reg.
#define ITM_LSR (*(volatile uint32_t*)0xE0000FB0) // ITM Lock Status Register
#define DHCSR (*(volatile uint32_t*)0xE000EDF0) // Debug register
#define DEMCR (*(volatile uint32_t*)0xE000EDFC) // Debug register
#define TPIU_ACPR  (*(volatile uint32_t*)0xE0040010) // Async Clock
                                                         // presacler register
#define TPIU_SPPR  (*(volatile uint32_t*)0xE00400F0) // Selected Pin Protocol
                                                         // Register
#define DWT_CTRL   (*(volatile uint32_t*)0xE0001000) // DWT Control Register
#define FFCR       (*(volatile uint32_t*)0xE0040304) // Formatter and flush
                                                         // Control Register
#define ITM_STIM_U32 (*(volatile uint32_t*)0xE0000000) // STIM word acces
#define ITM_STIM_U8  (*(volatile char*)0xE0000000) // STIM byte acces

void swo_putc(char c)
{
  if ((DHCSR & 1)!= 1)
    return;

  if ((DEMCR & (1 << 24)) == 0)
    return;

  if ((ITM_TCR & (1 << 22)) == 1)
    return;

  if ((ITM_ENA & 1) == 0)
    return;

  while ((ITM_STIM_U8 & 1) == 0);
  ITM_STIM_U8 = c;
}

static void swo_enable()
{
  uint32_t _ITMPort = 0; // The stimulus port from which SWO data is received and displayed.
  uint32_t TargetDiv = 32;
  uint32_t StimulusRegs;

  //
  // Enable access to SWO registers
  //

  DEMCR |= (1 << 24);
  ITM_LSR = 0xC5ACCE55;

  //
  // Initially disable ITM and stimulus port
  // To make sure that nothing is transferred via SWO
  // when changing the SWO prescaler etc.
  //

  StimulusRegs = ITM_ENA;
  StimulusRegs &= ~(1 << _ITMPort);
  ITM_ENA    = StimulusRegs;
  ITM_TCR    = 0;

  //
  // Initialize SWO (prescaler, etc.)
  //

  TPIU_SPPR  = 0x00000002;
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
