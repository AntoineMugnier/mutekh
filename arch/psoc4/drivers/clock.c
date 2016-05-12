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

  Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

# include <mutek/startup.h>

#include <arch/psoc4/srss.h>
#include <arch/psoc4/peri.h>
#include <arch/psoc4/sflash.h>
#include <arch/psoc4/variant/procble.h>
#include <arch/psoc4/cpuss.h>

//#define dprintk printk
#ifndef dprintk
# define dwritek(...) do{}while(0)
# define dprintk(...) do{}while(0)
#else
# define dwritek writek
#endif

#define SRSS PSOC4_SRSS_ADDR
#define PERI PSOC4_PERI_ADDR
#define SFLASH PSOC4_SFLASH_ADDR
#define CPUSS PSOC4_CPUSS_ADDR

__attribute__((noinline))
static void psoc4_busy_wait(uint32_t loops)
{
  for (; loops; --loops)
    asm volatile("");
}

static uint_fast8_t clk_imo_trim2(uint_fast8_t freq_mhz)
{
  static const uint8_t gaps[] = {12, 24, 33, 40, 40};

  for (uint_fast8_t i = 0; i < sizeof(gaps); ++i) {
    if (freq_mhz <= gaps[i])
      return freq_mhz + i;
  }

  return freq_mhz + sizeof(gaps);
}

static
void psoc4_imo_mhz_set(uint_fast8_t freq_mhz)
{
  uint_fast32_t old_trim2 = cpu_mem_read_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR);
  uint_fast32_t imo_trim2 = SRSS_CLK_IMO_TRIM2_FREQ(clk_imo_trim2(freq_mhz));

  if (old_trim2 == imo_trim2)
    return;

  uint_fast32_t high_trim2 = clk_imo_trim2(24);
  uint_fast32_t imo_trim1 = SRSS_CLK_IMO_TRIM1_OFFSET(
    cpu_mem_read_8(SFLASH + SFLASH_IMO_TRIM_ADDR(freq_mhz - 3)));
  uint_fast32_t pwr_trim4 = SRSS_PWR_BG_TRIM4_ABS_TRIM_IMO(
    cpu_mem_read_8(SFLASH + SFLASH_IMO_ABS4_ADDR));
  uint_fast32_t pwr_trim5 = SRSS_PWR_BG_TRIM5_TMPCO_TRIM_IMO(
    cpu_mem_read_8(SFLASH + SFLASH_IMO_TMPCO4_ADDR));
  uint32_t select, flash_ctl;

  for (uint_fast8_t i = 0; i < SFLASH_IMO_MAXF_COUNT; ++i) {
    if (freq_mhz <= cpu_mem_read_8(SFLASH + SFLASH_IMO_MAXF_ADDR(i))) {
      pwr_trim4 = SRSS_PWR_BG_TRIM4_ABS_TRIM_IMO(
        cpu_mem_read_8(SFLASH + SFLASH_IMO_ABS_ADDR(i)));
      pwr_trim5 = SRSS_PWR_BG_TRIM5_TMPCO_TRIM_IMO(
        cpu_mem_read_8(SFLASH + SFLASH_IMO_TMPCO_ADDR(i)));
      
      break;
    }
  }

  dprintk("%s pwr trim4: %08x trim5: %08x, imo trim1: %08x, trim2: %08x\n",
         __FUNCTION__,
         pwr_trim4,
         pwr_trim5,
         imo_trim1,
         imo_trim2);

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  dprintk("Setting flash half\n");
  select = cpu_mem_read_32(SRSS + SRSS_CLK_SELECT_ADDR);
  select |= SRSS_CLK_SELECT_HALF_EN;
  cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, select);

  flash_ctl = cpu_mem_read_32(CPUSS + CPUSS_FLASH_CTL_ADDR);
  CPUSS_FLASH_CTL_FLASH_WS_SETVAL(flash_ctl, 2);
  cpu_mem_write_32(CPUSS + CPUSS_FLASH_CTL_ADDR, flash_ctl);

  if (old_trim2 >= high_trim2 || imo_trim2 >= high_trim2) {
    dprintk("Using 24MHz temporarily\n");
    old_trim2 = SRSS_CLK_IMO_TRIM2_FREQ(clk_imo_trim2(24));
    cpu_mem_write_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR,
                     old_trim2);
    psoc4_busy_wait(4*5);
  }

  if (old_trim2 > imo_trim2) {
    dprintk("Setting trim2 first\n");
    cpu_mem_write_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR, imo_trim2);
    psoc4_busy_wait(4*5);
  }

  dprintk("Applying trims\n");
  cpu_mem_write_32(SRSS + SRSS_CLK_IMO_TRIM1_ADDR, imo_trim1);
  cpu_mem_write_32(SRSS + SRSS_PWR_BG_TRIM4_ADDR, pwr_trim4);
  cpu_mem_write_32(SRSS + SRSS_PWR_BG_TRIM5_ADDR, pwr_trim5);
  psoc4_busy_wait(50*4*5);

  if (old_trim2 < imo_trim2) {
    dprintk("Setting trim2\n");
    cpu_mem_write_32(SRSS + SRSS_CLK_IMO_TRIM2_ADDR, imo_trim2);
    psoc4_busy_wait(50*4*5);
  }

  CPUSS_FLASH_CTL_FLASH_WS_SETVAL(flash_ctl, freq_mhz / 16);
  cpu_mem_write_32(CPUSS + CPUSS_FLASH_CTL_ADDR, flash_ctl);

  if (imo_trim2 < high_trim2) {
    dprintk("Removing flash half\n");
    select &= ~SRSS_CLK_SELECT_HALF_EN;
    cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, select);
    psoc4_busy_wait(5*4*5);
  }

  CPU_INTERRUPT_RESTORESTATE;
}

void psoc4_clock_setup(void)
{
  cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, 0
                   | SRSS_CLK_SELECT_DIRECT_SEL(IMO)
                   );

  psoc4_imo_mhz_set(24);

  cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, 0
                   | SRSS_CLK_ILO_CONFIG_ENABLE
                   | SRSS_CLK_ILO_CONFIG_SATBIAS(SATURATED)
                   | SRSS_CLK_ILO_CONFIG_TURBO_EN
                   );
}
