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

#include <arch/psoc4/variant/procble.h>
#include <arch/psoc4/srss.h>

#include <cpu/arm32m/v7m.h>

#include <hexo/power.h>
#include <hexo/interrupt.h>
#include <mutek/startup.h>
#include <hexo/power.h>

#include <hexo/iospace.h>

error_t power_reboot(void)
{
  cpu_mem_write_32(ARMV7M_AIRCR_ADDR, 0x5FA0004);
  return 0;
}

void psoc4_power_setup(void)
{
  uint32_t tmp = cpu_mem_read_32(PSOC4_SRSS_ADDR + SRSS_PWR_STOP_ADDR);

  SRSS_PWR_STOP_UNLOCK_SET(tmp, KEY);
  cpu_mem_write_32(PSOC4_SRSS_ADDR + SRSS_PWR_STOP_ADDR, tmp);

  tmp &= ~SRSS_PWR_STOP_FREEZE;
  cpu_mem_write_32(PSOC4_SRSS_ADDR + SRSS_PWR_STOP_ADDR, tmp);

  SRSS_PWR_STOP_UNLOCK_SETVAL(tmp, 0);
  cpu_mem_write_32(PSOC4_SRSS_ADDR + SRSS_PWR_STOP_ADDR, tmp);
}

error_t power_shutdown(void)
{
  cpu_interrupt_disable();

  uint32_t tmp = cpu_mem_read_32(PSOC4_SRSS_ADDR + SRSS_PWR_STOP_ADDR);

  SRSS_PWR_STOP_TOKEN_SET(tmp, 1);
  cpu_mem_write_32(PSOC4_SRSS_ADDR + SRSS_PWR_STOP_ADDR, tmp);

  SRSS_PWR_STOP_UNLOCK_SET(tmp, KEY);
  cpu_mem_write_32(PSOC4_SRSS_ADDR + SRSS_PWR_STOP_ADDR, tmp);

  tmp |= SRSS_PWR_STOP_FREEZE;
  cpu_mem_write_32(PSOC4_SRSS_ADDR + SRSS_PWR_STOP_ADDR, tmp);

  asm volatile("nop");
  asm volatile("nop");

  tmp |= SRSS_PWR_STOP_STOP;
  cpu_mem_write_32(PSOC4_SRSS_ADDR + SRSS_PWR_STOP_ADDR, tmp);

  return 0;
}

static
enum power_reset_cause_e power_reset_cause_get(void)
{
  uint32_t cause = cpu_mem_read_32(PSOC4_SRSS_ADDR + SRSS_RES_CAUSE_ADDR);
  uint32_t stop = cpu_mem_read_32(PSOC4_SRSS_ADDR + SRSS_PWR_STOP_ADDR);

  cpu_mem_write_32(PSOC4_SRSS_ADDR + SRSS_RES_CAUSE_ADDR, cause);

  if (cause & SRSS_RES_CAUSE_RESET_WDT)
    return POWER_RESET_CAUSE_WATCHDOG;

  if (cause & SRSS_RES_CAUSE_RESET_PROT_FAULT)
    return POWER_RESET_CAUSE_FAULT;

  if (cause & SRSS_RES_CAUSE_RESET_SOFT)
    return POWER_RESET_CAUSE_SOFT;

  if (cause & ~(SRSS_RES_CAUSE_RESET_SOFT
                | SRSS_RES_CAUSE_RESET_PROT_FAULT
                | SRSS_RES_CAUSE_RESET_WDT))
    return POWER_RESET_CAUSE_HARD;

  if (SRSS_PWR_STOP_TOKEN_GET(stop))
    return POWER_RESET_CAUSE_WAKEUP;

  return POWER_RESET_CAUSE_POWERUP;
}

enum power_reset_cause_e power_reset_cause(void)
{
  static enum power_reset_cause_e cause = POWER_RESET_CAUSE_UNKNOWN;

  if (cause == POWER_RESET_CAUSE_UNKNOWN)
    cause = power_reset_cause_get();

  return cause;
}
