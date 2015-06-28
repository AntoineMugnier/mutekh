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

#include <arch/nrf5x/ids.h>
#include <arch/nrf5x/power.h>
#include <arch/nrf5x/clock.h>
#include <arch/nrf5x/gpio.h>
#include <arch/nrf5x/rtc.h>
#include <arch/nrf5x/timer.h>

#include <cpu/arm32m/v7m.h>

#include <hexo/power.h>
#include <hexo/interrupt.h>

#include <hexo/power.h>

error_t power_reboot()
{
  cpu_mem_write_32(ARMV7M_AIRCR_ADDR, 0x5FA0004);
  return 0;
}

error_t power_shutdown()
{
  cpu_interrupt_disable();

  nrf_task_trigger(NRF_PERIPHERAL_ADDR(NRF5X_RTC0), NRF_RTC_STOP);
  nrf_task_trigger(NRF_PERIPHERAL_ADDR(NRF5X_RTC1), NRF_RTC_STOP);
  nrf_task_trigger(NRF_PERIPHERAL_ADDR(NRF5X_TIMER0), NRF_TIMER_STOP);
  nrf_task_trigger(NRF_PERIPHERAL_ADDR(NRF5X_TIMER1), NRF_TIMER_STOP);
  nrf_task_trigger(NRF_PERIPHERAL_ADDR(NRF5X_TIMER2), NRF_TIMER_STOP);

  nrf_task_trigger(NRF_PERIPHERAL_ADDR(NRF5X_CLOCK), NRF_CLOCK_HFCLKSTOP);
  nrf_task_trigger(NRF_PERIPHERAL_ADDR(NRF5X_CLOCK), NRF_CLOCK_LFCLKSTOP);
  nrf_task_trigger(NRF_PERIPHERAL_ADDR(NRF5X_CLOCK), NRF_CLOCK_CTSTOP);
  nrf_task_trigger(NRF_PERIPHERAL_ADDR(NRF5X_CLOCK), NRF_CLOCK_CTSTOP);

  nrf_reg_set(NRF_PERIPHERAL_ADDR(NRF5X_POWER), NRF_POWER_ENABLE, 0);

#if defined(CONFIG_ARCH_NRF51)
  nrf_reg_set(NRF_PERIPHERAL_ADDR(NRF5X_POWER), NRF_POWER_RAMON, 0x3);
  nrf_reg_set(NRF_PERIPHERAL_ADDR(NRF5X_POWER), NRF_POWER_RAMONB, 0x3);

  nrf_reg_set(NRF_PERIPHERAL_ADDR(NRF5X_POWER), NRF_POWER_DCDCEN, 0);
#endif
  nrf_reg_set(NRF_PERIPHERAL_ADDR(NRF5X_POWER), NRF_POWER_SYSTEMOFF, 1);

  for(;;)
    asm volatile("wfe");
}

static
enum power_reset_cause_e power_reset_cause_decode(uint32_t cause)
{
  if (cause & (NRF_POWER_RESETREAS_OFF | NRF_POWER_RESETREAS_LPCOMP | NRF_POWER_RESETREAS_DIF))
    return POWER_RESET_CAUSE_WAKEUP;

  if (cause & NRF_POWER_RESETREAS_RESETPIN)
    return POWER_RESET_CAUSE_HARD;

  if (cause & NRF_POWER_RESETREAS_SREQ)
    return POWER_RESET_CAUSE_SOFT;

  if (cause & (NRF_POWER_RESETREAS_DOG | NRF_POWER_RESETREAS_LOCKUP))
    return POWER_RESET_CAUSE_WATCHDOG;

  return POWER_RESET_CAUSE_UNKNOWN;
}

enum power_reset_cause_e power_reset_cause(void)
{
  static enum power_reset_cause_e cause = POWER_RESET_CAUSE_UNKNOWN;

  if (cause == POWER_RESET_CAUSE_UNKNOWN) {
    uint32_t tmp = nrf_reg_get(NRF_PERIPHERAL_ADDR(NRF5X_POWER), NRF_POWER_RESETREAS);
    nrf_reg_set(NRF_PERIPHERAL_ADDR(NRF5X_POWER), NRF_POWER_RESETREAS, -1);

    cause = power_reset_cause_decode(tmp);
  }

  return cause;
}
