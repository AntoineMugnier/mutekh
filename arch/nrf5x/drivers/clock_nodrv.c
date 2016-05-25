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

#include <mutek/startup.h>
#include <arch/nrf5x/clock.h>

#define CLOCK_ADDR NRF_PERIPHERAL_ADDR(NRF5X_CLOCK)

void arch_nrf5x_clock_init(void)
{
  nrf_reg_set(CLOCK_ADDR, NRF_CLOCK_LFCLKSRC, NRF_CLOCK_LF_SRC_RC);
  nrf_task_trigger(CLOCK_ADDR, NRF_CLOCK_LFCLKSTART);
  nrf_event_wait_clear(CLOCK_ADDR, NRF_CLOCK_LFCLKSTARTED);
}
