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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

#ifndef ARCH_NRF51_TIMER_H_
#define ARCH_NRF51_TIMER_H_

enum nrf51_timer_task {
    NRF51_TIMER_START = 0,
    NRF51_TIMER_STOP = 1,
    NRF51_TIMER_COUNT = 2,
    NRF51_TIMER_CLEAR = 3,
    NRF51_TIMER_SHUTDOWN = 4,
    NRF51_TIMER_CAPTURE0 = 16,
    NRF51_TIMER_CAPTURE1 = 17,
    NRF51_TIMER_CAPTURE2 = 18,
    NRF51_TIMER_CAPTURE3 = 19,
#define NRF51_TIMER_CAPTURE(x) (NRF51_TIMER_CAPTURE0 + (x))
};

enum nrf51_timer_event {
    NRF51_TIMER_COMPARE0 = 16,
    NRF51_TIMER_COMPARE1 = 17,
    NRF51_TIMER_COMPARE2 = 18,
    NRF51_TIMER_COMPARE3 = 19,
#define NRF51_TIMER_COMPARE(x) (NRF51_TIMER_COMPARE0 + (x))
};

enum nrf51_timer_register {
    NRF51_TIMER_MODE = 65,
    NRF51_TIMER_BITMODE = 66,
    NRF51_TIMER_PRESCALER = 68,
    NRF51_TIMER_CC0 = 80,
    NRF51_TIMER_CC1 = 81,
    NRF51_TIMER_CC2 = 82,
    NRF51_TIMER_CC3 = 83,
#define NRF51_TIMER_CC(x) (NRF51_TIMER_CC0 + (x))
    // See PAN-73
    NRF51_TIMER_PPI_GPIOTE = 515,
};

enum nrf51_timer_shorts {
  NRF51_TIMER_COMPARE0_CLEAR,
  NRF51_TIMER_COMPARE1_CLEAR,
  NRF51_TIMER_COMPARE2_CLEAR,
  NRF51_TIMER_COMPARE3_CLEAR,
#define NRF51_TIMER_COMPARE_CLEAR(x) (NRF51_TIMER_COMPARE0_CLEAR + (x))
  NRF51_TIMER_COMPARE0_STOP = 8,
  NRF51_TIMER_COMPARE1_STOP,
  NRF51_TIMER_COMPARE2_STOP,
  NRF51_TIMER_COMPARE3_STOP,
#define NRF51_TIMER_COMPARE_STOP(x) (NRF51_TIMER_COMPARE0_STOP + (x))
};

#define NRF51_TIMER_MODE_TIMER 0
#define NRF51_TIMER_MODE_COUNTER 1

#define NRF51_TIMER_BITMODE_16 0
#define NRF51_TIMER_BITMODE_8 1
#define NRF51_TIMER_BITMODE_24 2
#define NRF51_TIMER_BITMODE_32 3

#endif
