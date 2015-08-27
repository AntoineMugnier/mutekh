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

#ifndef ARCH_NRF_TIMER_H_
#define ARCH_NRF_TIMER_H_

enum nrf5x_timer_task {
    NRF_TIMER_START = 0,
    NRF_TIMER_STOP = 1,
    NRF_TIMER_COUNT = 2,
    NRF_TIMER_CLEAR = 3,
    NRF_TIMER_SHUTDOWN = 4,
    NRF_TIMER_CAPTURE0 = 16,
#define NRF_TIMER_CAPTURE(x) (NRF_TIMER_CAPTURE0 + (x))
};

enum nrf5x_timer_event {
    NRF_TIMER_COMPARE0 = 16,
#define NRF_TIMER_COMPARE(x) (NRF_TIMER_COMPARE0 + (x))
};

enum nrf5x_timer_register {
    NRF_TIMER_MODE = 65,
    NRF_TIMER_BITMODE = 66,
    NRF_TIMER_PRESCALER = 68,
    NRF_TIMER_CC0 = 80,
#define NRF_TIMER_CC(x) (NRF_TIMER_CC0 + (x))
    // See PAN-73
    NRF_TIMER_PPI_GPIOTE = 515,
};

enum nrf5x_timer_shorts {
  NRF_TIMER_COMPARE0_CLEAR,
#define NRF_TIMER_COMPARE_CLEAR(x) (NRF_TIMER_COMPARE0_CLEAR + (x))
  NRF_TIMER_COMPARE0_STOP = 8,
#define NRF_TIMER_COMPARE_STOP(x) (NRF_TIMER_COMPARE0_STOP + (x))
};

#define NRF_TIMER_MODE_TIMER 0
#define NRF_TIMER_MODE_COUNTER 1
#define NRF_TIMER_MODE_LOW_POWER_COUNTER 2

#define NRF_TIMER_BITMODE_16 0
#define NRF_TIMER_BITMODE_8 1
#define NRF_TIMER_BITMODE_24 2
#define NRF_TIMER_BITMODE_32 3

#endif
