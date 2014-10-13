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

#ifndef ARCH_NRF_RTC_H_
#define ARCH_NRF_RTC_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_rtc_task {
    NRF_RTC_START = 0,
    NRF_RTC_STOP = 1,
    NRF_RTC_CLEAR = 2,
    NRF_RTC_TRIGOVERFLW = 3,
    NRF_RTC_UNDOC_TASK4 = 4, // Shutdown ?
};

enum nrf5x_rtc_event {
    NRF_RTC_TICK = 0,
    NRF_RTC_OVERFLW = 1,
    NRF_RTC_COMPARE0 = 16,
#define NRF_RTC_COMPARE(x) (16 + (x))
};

enum nrf5x_rtc_register {
    NRF_RTC_COUNTER = 65,
    NRF_RTC_PRESCALER = 66,
    NRF_RTC_CC0 = 80,
#define NRF_RTC_CC(x) (80 + (x))
};

#endif
