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

#ifndef ARCH_NRF51_RTC_H_
#define ARCH_NRF51_RTC_H_

#include "peripheral.h"
#include "ids.h"

enum nrf51_rtc_task {
    NRF51_RTC_START = 0,
    NRF51_RTC_STOP = 1,
    NRF51_RTC_CLEAR = 2,
    NRF51_RTC_TRIGOVERFLW = 3,
    NRF51_RTC_UNDOC_TASK4 = 4, // Shutdown ?
};

enum nrf51_rtc_event {
    NRF51_RTC_TICK = 0,
    NRF51_RTC_OVERFLW = 1,
    NRF51_RTC_COMPARE0 = 16,
    NRF51_RTC_COMPARE1 = 17,
    NRF51_RTC_COMPARE2 = 18,
    NRF51_RTC_COMPARE3 = 19,
#define NRF51_RTC_COMPARE(x) (16 + (x))
};

enum nrf51_rtc_register {
    NRF51_RTC_COUNTER = 65,
    NRF51_RTC_PRESCALER = 66,
    NRF51_RTC_CC0 = 80,
    NRF51_RTC_CC1 = 81,
    NRF51_RTC_CC2 = 82,
    NRF51_RTC_CC3 = 83,
#define NRF51_RTC_CC(x) (80 + (x))
};

#endif
