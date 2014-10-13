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

#ifndef ARCH_NRF_CLOCK_H_
#define ARCH_NRF_CLOCK_H_

#include "peripheral.h"
#include "ids.h"

enum nrf51_clock_task {
    NRF51_CLOCK_HFCLKSTART = 0,
    NRF51_CLOCK_HFCLKSTOP = 1,
    NRF51_CLOCK_LFCLKSTART = 2,
    NRF51_CLOCK_LFCLKSTOP = 3,
    NRF51_CLOCK_CAL = 4,
    NRF51_CLOCK_CTSTART = 5,
    NRF51_CLOCK_CTSTOP = 6,
};

enum nrf51_clock_event {
    NRF51_CLOCK_HFCLKSTARTED = 0,
    NRF51_CLOCK_LFCLKSTARTED = 1,
    NRF51_CLOCK_DONE = 3,
    NRF51_CLOCK_CTTO = 4,
};

enum nrf51_clock_register {
    NRF51_CLOCK_HFCLKSTAT = 3,
    NRF51_CLOCK_LFCLKSTAT = 6,
    NRF51_CLOCK_LFCLKSRC = 70,
    NRF51_CLOCK_CTIV = 78, // In .25s steps
    NRF51_CLOCK_XTALFREQ = 84,
};

#define NRF51_CLOCK_HFCLKSTAT_SRC_MASK      0x1
#define NRF51_CLOCK_HFCLKSTAT_SRC_RC        0x0
#define NRF51_CLOCK_HFCLKSTAT_SRC_XTAL      0x1
#define NRF51_CLOCK_HFCLKSTAT_STATE_MASK    0x10000
#define NRF51_CLOCK_HFCLKSTAT_STATE_RUNNING 0x10000

#define NRF51_CLOCK_LFCLKSTAT_SRC_MASK      0x3
#define NRF51_CLOCK_LFCLKSTAT_SRC_RC        0x0
#define NRF51_CLOCK_LFCLKSTAT_SRC_XTAL      0x1
#define NRF51_CLOCK_LFCLKSTAT_SRC_SYNTH     0x2
#define NRF51_CLOCK_LFCLKSTAT_STATE_MASK    0x10000
#define NRF51_CLOCK_LFCLKSTAT_STATE_RUNNING 0x10000

#define NRF51_CLOCK_XTALFREQ_MASK  0xff
#define NRF51_CLOCK_XTALFREQ_16MHZ 0xff
#define NRF51_CLOCK_XTALFREQ_32MHZ 0x00

#endif
