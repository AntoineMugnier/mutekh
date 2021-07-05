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

enum nrf5x_clock_task {
    NRF_CLOCK_HFCLKSTART = 0,
    NRF_CLOCK_HFCLKSTOP = 1,
    NRF_CLOCK_LFCLKSTART = 2,
    NRF_CLOCK_LFCLKSTOP = 3,
    NRF_CLOCK_CAL = 4,
    NRF_CLOCK_CTSTART = 5,
    NRF_CLOCK_CTSTOP = 6,
};

enum nrf5x_clock_event {
    NRF_CLOCK_HFCLKSTARTED = 0,
    NRF_CLOCK_LFCLKSTARTED = 1,
    NRF_CLOCK_DONE = 3,
    NRF_CLOCK_CTTO = 4,
};

enum nrf5x_clock_register {
    NRF_CLOCK_HFCLKRUN = 2,
    NRF_CLOCK_HFCLKSTAT = 3,
    NRF_CLOCK_LFCLKRUN = 5,
    NRF_CLOCK_LFCLKSTAT = 6,
    NRF_CLOCK_LFCLKSRCCOPY = 7,
    NRF_CLOCK_LFCLKSRC = 70,
    NRF_CLOCK_CTIV = 78, // In .25s steps
    NRF_CLOCK_XTALFREQ = 84,
    NRF_CLOCK_TRACECONFIG = 87,
#if CONFIG_NRF5X_MODEL == 52840
    NRF_CLOCK_LFRCMODE = 109,
    NRF_CLOCK_HFXODEBOUNCE = 74,
#endif
};

#define NRF_CLOCK_HF_SRC_MASK      0x1
#define NRF_CLOCK_HF_SRC_RC        0x0
#define NRF_CLOCK_HF_SRC_XTAL      0x1
#define NRF_CLOCK_HFCLKSTAT_STATE_MASK    0x10000
#define NRF_CLOCK_HFCLKSTAT_STATE_RUNNING 0x10000

#define NRF_CLOCK_LF_SRC_MASK      0x3
#define NRF_CLOCK_LF_SRC_RC        0x0
#define NRF_CLOCK_LF_SRC_XTAL      0x1
#define NRF_CLOCK_LF_SRC_SYNTH     0x2
#define NRF_CLOCK_LFCLKSTAT_STATE_MASK    0x10000
#define NRF_CLOCK_LFCLKSTAT_STATE_RUNNING 0x10000

#if CONFIG_NRF5X_MODEL <= 51999
# define NRF_CLOCK_XTALFREQ_MASK  0xff
# define NRF_CLOCK_XTALFREQ_16MHZ 0xff
# define NRF_CLOCK_XTALFREQ_32MHZ 0x00
#endif

#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
# define NRF_CLOCK_TRACECONFIG_TRACEPORTSPEED_32MHZ 0
# define NRF_CLOCK_TRACECONFIG_TRACEPORTSPEED_16MHZ 1
# define NRF_CLOCK_TRACECONFIG_TRACEPORTSPEED_8MHZ 2
# define NRF_CLOCK_TRACECONFIG_TRACEPORTSPEED_4MHZ 3

# define NRF_CLOCK_TRACECONFIG_TRACEMUX_GPIO 0x00000
# define NRF_CLOCK_TRACECONFIG_TRACEMUX_SERIAL 0x10000
# define NRF_CLOCK_TRACECONFIG_TRACEMUX_PARALLEL 0x20000
#endif

#define NRF_CLOCK_LFRCMODE_MODE_ULP 0x1
#define NRF_CLOCK_LFRCMODE_STATUS_ULP 0x10000

#endif
