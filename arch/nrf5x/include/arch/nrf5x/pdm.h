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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2016
*/

#ifndef ARCH_NRF_PDM_H_
#define ARCH_NRF_PDM_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_pdm_task {
    NRF_PDM_START = 0,
    NRF_PDM_STOP = 1,
};

enum nrf5x_pdm_event {
    NRF_PDM_STARTED = 0,
    NRF_PDM_STOPPED = 1,
    NRF_PDM_END = 2,
};

enum nrf5x_pdm_register {
    NRF_PDM_ENABLE = 64,
    NRF_PDM_CLKCTRL = 65,
    NRF_PDM_MODE = 66,
    NRF_PDM_GAINL = 70,
    NRF_PDM_GAINR = 71,
    NRF_PDM_PSEL_CLK = 80,
    NRF_PDM_PSEL_DIN = 81,
    NRF_PDM_SAMPLE_PTR = 88,
    NRF_PDM_SAMPLE_MAXCNT = 89,
};

#define NRF_PDM_ENABLE_ENABLED 1

#define NRF_PDM_CLKCTRL_FREQ(rate) (0x10000000 - (((uint32_t)0xf4240000 / (((uint32_t)(rate) + 0x100) >> 9) + 0x3ff000) & ~0x3fffff))
#define NRF_PDM_CLKCTRL_RATE(reg) (0x1e8480 / ((0x10000000 - (reg)) >> 20))

#define NRF_PDM_MODE_OPERATION_MASK 1
#define NRF_PDM_MODE_OPERATION_STEREO 0
#define NRF_PDM_MODE_OPERATION_MONO 1
#define NRF_PDM_MODE_EDGE_MASK 2
#define NRF_PDM_MODE_EDGE_LEFTFALLING 0
#define NRF_PDM_MODE_EDGE_LEFTRISING 2

#define NRF_PDM_GAIN(half_db_gain) (0x28 + (half_db_gain))

#endif
