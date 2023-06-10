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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2021
*/

#ifndef ARCH_NRF_QDEC_H_
#define ARCH_NRF_QDEC_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_qdec_tasks {
    NRF_QDEC_START = 0,
    NRF_QDEC_STOP = 1,
    NRF_QDEC_READCLRACC = 2,
    NRF_QDEC_RDCLRACC = 3,
    NRF_QDEC_RDCLRDBL = 4,
};

enum nrf5x_qdec_event {
    NRF_QDEC_SAMPLERDY = 0,
    NRF_QDEC_REPORTRDY = 1,
    NRF_QDEC_ACCOF = 2,
    NRF_QDEC_DBLRDY = 3,
    NRF_QDEC_STOPPED = 4,
};

enum nrf5x_qdec_register {
    NRF_QDEC_ENABLE = 64,
    NRF_QDEC_LEDPOL = 65,
    NRF_QDEC_SAMPLEPER = 66,
    NRF_QDEC_SAMPLE = 67,
    NRF_QDEC_REPORTPER = 68,
    NRF_QDEC_ACC = 69,
    NRF_QDEC_ACCREAD = 70,
    NRF_QDEC_PSEL_LED = 71,
    NRF_QDEC_PSEL_A = 72,
    NRF_QDEC_PSEL_B = 73,
    NRF_QDEC_DBFEN = 74,
    NRF_QDEC_LEDPRE = 80,
    NRF_QDEC_ACCDBL = 81,
    NRF_QDEC_ACCDBLREAD = 82,
};

#define NRF_QDEC_ENABLE_MASK       0xf
#define NRF_QDEC_ENABLE_DISABLED   0x0
#define NRF_QDEC_ENABLE_ENABLED    0x1

#endif
