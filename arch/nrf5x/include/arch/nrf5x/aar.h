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

#ifndef ARCH_NRF_AAR_H_
#define ARCH_NRF_AAR_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_aar_task {
    NRF_AAR_START = 0,
    NRF_AAR_STOP = 2,
};

enum nrf5x_aar_event {
    NRF_AAR_END = 0,
    NRF_AAR_RESOLVED = 1,
    NRF_AAR_NOTRESOLVED = 2,
};

enum nrf5x_aar_register {
    NRF_AAR_STATUS = 0,
    NRF_AAR_ENABLE = 64,
    NRF_AAR_NIRK = 65,
    NRF_AAR_IRKPTR = 66,
    NRF_AAR_ADDRPTR = 68,
    NRF_AAR_SCRATCHPTR = 69,
};

#define NRF_AAR_ENABLE_DISABLED 0
#define NRF_AAR_ENABLE_ENABLED 3

#define NRF_AAR_SCRATCH_SIZE 3

#endif
