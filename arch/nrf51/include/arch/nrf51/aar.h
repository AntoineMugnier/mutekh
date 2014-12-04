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

#ifndef ARCH_NRF51_AAR_H_
#define ARCH_NRF51_AAR_H_

#include "peripheral.h"
#include "ids.h"

enum nrf51_aar_task {
    NRF51_AAR_START = 0,
    NRF51_AAR_STOP = 2,
};

enum nrf51_aar_event {
    NRF51_AAR_END = 0,
    NRF51_AAR_RESOLVED = 1,
    NRF51_AAR_NOTRESOLVED = 2,
};

enum nrf51_aar_register {
    NRF51_AAR_STATUS = 0,
    NRF51_AAR_ENABLE = 64,
    NRF51_AAR_NIRK = 65,
    NRF51_AAR_IRKPTR = 66,
    NRF51_AAR_ADDRPTR = 68,
    NRF51_AAR_SCRATCHPTR = 69,
};

#define NRF51_AAR_ENABLE_DISABLED 0
#define NRF51_AAR_ENABLE_ENABLED 3

#define NRF51_AAR_SCRATCH_SIZE 3

#endif
