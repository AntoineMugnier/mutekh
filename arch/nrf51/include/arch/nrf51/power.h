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

#ifndef ARCH_NRF_POWER_H_
#define ARCH_NRF_POWER_H_

#include "peripheral.h"
#include "ids.h"

enum nrf51_power_task {
    NRF51_POWER_CONSTLAT = 30,
    NRF51_POWER_LOWPWR = 31,
};

enum nrf51_power_event {
    NRF51_POWER_POFWARN = 2,
};

enum nrf51_power_register {
    NRF51_POWER_RESETREAS = 0,
    NRF51_POWER_STATUS = 10,
    NRF51_POWER_SYSTEMOFF = 64,
    NRF51_POWER_ENABLE = 65,
    NRF51_POWER_POFCON = 66,
    NRF51_POWER_GPREGRET = 67,
    NRF51_POWER_RAMON = 73,
    NRF51_POWER_RESET = 81,
    NRF51_POWER_RAMONB = 85,
    NRF51_POWER_DCDCEN = 94,
    NRF51_POWER_DCDCFORCE = 386,
};

#define NRF51_POWER_RESETREAS_RESETPIN 0x1
#define NRF51_POWER_RESETREAS_DOG      0x2
#define NRF51_POWER_RESETREAS_SREQ     0x4
#define NRF51_POWER_RESETREAS_LOCKUP   0x8
#define NRF51_POWER_RESETREAS_OFF      0x10000
#define NRF51_POWER_RESETREAS_LPCOMP   0x20000
#define NRF51_POWER_RESETREAS_DIF      0x40000

#define NRF51_POWER_SYSTEMOFF_ENTER 0x1

#define NRF51_POWER_POFCON_ENABLE 1
#define NRF51_POWER_POFCON_V21 0
#define NRF51_POWER_POFCON_V23 2
#define NRF51_POWER_POFCON_V25 4
#define NRF51_POWER_POFCON_V27 6

#define NRF51_POWER_DCDCEN_ENABLE 0x1

#endif
