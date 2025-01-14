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

#ifndef ARCH_NRF_RNG_H_
#define ARCH_NRF_RNG_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_rng_task {
    NRF_RNG_START = 0,
    NRF_RNG_STOP = 1,
};

enum nrf5x_rng_shorts {
    NRF_RNG_VALRDY_STOP = 0,
};

enum nrf5x_rng_event {
    NRF_RNG_VALRDY = 0,
};

enum nrf5x_rng_register {
    NRF_RNG_CONFIG = 65,
    NRF_RNG_VALUE = 66,
};

#define NRF_RNG_CONFIG_DERCEN_DISABLED 0
#define NRF_RNG_CONFIG_DERCEN_ENABLED 1

#endif
