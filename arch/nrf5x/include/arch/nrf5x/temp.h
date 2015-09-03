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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#ifndef ARCH_NRF_TEMP_H_
#define ARCH_NRF_TEMP_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_temp_task {
    NRF_TEMP_START = 0,
    NRF_TEMP_STOP = 1,
};

enum nrf5x_temp_event {
    NRF_TEMP_DATARDY = 0,
};

enum nrf5x_temp_register {
    NRF_TEMP_TEMP = 66,
};

#endif
