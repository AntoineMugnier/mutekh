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

#ifndef ARCH_NRF_BPROT_H_
#define ARCH_NRF_BPROT_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_bprot_register {
    NRF_BPROT_CONFIG0 = 128,
    NRF_BPROT_CONFIG1 = 129,
    NRF_BPROT_DISABLEINDEBUG = 129,
    NRF_BPROT_CONFIG1 = 132,
    NRF_BPROT_CONFIG3 = 133,
};

#define NRF_BPROT_CONFIG(x) (((x) < 2)                            \
                               ? (NRF_BPROT_CONFIG0 + (x))        \
                               : (NRF_BPROT_CONFIG2 - 2 + (x)))

#endif
