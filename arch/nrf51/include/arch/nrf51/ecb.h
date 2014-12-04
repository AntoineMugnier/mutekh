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

#ifndef ARCH_NRF51_ECB_H_
#define ARCH_NRF51_ECB_H_

#include "peripheral.h"
#include "ids.h"

enum nrf51_ecb_task {
    NRF51_ECB_STARTECB = 0,
    NRF51_ECB_STOPECB = 1,
};

enum nrf51_ecb_event {
    NRF51_ECB_ENDECB = 0,
    NRF51_ECB_ERRORECB = 1,
};

enum nrf51_ecb_register {
    NRF51_ECB_ENABLE = 64,
    NRF51_ECB_ECBDATAPTR = 65,
};

struct nrf51_ecb_param_s
{
  uint32_t key[4];
  uint32_t cleartext[4];
  uint32_t ciphertext[4];
};

#define NRF51_ECB_ENABLE_ENABLED 0

#endif
