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

#ifndef ARCH_NRF51_CCM_H_
#define ARCH_NRF51_CCM_H_

#include "peripheral.h"
#include "ids.h"

enum nrf51_ccm_task {
    NRF51_CCM_KSGEN = 0,
    NRF51_CCM_CRYPT = 1,
    NRF51_CCM_STOP = 2,
};

enum nrf51_ccm_event {
    NRF51_CCM_ENDKSGEN = 0,
    NRF51_CCM_ENDCRYPT = 1,
    NRF51_CCM_ERROR = 2,
};

enum nrf51_ccm_short {
    NRF51_CCM_ENDKSGEN_CRYPT = 0,
};

enum nrf51_ccm_register {
    NRF51_CCM_MICSTATUS = 0,
    NRF51_CCM_ENABLE = 64,
    NRF51_CCM_MODE = 65,
    NRF51_CCM_CNFPTR = 66,
    NRF51_CCM_INPTR = 67,
    NRF51_CCM_OUTPTR = 68,
    NRF51_CCM_SCRATCHPTR = 69,
};

#define NRF51_CCM_MICSTATUS_FAILED 0
#define NRF51_CCM_MICSTATUS_PASSED 1

#define NRF51_CCM_ENABLE_DISABLED 0
#define NRF51_CCM_ENABLE_ENABLED 2

#define NRF51_CCM_MODE_ENCRYPTION 0
#define NRF51_CCM_MODE_DECRYPTION_1MBIT 1
#define NRF51_CCM_MODE_DECRYPTION 2

#define NRF51_CCM_SCRATCH_SIZE(max_packet_size) (16 + (max_packet_size))

__attribute__((__packed__))
struct nrf51_ccm_param_s
{
  uint32_t key[4];
  uint64_t packet_counter;
  uint8_t direction;
  uint8_t iv[8];
};

#endif
