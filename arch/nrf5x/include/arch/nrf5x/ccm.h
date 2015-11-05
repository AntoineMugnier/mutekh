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

#ifndef ARCH_NRF_CCM_H_
#define ARCH_NRF_CCM_H_

#include "peripheral.h"
#include "ids.h"

enum nrf5x_ccm_task {
    NRF_CCM_KSGEN = 0,
    NRF_CCM_CRYPT = 1,
    NRF_CCM_STOP = 2,
};

enum nrf5x_ccm_event {
    NRF_CCM_ENDKSGEN = 0,
    NRF_CCM_ENDCRYPT = 1,
    NRF_CCM_ERROR = 2,
};

enum nrf5x_ccm_short {
    NRF_CCM_ENDKSGEN_CRYPT = 0,
};

enum nrf5x_ccm_register {
    NRF_CCM_MICSTATUS = 0,
    NRF_CCM_ENABLE = 64,
    NRF_CCM_MODE = 65,
    NRF_CCM_CNFPTR = 66,
    NRF_CCM_INPTR = 67,
    NRF_CCM_OUTPTR = 68,
    NRF_CCM_SCRATCHPTR = 69,
};

#define NRF_CCM_MICSTATUS_FAILED 0
#define NRF_CCM_MICSTATUS_PASSED 1

#define NRF_CCM_ENABLE_DISABLED 0
#define NRF_CCM_ENABLE_ENABLED 2

#define NRF_CCM_MODE_ENCRYPTION 0
#define NRF_CCM_MODE_DECRYPTION 1
#define NRF_CCM_MODE_DECRYPTION_FASTEST 2

#if defined(CONFIG_ARCH_NRF52)
# define NRF_CCM_MODE_RATE_1MBIT 0x000
# define NRF_CCM_MODE_RATE_2MBIT 0x100
# define NRF_CCM_MODE_LENGTH_5BIT 0
# define NRF_CCM_MODE_LENGTH_8BIT 0x1000000
#endif

#if defined(CONFIG_ARCH_NRF51)
# define NRF_CCM_MAX_HW_PACKET_SIZE 27
#elif defined(CONFIG_ARCH_NRF52)
# define NRF_CCM_MAX_HW_PACKET_SIZE 251
#endif
# define NRF_CCM_MAX_SW_PACKET_SIZE 251
# define NRF_CCM_SCRATCH_SIZE (16 + NRF_CCM_MAX_HW_PACKET_SIZE)

__attribute__((__packed__))
struct nrf5x_ccm_param_s
{
  uint32_t key[4];
  uint64_t packet_counter;
  uint8_t direction;
  uint8_t iv[8];
};

#endif
