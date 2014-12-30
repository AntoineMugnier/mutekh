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

#ifndef ARCH_NRF51_ADC_H_
#define ARCH_NRF51_ADC_H_

#include "peripheral.h"
#include "ids.h"

enum nrf51_adc_task {
    NRF51_ADC_START = 0,
    NRF51_ADC_STOP = 1,
};

enum nrf51_adc_event {
    NRF51_ADC_END = 0,
};

enum nrf51_adc_register {
    NRF51_ADC_BUSY = 0,
    NRF51_ADC_ENABLE = 64,
    NRF51_ADC_CONFIG = 65,
    NRF51_ADC_RESULT = 66,
};

#define NRF51_ADC_ENABLE_ENABLED 1

#define NRF51_ADC_CONFIG_RES_8BIT 0
#define NRF51_ADC_CONFIG_RES_9BIT 1
#define NRF51_ADC_CONFIG_RES_10BIT 2

#define NRF51_ADC_CONFIG_INPSEL_RAW (0 << 2)
#define NRF51_ADC_CONFIG_INPSEL_2_3 (1 << 2)
#define NRF51_ADC_CONFIG_INPSEL_1_3 (2 << 2)
#define NRF51_ADC_CONFIG_INPSEL_VDD_2_3 (5 << 2)
#define NRF51_ADC_CONFIG_INPSEL_VDD_1_3 (6 << 2)

#define NRF51_ADC_CONFIG_REFSEL_1_2V (0 << 5)
#define NRF51_ADC_CONFIG_REFSEL_EXT (1 << 5)
#define NRF51_ADC_CONFIG_REFSEL_VDD_2 (2 << 5)
#define NRF51_ADC_CONFIG_REFSEL_VDD_3 (3 << 5)

#define NRF51_ADC_CONFIG_PSEL_NONE 0
#define NRF51_ADC_CONFIG_PSEL(x) (1 << ((x) + 8))

#define NRF51_ADC_CONFIG_EXTREF_NONE 0
#define NRF51_ADC_CONFIG_EXTREF_0 (1 << 16)
#define NRF51_ADC_CONFIG_EXTREF_1 (2 << 16)

#endif
