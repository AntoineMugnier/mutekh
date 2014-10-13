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

#ifndef ARCH_NRF51_PPI_H_
#define ARCH_NRF51_PPI_H_

#include "peripheral.h"
#include "ids.h"

#define NRF51_PPI_ADDR NRF_PERIPHERAL_ADDR(NRF51_PPI)

enum nrf51_ppi_task {
    NRF51_PPI_CHG0_EN = 0,
    NRF51_PPI_CHG0_DIS = 1,
    NRF51_PPI_CHG1_EN = 0,
    NRF51_PPI_CHG1_DIS = 1,
    NRF51_PPI_CHG2_EN = 0,
    NRF51_PPI_CHG2_DIS = 1,
    NRF51_PPI_CHG3_EN = 0,
    NRF51_PPI_CHG3_DIS = 1,
};

enum nrf51_ppi_register {
    NRF51_PPI_CHEN = 64,
    NRF51_PPI_CHENSET = 65,
    NRF51_PPI_CHENCLR = 66,
    NRF51_PPI_CH0_EEP = 68,
    NRF51_PPI_CH0_TEP = 69,
    NRF51_PPI_CH1_EEP = 70,
    NRF51_PPI_CH1_TEP = 71,
    NRF51_PPI_CH2_EEP = 72,
    NRF51_PPI_CH2_TEP = 73,
    NRF51_PPI_CH3_EEP = 74,
    NRF51_PPI_CH3_TEP = 75,
    NRF51_PPI_CH4_EEP = 76,
    NRF51_PPI_CH4_TEP = 77,
    NRF51_PPI_CH5_EEP = 78,
    NRF51_PPI_CH5_TEP = 79,
    NRF51_PPI_CH6_EEP = 80,
    NRF51_PPI_CH6_TEP = 81,
    NRF51_PPI_CH7_EEP = 82,
    NRF51_PPI_CH7_TEP = 83,
    NRF51_PPI_CH8_EEP = 84,
    NRF51_PPI_CH8_TEP = 85,
    NRF51_PPI_CH9_EEP = 86,
    NRF51_PPI_CH9_TEP = 87,
    NRF51_PPI_CH10_EEP = 88,
    NRF51_PPI_CH10_TEP = 89,
    NRF51_PPI_CH11_EEP = 90,
    NRF51_PPI_CH11_TEP = 91,
    NRF51_PPI_CH12_EEP = 92,
    NRF51_PPI_CH12_TEP = 93,
    NRF51_PPI_CH13_EEP = 94,
    NRF51_PPI_CH13_TEP = 95,
    NRF51_PPI_CH14_EEP = 96,
    NRF51_PPI_CH14_TEP = 97,
    NRF51_PPI_CH15_EEP = 98,
    NRF51_PPI_CH15_TEP = 99,
    NRF51_PPI_CHG0 = 256,
    NRF51_PPI_CHG1 = 257,
    NRF51_PPI_CHG2 = 258,
    NRF51_PPI_CHG3 = 259,
};

#define NRF51_PPI_EEP(x) (NRF51_PPI_CH0_EEP + 2 * (x))
#define NRF51_PPI_TEP(x) (NRF51_PPI_CH0_TEP + 2 * (x))
#define NRF51_PPI_CHG(x) (NRF51_PPI_CHG0 + (x))

#define NRF51_PPI_TIMER0_COMPARE_0_RADIO_TXEN     20
#define NRF51_PPI_TIMER0_COMPARE_0_RADIO_RXEN     21
#define NRF51_PPI_TIMER0_COMPARE_1_RADIO_DISABLE  22
#define NRF51_PPI_RADIO_BCMATCH_AAR_START         23
#define NRF51_PPI_RADIO_READY_CCM_KSGEN           24
#define NRF51_PPI_RADIO_ADDRESS_CCM_CRYPT         25
#define NRF51_PPI_RADIO_ADDRESS_TIMER0_CAPTURE_1  26
#define NRF51_PPI_RADIO_END_TIMER0_CAPTURE_2      27
#define NRF51_PPI_RTC0_COMPARE_0_RADIO_TXEN       28
#define NRF51_PPI_RTC0_COMPARE_0_RADIO_RXEN       29
#define NRF51_PPI_RTC0_COMPARE_0_TIMER0_CLEAR     30
#define NRF51_PPI_RTC0_COMPARE_0_TIMER0_START     31

ALWAYS_INLINE
void nrf51_ppi_setup(
    uintptr_t base, uint8_t channel,
    uintptr_t src, uint8_t event,
    uintptr_t dst, uint8_t task)
{
    uintptr_t eaddr = src | NRF_EVENT | (event << 2);
    uintptr_t taddr = dst | NRF_TASK | (task << 2);

    nrf_reg_set(base, NRF51_PPI_EEP(channel), eaddr);
    nrf_reg_set(base, NRF51_PPI_TEP(channel), taddr);
}

ALWAYS_INLINE
void nrf51_ppi_enable_mask(uintptr_t base, uint32_t mask)
{
    nrf_reg_set(base, NRF51_PPI_CHENSET, mask);
}

ALWAYS_INLINE
void nrf51_ppi_disable_mask(uintptr_t base, uint32_t mask)
{
    nrf_reg_set(base, NRF51_PPI_CHENCLR, mask);
}

ALWAYS_INLINE
void nrf51_ppi_enable(uintptr_t base, uint8_t channel)
{
  nrf51_ppi_enable_mask(base, 1 << channel);
}

ALWAYS_INLINE
void nrf51_ppi_disable(uintptr_t base, uint8_t channel)
{
  nrf51_ppi_disable_mask(base, 1 << channel);
}

#endif
