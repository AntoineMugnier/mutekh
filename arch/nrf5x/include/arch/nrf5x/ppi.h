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

#ifndef ARCH_NRF_PPI_H_
#define ARCH_NRF_PPI_H_

#include "peripheral.h"
#include "ids.h"

#if defined(CONFIG_ARCH_NRF51)
# define NRF_PPI_COUNT 16
# define NRF_PPI_GROUP_COUNT 4
#elif defined(CONFIG_ARCH_NRF52)
# define NRF_PPI_COUNT 20
# define NRF_PPI_GROUP_COUNT 6
#endif

#define NRF_PPI_ADDR NRF_PERIPHERAL_ADDR(NRF5X_PPI)

enum nrf_ppi_task {
    NRF_PPI_CHG0_EN = 0,
    NRF_PPI_CHG0_DIS = 1,
};

#define NRF_PPI_CHG_EN(x) (NRF_PPI_CHG0_EN + (x))
#define NRF_PPI_CHG_DIS(x) (NRF_PPI_CHG0_DIS + (x))

enum nrf_ppi_register {
    NRF_PPI_CHEN = 64,
    NRF_PPI_CHENSET = 65,
    NRF_PPI_CHENCLR = 66,
    NRF_PPI_CH0_EEP = 68,
    NRF_PPI_CH0_TEP = 69,
    NRF_PPI_FORK0_TEP = 324,
    NRF_PPI_CHG0 = 256,
};

#define NRF_PPI_EEP(x) (NRF_PPI_CH0_EEP + 2 * (x))
#define NRF_PPI_TEP(x) (NRF_PPI_CH0_TEP + 2 * (x))
#define NRF_PPI_FORK_TEP(x) (NRF_PPI_FORK0_TEP + (x))
#define NRF_PPI_CHG(x) (NRF_PPI_CHG0 + (x))

#define NRF_PPI_TIMER0_COMPARE_0_RADIO_TXEN     20
#define NRF_PPI_TIMER0_COMPARE_0_RADIO_RXEN     21
#define NRF_PPI_TIMER0_COMPARE_1_RADIO_DISABLE  22
#define NRF_PPI_RADIO_BCMATCH_AAR_START         23
#define NRF_PPI_RADIO_READY_CCM_KSGEN           24
#define NRF_PPI_RADIO_ADDRESS_CCM_CRYPT         25
#define NRF_PPI_RADIO_ADDRESS_TIMER0_CAPTURE_1  26
#define NRF_PPI_RADIO_END_TIMER0_CAPTURE_2      27
#define NRF_PPI_RTC0_COMPARE_0_RADIO_TXEN       28
#define NRF_PPI_RTC0_COMPARE_0_RADIO_RXEN       29
#define NRF_PPI_RTC0_COMPARE_0_TIMER0_CLEAR     30
#define NRF_PPI_RTC0_COMPARE_0_TIMER0_START     31

#include <mutek/printk.h>

ALWAYS_INLINE
void nrf_ppi_setup(uint8_t channel,
    uintptr_t src, uint8_t event,
    uintptr_t dst, uint8_t task)
{
    uintptr_t eaddr = src | NRF_EVENT | (event << 2);
    uintptr_t taddr = dst | NRF_TASK | (task << 2);

    nrf_reg_set(NRF_PPI_ADDR, NRF_PPI_EEP(channel), eaddr);
    nrf_reg_set(NRF_PPI_ADDR, NRF_PPI_TEP(channel), taddr);
}

ALWAYS_INLINE
void nrf_ppi_enable_mask(uint32_t mask)
{
    nrf_reg_set(NRF_PPI_ADDR, NRF_PPI_CHENSET, mask);
}

ALWAYS_INLINE
void nrf_ppi_disable_mask(uint32_t mask)
{
    nrf_reg_set(NRF_PPI_ADDR, NRF_PPI_CHENCLR, mask);
}

ALWAYS_INLINE
void nrf_ppi_disable_enable_mask(uint32_t disable, uint32_t enable)
{
  nrf_ppi_disable_mask(disable);
  nrf_ppi_enable_mask(enable);
}

ALWAYS_INLINE
void nrf_ppi_enable(uint8_t channel)
{
  nrf_ppi_enable_mask(1 << channel);
}

ALWAYS_INLINE
void nrf_ppi_disable(uint8_t channel)
{
  nrf_ppi_disable_mask(1 << channel);
}

#endif
