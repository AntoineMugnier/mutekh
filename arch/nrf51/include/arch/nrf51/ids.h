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

#ifndef ARCH_NRF51_IDS_H_
#define ARCH_NRF51_IDS_H_

#include <hexo/types.h>
#include <hexo/decls.h>

#define NRF_PERIPHERAL_ADDR(no) (0x40000000 | ((uintptr_t)(no) << 12))

#define NRF_STATIC_RES_PERIPHERAL_MEM(x) DEV_STATIC_RES_MEM(NRF_PERIPHERAL_ADDR((x)), NRF_PERIPHERAL_ADDR((x) + 1))

ALWAYS_INLINE uintptr_t nrf_peripheral_addr(uint8_t no)
{
    return 0x40000000 | ((uintptr_t)no << 12);
}

enum nrf51_peripheral {
    NRF51_POWER = 0,
    NRF51_CLOCK = 0,
    NRF51_RADIO = 1,
    NRF51_UART0 = 2,
    NRF51_SPI0 = 3,
    NRF51_TWI0 = 3,
    NRF51_SPI1 = 4,
    NRF51_TWI1 = 4,
    NRF51_SPIS1 = 4,
    NRF51_GPIOTE = 6,
    NRF51_ADC = 7,
    NRF51_TIMER0 = 8,
    NRF51_TIMER1 = 9,
    NRF51_TIMER2 = 10,
    NRF51_RTC0 = 11,
    NRF51_TEMP = 12,
    NRF51_RNG = 13,
    NRF51_ECB = 14,
    NRF51_CCM = 15,
    NRF51_AAR = 15,
    NRF51_WDT = 16,
    NRF51_RTC1 = 17,
    NRF51_QDEC = 18,
    NRF51_LPCOMP = 19,
    NRF51_NVMC = 30,
    NRF51_PPI = 31,
};

#define NRF51_GPIO_RANGE_IRQ_ID 32

enum nrf51_clock
{
  NRF51_CLOCK_LF,
  NRF51_CLOCK_LF_CALIBRATED,
  NRF51_CLOCK_HF,
#if CONFIG_DRIVER_NRF51_CLOCK_HFCLK_FREQ != 0
  NRF51_CLOCK_HF_PRECISE,
#else
  NRF51_CLOCK_HF_PRECISE = NRF51_CLOCK_HF,
#endif
  NRF51_CLOCK_EP_COUNT,
};

enum nrf51_xo
{
  NRF51_LFXO,
  NRF51_HFXO,
};

enum {
  NRF51_BLE_RADIO_IRQ_RADIO,
  NRF51_BLE_RADIO_IRQ_TIMER,
  NRF51_BLE_RADIO_IRQ_RTC,
  NRF51_BLE_RADIO_IRQ_COUNT,
};

enum {
  NRF51_BLE_RADIO_CLK_SLEEP,
  NRF51_BLE_RADIO_CLK_RADIO,
  NRF51_BLE_RADIO_CLK_COUNT,
};

#endif
