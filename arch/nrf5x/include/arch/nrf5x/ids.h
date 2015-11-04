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

#ifndef ARCH_NRF5X_IDS_H_
#define ARCH_NRF5X_IDS_H_

#include <hexo/types.h>
#include <hexo/decls.h>

#define NRF_PERIPHERAL_ADDR(no) (0x40000000 | ((uintptr_t)(no) << 12))
#define NRF_PERIPHERAL_ID(addr) ((uint8_t)((addr) >> 12) & 0x7f)

#define NRF_STATIC_RES_PERIPHERAL_MEM(x) DEV_STATIC_RES_MEM(NRF_PERIPHERAL_ADDR((x)), NRF_PERIPHERAL_ADDR((x) + 1))

ALWAYS_INLINE uintptr_t nrf_peripheral_addr(uint8_t no)
{
    return 0x40000000 | ((uintptr_t)no << 12);
}

enum nrf5x_peripheral {
    NRF5X_POWER = 0,
    NRF5X_CLOCK = 0,
    NRF5X_RADIO = 1,
    NRF5X_UART0 = 2,
    NRF5X_SPI0 = 3,
    NRF5X_TWI0 = 3,
    NRF5X_SPI1 = 4,
    NRF5X_TWI1 = 4,
    NRF5X_SPIS1 = 4,
    NRF5X_GPIOTE = 6,
    NRF5X_ADC = 7,
    NRF5X_TIMER0 = 8,
    NRF5X_TIMER1 = 9,
    NRF5X_TIMER2 = 10,
    NRF5X_RTC0 = 11,
    NRF5X_TEMP = 12,
    NRF5X_RNG = 13,
    NRF5X_ECB = 14,
    NRF5X_CCM = 15,
    NRF5X_AAR = 15,
    NRF5X_WDT = 16,
    NRF5X_RTC1 = 17,
    NRF5X_QDEC = 18,
    NRF5X_LPCOMP = 19,
    NRF5X_NVMC = 30,
    NRF5X_PPI = 31,

#if defined(CONFIG_ARCH_NRF52)
    NRF5X_BPROT = 0,
    NRF5X_UARTE0 = 2,
    NRF5X_SPIM0 = 3,
    NRF5X_TWIM0 = 3,
    NRF5X_TWIS0 = 3,
    NRF5X_SPIS0 = 3,
    NRF5X_SPIM1 = 4,
    NRF5X_TWIM1 = 4,
    NRF5X_TWIS1 = 4,
    NRF5X_NRFT = 5,
    NRF5X_COMP = 19,
    NRF5X_EGU0 = 20,
    NRF5X_SWI0 = 20,
    NRF5X_EGU1 = 21,
    NRF5X_SWI1 = 21,
    NRF5X_EGU2 = 22,
    NRF5X_SWI2 = 22,
    NRF5X_EGU3 = 23,
    NRF5X_SWI3 = 23,
    NRF5X_EGU4 = 24,
    NRF5X_SWI4 = 24,
    NRF5X_EGU5 = 25,
    NRF5X_SWI5 = 25,
    NRF5X_TIMER3 = 26,
    NRF5X_TIMER4 = 27,
    NRF5X_PWM0 = 28,
    NRF5X_PDM = 29,
    NRF5X_MWU = 32,
    NRF5X_PWM1 = 33,
    NRF5X_PWM2 = 34,
    NRF5X_SPI2 = 35,
    NRF5X_SPIS2 = 35,
    NRF5X_SPIM2 = 35,
    NRF5X_RTC2 = 36,
    NRF5X_I2S = 37,
#endif
};

#define NRF_GPIO_RANGE_IRQ_ID 32

enum nrf5x_clock
{
  NRF_CLOCK_LF,
  NRF_CLOCK_LF_PRECISE,
  NRF_CLOCK_HF,
#if CONFIG_DRIVER_NRF5X_CLOCK_HFCLK_FREQ != 0
  NRF_CLOCK_HF_PRECISE,
#else
  NRF_CLOCK_HF_PRECISE = NRF_CLOCK_HF,
#endif
  NRF_CLOCK_EP_COUNT,
};

enum nrf5x_xo
{
  NRF5X_LFXO,
  NRF5X_HFXO,
};

enum {
  NRF5X_BLE_RADIO_IRQ_RADIO,
  NRF5X_BLE_RADIO_IRQ_TIMER,
  NRF5X_BLE_RADIO_IRQ_RTC,
  NRF5X_BLE_RADIO_IRQ_COUNT,
};

enum {
  NRF5X_BLE_RADIO_CLK_SLEEP,
  NRF5X_BLE_RADIO_CLK_RADIO,
  NRF5X_BLE_RADIO_CLK_COUNT,
};

enum {
  NRF5X_BLE_IRQ_RADIO,
  NRF5X_BLE_IRQ_TIMER,
  NRF5X_BLE_IRQ_RTC,
  NRF5X_BLE_IRQ_COUNT,
};

enum {
  NRF5X_BLE_CLK_SLEEP,
  NRF5X_BLE_CLK_RADIO,
  NRF5X_BLE_CLK_COUNT,
};

enum {
  NRF5X_RADIO_IRQ_RADIO,
  NRF5X_RADIO_IRQ_RTC,
  NRF5X_RADIO_IRQ_COUNT,
};

enum {
  NRF5X_RADIO_CLK_SLEEP,
  NRF5X_RADIO_CLK_RADIO,
  NRF5X_RADIO_CLK_COUNT,
};

#endif