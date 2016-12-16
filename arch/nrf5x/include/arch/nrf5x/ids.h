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

/**
   @file
   @short nRF51/nRF52 peripheral IDs
   @module {Platforms::nRF5X platform}

   nRF5x devices all look the same and regular.  Device at address
   0x4000000 has number 0 and interrupt 0, device at address
   0x40001000 has number 1 and interrupt 1, and so on.

   @ref nrf5x_peripheral_id_e defines available peripheral IDs. All of
   them map to an address range, and an interrupt source to NVIC.

   @ref #NRF_PERIPHERAL_ADDR and @ref nrf_peripheral_addr allow to
   retrieve device base address, @ref #NRF_STATIC_RES_PERIPHERAL_MEM
   is a shortcut for declaring a @ref #DEV_STATIC_RES_MEM for a
   device.
*/

#if !defined(CONFIG_ARCH_NRF5X)
# error You should not include this header on non-nRF builds
#endif

#include <hexo/types.h>
#include <hexo/decls.h>

/**
   @this statically computes the device base address from its @ref
   {nrf5x_peripheral} {device number}.

   @see nrf_peripheral_addr
 */
#define NRF_PERIPHERAL_ADDR(no) (0x40000000 | ((uintptr_t)(no) << 12))

/**
   @this statically retrieves device ID from its base address.
   Address is assumed to be existing.
 */
#define NRF_PERIPHERAL_ID(addr) ((uint8_t)((addr) >> 12) & 0x7f)

/**
   @this expands to @ref #DEV_STATIC_RES_MEM for a given peripheral
   ID.
 */
#define NRF_STATIC_RES_PERIPHERAL_MEM(x) DEV_STATIC_RES_MEM(NRF_PERIPHERAL_ADDR((x)), NRF_PERIPHERAL_ADDR((x) + 1))

/**
   @this computes the device base address from its @ref
   {nrf5x_peripheral} {device number}.

   @see NRF_PERIPHERAL_ADDR
 */
ALWAYS_INLINE uintptr_t nrf_peripheral_addr(uint8_t no)
{
    return 0x40000000 | ((uintptr_t)no << 12);
}

/**
   @this defines the list of peripheral IDs for nRF51/nRF52 chips.
 */
enum nrf5x_peripheral_id_e
{
    NRF5X_POWER  = 0,
    NRF5X_CLOCK  = 0,
    NRF5X_RADIO  = 1,
    NRF5X_UART0  = 2,
    NRF5X_SPI0   = 3,
    NRF5X_TWI0   = 3,
    NRF5X_SPI1   = 4,
    NRF5X_TWI1   = 4,
    NRF5X_SPIS1  = 4,
    NRF5X_GPIOTE = 6,
    NRF5X_ADC    = 7,
    NRF5X_TIMER0 = 8,
    NRF5X_TIMER1 = 9,
    NRF5X_TIMER2 = 10,
    NRF5X_RTC0   = 11,
    NRF5X_TEMP   = 12,
    NRF5X_RNG    = 13,
    NRF5X_ECB    = 14,
    NRF5X_CCM    = 15,
    NRF5X_AAR    = 15,
    NRF5X_WDT    = 16,
    NRF5X_RTC1   = 17,
    NRF5X_QDEC   = 18,
    NRF5X_LPCOMP = 19,
    NRF5X_NVMC   = 30,
    NRF5X_PPI    = 31,

#if 52000 <= CONFIG_NRF5X_MODEL && CONFIG_NRF5X_MODEL <= 52999
    NRF5X_BPROT  = 0,
    NRF5X_UARTE0 = 2,
    NRF5X_SPIM0  = 3,
    NRF5X_TWIM0  = 3,
    NRF5X_TWIS0  = 3,
    NRF5X_SPIS0  = 3,
    NRF5X_SPIM1  = 4,
    NRF5X_TWIM1  = 4,
    NRF5X_TWIS1  = 4,
    NRF5X_NFCT   = 5,
    NRF5X_SAADC  = 7,
    NRF5X_COMP   = 19,
    NRF5X_EGU0   = 20,
    NRF5X_SWI0   = 20,
    NRF5X_EGU1   = 21,
    NRF5X_SWI1   = 21,
    NRF5X_EGU2   = 22,
    NRF5X_SWI2   = 22,
    NRF5X_EGU3   = 23,
    NRF5X_SWI3   = 23,
    NRF5X_EGU4   = 24,
    NRF5X_SWI4   = 24,
    NRF5X_EGU5   = 25,
    NRF5X_SWI5   = 25,
    NRF5X_TIMER3 = 26,
    NRF5X_TIMER4 = 27,
    NRF5X_PWM0   = 28,
    NRF5X_PDM    = 29,
    NRF5X_MWU   = 32,
    NRF5X_PWM1  = 33,
    NRF5X_PWM2  = 34,
    NRF5X_SPI2  = 35,
    NRF5X_SPIS2 = 35,
    NRF5X_SPIM2 = 35,
    NRF5X_RTC2  = 36,
    NRF5X_I2S   = 37,
#endif
};

/**
   @this is the @em {input range} interrupt endpoint ID in GPIO
   driver.
 */
#define NRF_GPIO_RANGE_IRQ_ID 32

/**
   @this defines identifiers for nRF51/nRF52 clocks.
 */
enum nrf5x_clock_id_e
{
  NRF_CLOCK_SRC_LFCLK,
  NRF_CLOCK_SRC_HFCLK,
  NRF_CLOCK_SRC_COUNT,
  NRF_CLOCK_OSC_LFRC = NRF_CLOCK_SRC_COUNT,
  NRF_CLOCK_OSC_HFRC,
  NRF_CLOCK_OSC_HFXO,
  NRF_CLOCK_OSC_LFXO,
  NRF_CLOCK_NODE_COUNT,
};

#define NRF52_SAADC_PIN_REF_0_6   0
#define NRF52_SAADC_PIN_REF_VDD_4 0x8

#define NRF52_SAADC_PIN_CONFIG(gain_num, gain_denom, ref) (0          \
   | ((gain_num) == 1 ? (6-(gain_denom)) : ((gain_num) == 2 ? 6 : 7)) \
   | NRF52_SAADC_PIN_REF_ ## ref                                      \
   )

#define NRF52_SAADC_PIN_CONFIG_EXTRACT(x) ((((x) & 0x7) << 8) | (((x) & 0x8) << 11))

#endif
