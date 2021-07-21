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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2020
*/

#include <device/resources.h>
#include <device/class/iomux.h>
#include <device/resource/uart.h>
#include <device/class/gpio.h>
#include <device/class/cmu.h>
#include <arch/nrf5x/ids.h>

#ifdef CONFIG_DRIVER_NRF5X_CLOCK

DEV_DECLARE_STATIC(clock_dev, "clock", 0, nrf5x_clock_drv,
  // Muxes
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFXO,     NRF_CLOCK_SRC_HFCLK, 0b100, 2, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFXO,     NRF_CLOCK_SRC_LFCLK, 0b110, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFRC_ULP, NRF_CLOCK_SRC_LFCLK, 0b001, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFRC,     NRF_CLOCK_SRC_HFCLK, 0b011, 1, 1),

  // Oscillators
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFXO,     0b111, 32768, 1, 2, 15), // 20ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFRC,     0b111, 32768, 1, 2, 25), // 2%
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFRC_ULP, 0b111, 32768, 1, 7, 26), // 7%
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFXO,     0b111, 32000000, 1, 7, 15), // 31ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFRC,     0b111, 64000000, 1, 7, 24), // 1.5%

  NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_CLOCK),
  DEV_STATIC_RES_DEV_ICU("/cpu"),
  DEV_STATIC_RES_IRQ(0, NRF5X_CLOCK, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
  );

#endif

#if defined(CONFIG_DRIVER_NRF52_UARTE) || defined(CONFIG_DRIVER_NRF5X_UART)

DEV_DECLARE_STATIC(uart_dev, "uart0", 0,
# if defined(CONFIG_DRIVER_NRF52_UARTE) && \
  !(defined(CONFIG_DRIVER_NRF5X_PRINTK) && CONFIG_MUTEK_PRINTK_ADDR == NRF5X_UARTE0)
                   nrf5x_uarte_drv,
# else
                   nrf5x_uart_drv,
# endif
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_UARTE0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_UART0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_UART(1000000, 8, DEV_UART_PARITY_NONE, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, 13, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, 15, 0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_GPIO) && defined(CONFIG_DRIVER_BUTTON_SET)

DEV_DECLARE_STATIC(keyboard_dev, "keyboard", 0, button_set_drv,
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_GPIO("pins", 38, 1),
                   DEV_STATIC_RES_UINT_PARAM("active", 0),
                   );

#endif

// On-board equipment, all active low
// LED1: P0.06
// LED2 (RGB): P0.08 P1.09 P0.12
// SW1: P1.06

// GPIO Headers, front facing:
//
//           Tx,    Rx                                      NFC Capable
// USB    P0.13, P.015, P0.17, P0.20, P0.22, P0.24, P1.00, P0.09, P0.10, GND
// Port    VBUS,   VDD,   GND, P0.31, P0.29, P0.02, P1.15, P1.13, P1.10, GND
//
