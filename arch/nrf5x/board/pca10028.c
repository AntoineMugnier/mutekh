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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2015
*/

#include <device/resources.h>
#include <device/class/iomux.h>
#include <device/class/uart.h>
#include <device/class/gpio.h>
#include <device/class/cmu.h>
#include <arch/nrf5x/ids.h>

#ifdef CONFIG_DRIVER_NRF5X_CLOCK

DEV_DECLARE_STATIC(clock_dev, "clock", 0, nrf5x_clock_drv,
  // Muxes
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFXO, NRF_CLOCK_SRC_HFCLK, 0b100, 1, 1),
#if defined(CONFIG_DRIVER_NRF5X_CLOCK_LFRC_CAL)
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFRC, NRF_CLOCK_SRC_LFCLK, 0b111, 1, 1),
#else
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFXO, NRF_CLOCK_SRC_LFCLK, 0b110, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFRC, NRF_CLOCK_SRC_LFCLK, 0b001, 1, 1),
#endif
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFRC, NRF_CLOCK_SRC_HFCLK, 0b011, 1, 1),

  // Oscillators
#if defined(CONFIG_DRIVER_NRF5X_CLOCK_LFRC_CAL)
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFRC, 0b110, 32768, 1, 7, 18), // 246ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFRC, 0b001, 32768, 1, 2, 25), // 2%
#else
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFXO, 0b111, 32768, 1, 2, 15), // 20ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFRC, 0b111, 32768, 1, 2, 25), // 2%
#endif
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFXO, 0b111, 16000000, 1, 7, 15), // 31ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFRC, 0b111, 16000000, 1, 2, 24), // 1%

  NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_CLOCK),
  DEV_STATIC_RES_DEV_ICU("/cpu"),
  DEV_STATIC_RES_IRQ(0, NRF5X_CLOCK, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),

#if defined(CONFIG_DRIVER_NRF5X_CLOCK_LFRC_CAL)
  NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TEMP),
  DEV_STATIC_RES_IRQ(1, NRF5X_TEMP, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
#endif
  );

#endif

#ifdef CONFIG_DRIVER_NRF5X_UART

DEV_DECLARE_STATIC(uart_dev, "uart0", 0, nrf5x_uart_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_UART0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_UART0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_UART(1000000, 8, DEV_UART_PARITY_NONE, 1, 1, 0),
                   DEV_STATIC_RES_IOMUX("rts", 0, 8, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, 9, 0, 0),
                   DEV_STATIC_RES_IOMUX("cts", 0, 10, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, 11, 0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_GPIO) && defined(CONFIG_DRIVER_BUTTON_SET)

DEV_DECLARE_STATIC(keyboard_dev, "keyboard", 0, button_set_drv,
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_IRQ(0, NRF_GPIO_RANGE_IRQ_ID, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IO(17, 20),
                   DEV_STATIC_RES_BLOB_PARAM("mask", dev_gpio_mask1),
                   DEV_STATIC_RES_UINT_PARAM("active", 0),
                   );

#endif
