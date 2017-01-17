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
#include <device/class/i2c.h>
#include <arch/nrf5x/ids.h>

#ifdef CONFIG_DRIVER_NRF5X_CLOCK

DEV_DECLARE_STATIC(clock_dev, "clock", 0, nrf5x_clock_drv,
  // Muxes
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFXO, NRF_CLOCK_SRC_HFCLK, 0b100, 2, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFXO, NRF_CLOCK_SRC_LFCLK, 0b110, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFRC, NRF_CLOCK_SRC_LFCLK, 0b001, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFRC, NRF_CLOCK_SRC_HFCLK, 0b011, 1, 1),

  // Oscillators
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFXO, 0b111, 32768, 1, 2, 15), // 20ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFRC, 0b111, 32768, 1, 2, 25), // 2%
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFXO, 0b111, 32000000, 1, 7, 15), // 31ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFRC, 0b111, 16000000, 1, 7, 24), // 1.5%

  NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_CLOCK),
  DEV_STATIC_RES_DEV_ICU("/cpu"),
  DEV_STATIC_RES_IRQ(0, NRF5X_CLOCK, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
  );

#endif

#if defined(CONFIG_DRIVER_NRF52_UARTE)

DEV_DECLARE_STATIC(uart_dev, "uart0", 0, nrf5x_uarte_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_UARTE0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_UART0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_UART(1000000, 8, DEV_UART_PARITY_NONE, 1, 1, 0),
                   DEV_STATIC_RES_IOMUX("rts", 0, 5, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, 6, 0, 0),
                   DEV_STATIC_RES_IOMUX("cts", 0, 7, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, 8, 0, 0)
                   );

#elif defined(CONFIG_DRIVER_NRF5X_UART)

DEV_DECLARE_STATIC(uart_dev, "uart0", 0, nrf5x_uart_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_UART0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_UART0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_UART(1000000, 8, DEV_UART_PARITY_NONE, 1, 1, 0),
                   DEV_STATIC_RES_IOMUX("rts", 0, 5, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, 6, 0, 0),
                   DEV_STATIC_RES_IOMUX("cts", 0, 7, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, 8, 0, 0)
                   );

#endif

#if defined(CONFIG_NRF5X_BOARD_ARDUINO_MODE)
# if defined(CONFIG_DRIVER_NRF5X_SPI)

DEV_DECLARE_STATIC(spi_dev, "spi0", 0, nrf5x_spi_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_SPI1),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_SPI1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("clk", 0, 25, 0, 0),
                   DEV_STATIC_RES_IOMUX("mosi", 0, 23, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", 0, 24, 0, 0),
                   DEV_STATIC_RES_DEV_TIMER("rtc* timer*"),
                   );
# endif

# if defined(CONFIG_DRIVER_NRF5X_I2C)

DEV_DECLARE_STATIC(i2c0_dev, "i2c0", 0, nrf5x_i2c_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TWI0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TWI0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_DEV_TIMER("/timer1"),
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_I2C_BITRATE(400000),
                   DEV_STATIC_RES_IOMUX("scl", 0, 27, 0, 0),
                   DEV_STATIC_RES_IOMUX("sda", 0, 26, 0, 0)
                   );

#  if defined(CONFIG_DRIVER_GPIO_PCAL6408A)
DEV_DECLARE_STATIC(pcal6408a_dev, "gpio1", 0, pcal6408a_drv,
                   DEV_STATIC_RES_I2C_ADDR("i2c0", 0x20),
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_IRQ(0, 17, DEV_IRQ_SENSE_LOW_LEVEL, 0, 1),
                   );

#   if defined(CONFIG_DRIVER_NRF5X_GPIO) && defined(CONFIG_DRIVER_BUTTON_SET)
DEV_DECLARE_STATIC(keyboard_dev, "keyboard", 0, button_set_drv,
                     DEV_STATIC_RES_DEV_GPIO("/gpio1"),
                     DEV_STATIC_RES_GPIO("pins", 0, 4),
                     DEV_STATIC_RES_UINT_PARAM("active", 0),
                     );
#   endif
#  endif
# endif
#else
# if defined(CONFIG_DRIVER_NRF5X_GPIO) && defined(CONFIG_DRIVER_BUTTON_SET)
DEV_DECLARE_STATIC(keyboard_dev, "keyboard", 0, button_set_drv,
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_GPIO("pins", 13, 4),
                   DEV_STATIC_RES_UINT_PARAM("active", 0),
                   );
# endif
#endif
