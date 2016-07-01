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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2016
*/

#include <arch/nrf5x/ids.h>

#ifdef CONFIG_DEVICE
# include <device/class/iomux.h>
# include <device/class/uart.h>
# include <device/class/gpio.h>
# include <device/class/timer.h>
# include <device/resources.h>
# include <device/irq.h>
# include <arch/nrf5x/ids.h>
# include <device/class/cmu.h>
# include <device/class/usbdev.h>
#endif

#ifdef CONFIG_DRIVER_NRF5X_CLOCK

DEV_DECLARE_STATIC(clock_dev, "clock", 0, nrf5x_clock_drv,
  // Muxes
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFXO, NRF_CLOCK_SRC_HFCLK, 0b111, 1, 2),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_SRC_HFCLK, NRF_CLOCK_SRC_LFCLK, 0b111, 1, 1),

  // Oscillators
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFRC, 0b111, 32768, 1, 2, 25), // 2%
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFXO, 0b111, 32000000, 1, 7, 15), // 31ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFRC, 0b111, 16000000, 1, 7, 24), // 1.5%

  NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_CLOCK),
  DEV_STATIC_RES_DEV_ICU("/cpu"),
  DEV_STATIC_RES_IRQ(0, NRF5X_CLOCK, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
  );

#endif

#ifdef CONFIG_DRIVER_NRF52_UARTE

DEV_DECLARE_STATIC(uart_dev, "uart0", 0, nrf5x_uarte_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_UARTE0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_UARTE0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_UART(1000000, 8, DEV_UART_PARITY_NONE, 1, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, 20, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, 19, 0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_GPIO) && defined(CONFIG_DRIVER_BUTTON_SET)

static const uint8_t kbd_mask[] = {0x1, 0x1, 0, 0, 0, 0, 0, 0};

DEV_DECLARE_STATIC(keyboard_dev, "keyboard", 0, button_set_drv,
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_IRQ(0, NRF_GPIO_RANGE_IRQ_ID, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IO(3, 3),
                   DEV_STATIC_RES_BLOB_PARAM("mask", kbd_mask),
                   DEV_STATIC_RES_UINT_PARAM("active", 0),
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_GPIO_PWM)

DEV_DECLARE_STATIC(pwm_dev, "leds", 0, nrf5x_gpio_pwm_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TIMER2),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TIMER2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("_p0", 0, 2, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p1", 0, 0, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p2", 0, 1, 0, 0),
                   );

#endif

#if defined(CONFIG_DRIVER_NRF52_SPIM) || defined(CONFIG_DRIVER_NRF5X_SPI)

#if defined(CONFIG_DRIVER_NRF52_SPIM)

DEV_DECLARE_STATIC(spi_dev, "spi1", 0, nrf5x_spim_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_SPIM1),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_DEV_TIMER("/timer1"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_SPIM1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("mosi", 0, 27, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", 0, 28, 0, 0),
                   DEV_STATIC_RES_IOMUX("clk", 0, 30, 0, 0),
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_SPI)

DEV_DECLARE_STATIC(spi_dev, "spi1", 0, nrf5x_spi_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_SPI1),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_DEV_TIMER("/timer1"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_SPI1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("mosi", 0, 27, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", 0, 28, 0, 0),
                   DEV_STATIC_RES_IOMUX("clk", 0, 30, 0, 0),
                   );

#endif

# if defined(CONFIG_DRIVER_USBDEV_MAX3420)

DEV_DECLARE_STATIC(max3420_dev, "max3420", 0, max3420_drv,
                   DEV_STATIC_RES_DEV_PARAM("spi", "/spi*"),
                   DEV_STATIC_RES_DEV_PARAM("gpio", "/gpio"),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/gpio"),
                   DEV_STATIC_RES_IRQ(0, 25, DEV_IRQ_SENSE_FALLING_EDGE, 0, 1),
                   DEV_STATIC_RES_GPIO("rst", 31, 1),
                   DEV_STATIC_RES_GPIO("nirq", 25, 1),
                   DEV_STATIC_RES_GPIO("gpx", 26, 1),
                   DEV_STATIC_RES_UINT_PARAM("gpio-cs-id", 29),
                   );

#  if defined(CONFIG_DRIVER_CHAR_USBDEV_CDC)

DEV_DECLARE_STATIC(usbdev_cdc0, "console", 0, usbdev_cdc_drv,
                   DEV_STATIC_RES_USBDEV_EP_MAP(0, 0x32, 0x01),
                   DEV_STATIC_RES_DEV_PARAM("usb-ctrl", "/max3420")
                   );

#  endif
# endif
#endif
