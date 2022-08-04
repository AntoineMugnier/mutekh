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

#include <arch/nrf5x/ids.h>

#ifdef CONFIG_DEVICE
# include <device/class/iomux.h>
# include <device/resource/uart.h>
# include <device/class/cmu.h>
# include <device/class/timer.h>
# include <device/class/gpio.h>
# include <device/resources.h>
# include <device/irq.h>
#endif

#ifdef CONFIG_DRIVER_NRF5X_CLOCK

DEV_DECLARE_STATIC(clock_dev, "clock", 0, nrf5x_clock_drv,
  // Muxes
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFXO, NRF_CLOCK_SRC_HFCLK, 0b100, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFXO, NRF_CLOCK_SRC_LFCLK, 0b110, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFRC, NRF_CLOCK_SRC_LFCLK, 0b001, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFRC, NRF_CLOCK_SRC_HFCLK, 0b011, 1, 1),

  // Oscillators
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFXO, 0b111, 32768, 1, 2, 15), // 20ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFRC, 0b111, 32768, 1, 2, 25), // 2%
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFXO, 0b111, 16000000, 1, 7, 15), // 31ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFRC, 0b111, 16000000, 1, 2, 24), // 1%

  NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_CLOCK),
  DEV_STATIC_RES_DEV_ICU("/cpu"),
  DEV_STATIC_RES_IRQ(0, NRF5X_CLOCK, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
  );

#endif

#ifdef CONFIG_DRIVER_NRF5X_UART

DEV_DECLARE_STATIC(uart_dev, "uart0", 0, nrf5x_uart_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_UART0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_UART0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_UART(1000000, 8, DEV_UART_PARITY_NONE, 1, 1),
                   DEV_STATIC_RES_IOMUX("rts", 0, 8, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, 9, 0, 0),
                   DEV_STATIC_RES_IOMUX("cts", 0, 10, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, 11, 0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_GPIO_PWM)

DEV_DECLARE_STATIC(led_pwm_dev, "led_pwm", 0, nrf5x_gpio_pwm_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TIMER2),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TIMER2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("_p0", 0, 21, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p1", 0, 22, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p2", 0, 23, 0, 0),
                   );

# if defined(CONFIG_DRIVER_RGB24_PWM)

DEV_DECLARE_STATIC(led_dev, "led", 0, rgb24_pwm_drv,
                   DEV_STATIC_RES_DEVCLASS_PARAM("pwm", "/led_pwm", DRIVER_CLASS_PWM),
                   DEV_STATIC_RES_UINT_PARAM("hz", 32),
                   );

# endif
#elif defined(CONFIG_DRIVER_RGB24_GPIO)

DEV_DECLARE_STATIC(led_dev, "led", 0, rgb24_gpio_drv,
                   DEV_STATIC_RES_DEV_TIMER("/rtc* /timer*"),
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_GPIO("_red", 21, 1),
                   DEV_STATIC_RES_GPIO("_green", 22, 1),
                   DEV_STATIC_RES_GPIO("_blue", 23, 1),
                   DEV_STATIC_RES_UINT_PARAM("active_mask", 0),
                   DEV_STATIC_RES_UINT_PARAM("hz", 32),
                   );

#endif
