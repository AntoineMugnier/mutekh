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

#include <arch/nrf5x/ids.h>

#ifdef CONFIG_DEVICE
# include <device/class/iomux.h>
# include <device/resource/uart.h>
# include <device/class/gpio.h>
# include <device/class/timer.h>
# if defined(CONFIG_DRIVER_NRF5X_I2C)
#  include <device/class/i2c.h>
# endif
# if defined(CONFIG_DRIVER_NRF5X_ADC)
#  include <device/class/valio.h>
#  include <device/valio/adc.h>
#  include <arch/nrf5x/adc.h>
# endif
# include <device/resources.h>
# include <device/irq.h>
# include <arch/nrf5x/ids.h>
# include <device/class/cmu.h>
#endif

/*
  Full design available on http://nipo.ssji.net/hardware/2016-02-23-nrf51-clock/
*/

#ifdef CONFIG_DRIVER_NRF5X_CLOCK

DEV_DECLARE_STATIC(clock_dev, "clock", 0, nrf5x_clock_drv,
  // Muxes
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFRC, NRF_CLOCK_SRC_HFCLK, 0b011, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFXO, NRF_CLOCK_SRC_HFCLK, 0b100, 1, 2),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFRC, NRF_CLOCK_SRC_LFCLK, 0b001, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFXO, NRF_CLOCK_SRC_LFCLK, 0b110, 1, 1),

  // Oscillators
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFXO, 0b111, 32768, 1, 2, 15), // 20ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFRC, 0b111, 32768, 1, 2, 25), // 2%
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFXO, 0b111, 32000000, 1, 7, 15), // 31ppm
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
                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_UART(1000000, 8, DEV_UART_PARITY_NONE, 1, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, 5, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, 6, 0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_GPIO) && defined(CONFIG_DRIVER_BUTTON_SET)

static const uint8_t kbd_mask[] = {0x1, 0x2, 0, 0, 0, 0, 0, 0};

DEV_DECLARE_STATIC(keyboard_dev, "keyboard", 0, button_set_drv,
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_DEV_GPIO("/gpio"),
                   DEV_STATIC_RES_IRQ(0, NRF_GPIO_RANGE_IRQ_ID, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IO(16, 25),
                   DEV_STATIC_RES_BLOB_PARAM("mask", kbd_mask),
                   DEV_STATIC_RES_UINT_PARAM("active", 0),
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_I2C)

DEV_DECLARE_STATIC(i2c_dev, "i2c0", 0, nrf5x_i2c_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TWI0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TWI0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX(",scl", 0, 13, 0, 0),
                   DEV_STATIC_RES_IOMUX(",sda", 0, 12, 0, 0),
                   );

#endif

#if defined(CONFIG_DRIVER_MAX44009)

DEV_DECLARE_STATIC(light_sensor_dev, "light0", 0, mpu6505_drv,
                   DEV_STATIC_RES_DEV_I2C("/i2c0"),
                   DEV_STATIC_RES_I2C_ADDRESS(0x4a),
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_IRQ(0, 14, DEV_IRQ_SENSE_LOW_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_TIMER("/rtc1"),
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_GPIO_PWM)

DEV_DECLARE_STATIC(pwm_dev, "leds", 0, nrf5x_gpio_pwm_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TIMER2),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TIMER2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("_p0", 0, 17, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p1", 0, 18, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p2", 0, 23, 0, 0),
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_ADC)

DEV_DECLARE_STATIC(adc_dev, "adc", 0, nrf5x_adc_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_ADC),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_ADC, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_UINT_ARRAY_PARAM("config",
                                                   // Take VBat/3 against 1.2V
                                                   // 1.8V is 512
                                                   // 2V is 569
                                                   // 2.5V is 711
                                                   // 3V is 853
                                                   // (ADC_Val - 569) / 2) is 0-100 for 2V - 2.7V
                                                   NRF_ADC_CONFIG_RES_10BIT
                                                   | NRF_ADC_CONFIG_INPSEL_VDD_1_3
                                                   | NRF_ADC_CONFIG_REFSEL_1_2V,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   0,
                                                   0),
                   );

#endif
