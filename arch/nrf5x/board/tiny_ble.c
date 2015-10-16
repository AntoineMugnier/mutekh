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
# include <device/class/uart.h>
# include <device/class/gpio.h>
# include <device/class/i2c.h>
# include <device/valio/adc.h>
# include <device/resources.h>
# include <device/irq.h>
# include <arch/nrf5x/ids.h>
# include <arch/nrf5x/adc.h>
# include <device/class/clock.h>
#endif

#ifdef CONFIG_DRIVER_NRF5X_UART

DEV_DECLARE_STATIC(uart_dev, "uart0", 0, nrf5x_uart_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_UART0),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_UART0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_UART(1000000, 8, DEV_UART_PARITY_NONE, 1, 1, 0),
                   DEV_STATIC_RES_IOMUX("rts", 0, 8, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, 9, 0, 0),
                   DEV_STATIC_RES_IOMUX("cts", 0, 10, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, 11, 0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_GPIO) && defined(CONFIG_DRIVER_BUTTON_SET)

DEV_DECLARE_STATIC(keyboard_dev, "keyboard", 0, button_set_drv,
                   DEV_STATIC_RES_DEV_PARAM("icu", "/gpio"),
                   DEV_STATIC_RES_IRQ(0, NRF_GPIO_RANGE_IRQ_ID, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IO(17, 17),
                   DEV_STATIC_RES_BLOB_PARAM("mask", dev_gpio_mask1),
                   DEV_STATIC_RES_UINT_PARAM("active", 0),
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_I2C)

DEV_DECLARE_STATIC(i2c_dev, "i2c0", 0, nrf5x_i2c_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TWI0),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TWI0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX(",scl", 0, 19, 0, 0),
                   DEV_STATIC_RES_IOMUX(",sda", 0, 18, 0, 0),
                   );

#endif

#if defined(CONFIG_DRIVER_MPU6505)

DEV_DECLARE_STATIC(accgyr0_dev, "accgyr0", 0, mpu6505_drv,
                   DEV_STATIC_RES_DEV_PARAM("bus", "/i2c0"),
                   DEV_STATIC_RES_I2C_ADDRESS(0x68),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/gpio"),
                   DEV_STATIC_RES_IRQ(0, 20, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_PARAM("timer", "/rtc1"),
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_GPIO_PWM)

DEV_DECLARE_STATIC(pwm_dev, "leds", 0, nrf5x_gpio_pwm_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TIMER2),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, NRF5X_TIMER2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("_p0", 0, 22, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p1", 0, 21, 0, 0),
                   DEV_STATIC_RES_IOMUX("_p2", 0, 23, 0, 0),
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_ADC)

DEV_DECLARE_STATIC(adc_dev, "adc", 0, nrf5x_adc_drv,
  NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_ADC),
  DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
  DEV_STATIC_RES_IRQ(0, NRF5X_ADC, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
  DEV_STATIC_RES_UINT_ARRAY_PARAM("config",
    0,
    0,
    // Input is Vcc * 2.2 / 12.2,  i.e. 0 -> 3.3V in 0 -> .60V
    // So, with 1.2V reference, we have 0 -> 3.3V in 0 -> 256 range
    // And (ADC_OUT - 156) maps 100 percents from 2.01 V to 3.3V
    NRF_ADC_CONFIG_RES_10BIT | NRF_ADC_CONFIG_INPSEL_RAW | NRF_ADC_CONFIG_REFSEL_1_2V,
    0,
    0,
    0,
    0,
    0,
    0),
  );

#endif
