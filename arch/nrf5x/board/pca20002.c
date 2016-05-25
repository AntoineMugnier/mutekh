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

#include <device/resources.h>
#include <device/class/iomux.h>
#include <device/class/i2c.h>
#include <device/class/uart.h>
#include <device/class/gpio.h>
#include <device/class/cmu.h>
#include <arch/nrf5x/ids.h>

DEV_DECLARE_STATIC(clock_dev, "clock", 0, nrf5x_clock_drv,
  // Muxes
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFXO, NRF_CLOCK_SRC_HFCLK, 0b100, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_HFRC, NRF_CLOCK_SRC_HFCLK, 0b011, 1, 1),
  DEV_STATIC_RES_CMU_MUX(NRF_CLOCK_OSC_LFRC, NRF_CLOCK_SRC_LFCLK, 0b111, 1, 1),

  // Oscillators
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFRC, 0b110, 32768, 1, 0, 19), // 262ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_LFRC, 0b001, 32768, 1, 2, 25), // 2%
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFXO, 0b111, 16000000, 1, 7, 15), // 31ppm
  DEV_STATIC_RES_CMU_OSC_ACC(NRF_CLOCK_OSC_HFRC, 0b111, 16000000, 1, 2, 24), // 1%

  NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_CLOCK),
  DEV_STATIC_RES_DEV_ICU("/cpu"),
  DEV_STATIC_RES_IRQ(0, NRF5X_CLOCK, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),

  NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TEMP),
  DEV_STATIC_RES_IRQ(1, NRF5X_TEMP, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
  );

#if defined(CONFIG_DRIVER_NRF5X_UART)

DEV_DECLARE_STATIC(uart_dev, "uart0", 0, nrf5x_uart_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_UART0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),,
                   DEV_STATIC_RES_IRQ(0, NRF5X_UART0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_UART(1000000, 8, DEV_UART_PARITY_NONE, 1, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, 20, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, 19, 0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_I2C)

DEV_DECLARE_STATIC(i2c_dev, "i2c0", 0, nrf5x_i2c_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_TWI0),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),,
                   DEV_STATIC_RES_IRQ(0, NRF5X_TWI0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("scl", 0, 8, 0, 0),
                   DEV_STATIC_RES_IOMUX("sda", 0, 7, 0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_NRF5X_SPI)

DEV_DECLARE_STATIC(spi_dev, "spi0", 0, nrf5x_spi_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF5X_SPI1),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),,
                   DEV_STATIC_RES_IRQ(0, NRF5X_SPI1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("clk", 0, 9, 0, 0),
                   DEV_STATIC_RES_IOMUX("mosi", 0, 10, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", 0, 11, 0, 0)
                   );

#endif

// PCA9557 (I2C I/O extender) named U2
// I2C bus, address 0x18
// Led1: R1 on U2.7, G1 on U2.6
// Led2: R2 on U2.5, G2 on U2.4

#if defined(CONFIG_DRIVER_NRF5X_I2C) && defined(CONFIG_DRIVER_PCA9557)

DEV_DECLARE_STATIC(gpio1_dev, "gpio1", 0, pca9557_drv,
                   DEV_STATIC_RES_DEV_BUS("/i2c0"),
                   DEV_STATIC_RES_I2C_ADDRESS(0x18)
                   );

#endif

// MPU 6050
// I2C bus, address 0x68
// Int on GP_IRQ_0 (P0.15)

#if defined(CONFIG_DRIVER_NRF5X_I2C) && defined(CONFIG_DRIVER_MPU6050)

DEV_DECLARE_STATIC(motion0_dev, "motion0", 0, mpu6050_drv,
                   DEV_STATIC_RES_DEV_BUS("/i2c0"),
                   DEV_STATIC_RES_I2C_ADDRESS(0x68),
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_IRQ(0, 15, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                   );

#endif

// Synaptics touchpad
// I2C bus, address 0x20
// Int on WU_IRQ_0 (P0.18)

/*
DEV_DECLARE_STATIC(touchpad0_dev, "touchpad0", 0, rmi4_drv,
  DEV_STATIC_RES_DEV_BUS("/i2c0"),
  DEV_STATIC_RES_I2C_ADDRESS(0x20),
  DEV_STATIC_RES_DEV_ICU("/gpio"),
  DEV_STATIC_RES_IRQ(0, 18, DEV_IRQ_SENSE_LOW_LEVEL, 0, 1)
  );
*/

// Keyboard
// 8 columns on P0.29-P0.30,P0.0-P0.5
// 8 rows on P0.21-P0.28
// Merge of rows (through diodes) on WU_IRQ_2 (P0.16)

#if defined(CONFIG_DRIVER_NRF5X_GPIO) && defined(CONFIG_DRIVER_MATRIX_KEYBOARD)

static const uint64_t columns_mask = endian_le64(0x6000003f);
static const uint64_t rows_mask    = endian_le64(0xff);

DEV_DECLARE_STATIC(keyboard_dev, "keyboard", 0, matrix_keyboard_drv,
                   DEV_STATIC_RES_DEV_TIMER("/rtc1"),
                   DEV_STATIC_RES_DEV_ICU("/gpio"),,
                   DEV_STATIC_RES_IRQ(0, NRF_GPIO_RANGE_IRQ_ID, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
                   DEV_STATIC_RES_IO(0, 31),
                   DEV_STATIC_RES_BLOB_PARAM("mask", &columns_mask),
                   DEV_STATIC_RES_IO(21, 28),
                   DEV_STATIC_RES_BLOB_PARAM("mask", &rows_mask),
                   DEV_STATIC_RES_UINT_PARAM("row_delay", 2),
                   DEV_STATIC_RES_UINT_PARAM("refresh_period", 33),
                   DEV_STATIC_RES_UINT_PARAM("refresh_max", 300),
                   );

#endif

// Lis3D
// SPI bus
// CS on CSn0 (P0.13)
// Int1 on WU_IRQ_1 (P0.17)
// Int2 on GP_IRQ_1 (P0.14)

// External expansion board
// I2C
// SPI_Clk (P0.9)
// SPI_Mosi (P0.10)
// SPI_Miso (P0.11)
// CSn1 (P0.12)
