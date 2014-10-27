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
#include <arch/nrf51/ids.h>

#if defined(CONFIG_DRIVER_NRF51_UART)

DEV_DECLARE_STATIC(uart_dev, "uart0", 0, nrf51_uart_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF51_UART0),
                   DEV_STATIC_RES_IRQ(0, NRF51_UART0, 0, "/cpu"),
                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_UART(1000000, 8, DEV_UART_PARITY_NONE, 1, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, 20, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, 19, 0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_NRF51_I2C)

DEV_DECLARE_STATIC(i2c_dev, "i2c0", 0, nrf51_i2c_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF51_TWI0),
                   DEV_STATIC_RES_IRQ(0, NRF51_TWI0, 0, "/cpu"),
                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("scl", 0, 8, 0, 0),
                   DEV_STATIC_RES_IOMUX("sda", 0, 7, 0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_NRF51_SPI)

DEV_DECLARE_STATIC(spi_dev, "spi0", 0, nrf51_spi_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF51_SPI1),
                   DEV_STATIC_RES_IRQ(0, NRF51_SPI1, 0, "/cpu"),
                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("clk", 0, 9, 0, 0),
                   DEV_STATIC_RES_IOMUX("mosi", 0, 10, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", 0, 11, 0, 0)
                   );

#endif

// PCA9557 (I2C I/O extender) named U2
// I2C bus, address 0x18
// Led1: R1 on U2.7, G1 on U2.6
// Led2: R2 on U2.5, G2 on U2.4

#if defined(CONFIG_DRIVER_NRF51_I2C) && defined(CONFIG_DRIVER_PCA9557)

DEV_DECLARE_STATIC(gpio1_dev, "gpio1", 0, pca9557_drv,
                   DEV_STATIC_RES_DEV_PARAM("bus", "/i2c0"),
                   DEV_STATIC_RES_I2C_ADDRESS(0x18)
                   );

#endif

// MPU 6050
// I2C bus, address 0x68
// Int on GP_IRQ_0 (P0.15)

#if defined(CONFIG_DRIVER_NRF51_I2C) && defined(CONFIG_DRIVER_MPU6050)

DEV_DECLARE_STATIC(motion0_dev, "motion0", 0, mpu6050_drv,
  DEV_STATIC_RES_DEV_PARAM("bus", "/i2c0"),
  DEV_STATIC_RES_I2C_ADDRESS(0x68),
  DEV_STATIC_RES_IRQ(0, 15, 0, "/gpio")
  );

#endif

// Synaptics touchpad
// I2C bus, address 0x20
// Int on WU_IRQ_0 (P0.18)

/*
DEV_DECLARE_STATIC(touchpad0_dev, "touchpad0", 0, rmi4_drv,
  DEV_STATIC_RES_DEV_PARAM("bus", "/i2c0"),
  DEV_STATIC_RES_I2C_ADDRESS(0x20),
  DEV_STATIC_RES_IRQ(0, 18, 0, "/gpio")
  );
*/

// Keyboard
// 8 columns on P0.29-P0.30,P0.0-P0.5
// 8 rows on P0.21-P0.28
// Merge of rows (through diodes) on WU_IRQ_2 (P0.16)

#if defined(CONFIG_DRIVER_NRF51_GPIO) && defined(CONFIG_DRIVER_MATRIX_KEYBOARD)

static const uint64_t columns_mask = endian_le64(0x6000003f);
static const uint64_t rows_mask    = endian_le64(0xff);

DEV_DECLARE_STATIC(keyboard_dev, "keyboard", 0, matrix_keyboard_drv,
  DEV_STATIC_RES_DEV_PARAM("timer", "/rtc1"),
  DEV_STATIC_RES_IRQ(0, NRF51_GPIO_RANGE_IRQ_ID, 0, "/gpio"),
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
