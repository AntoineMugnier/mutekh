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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#ifndef PSOC4_PROCBLE_H
#define PSOC4_PROCBLE_H

#include <arch/psoc4/hsiom_port.h>

/*
  Peripheral addresses
 */

#define PSOC4_PERI_ADDR         0x40010000
#define PSOC4_HSIOM_PRT_ADDR(x) (PSOC4_HSIOM_PRT0_ADDR + ((x) << 8))
#define PSOC4_HSIOM_PRT0_ADDR   0x40020000
#define PSOC4_HSIOM_PRT1_ADDR   0x40020100
#define PSOC4_HSIOM_PRT2_ADDR   0x40020200
#define PSOC4_HSIOM_PRT3_ADDR   0x40020300
#define PSOC4_HSIOM_PRT4_ADDR   0x40020400
#define PSOC4_HSIOM_PRT5_ADDR   0x40020500
#define PSOC4_HSIOM_PRT6_ADDR   0x40020600
#define PSOC4_HSIOM_ADDR        0x40022100
#define PSOC4_TST_ADDR          0x40030014
#define PSOC4_GPIO_PRT_ADDR(x) (PSOC4_GPIO_PRT0_ADDR + ((x) << 8))
#define PSOC4_GPIO_PRT0_ADDR    0x40040000
#define PSOC4_GPIO_PRT1_ADDR    0x40040100
#define PSOC4_GPIO_PRT2_ADDR    0x40040200
#define PSOC4_GPIO_PRT3_ADDR    0x40040300
#define PSOC4_GPIO_PRT4_ADDR    0x40040400
#define PSOC4_GPIO_PRT5_ADDR    0x40040500
#define PSOC4_GPIO_PRT6_ADDR    0x40040600
#define PSOC4_GPIO_ADDR         0x40041000
#define PSOC4_SRSS_ADDR         0x400b0000
#define PSOC4_CORE_ADDR         0x400bf000
#define PSOC4_CPUSS_ADDR        0x40100000
#define PSOC4_SPCIF_ADDR        0x40110000
#define PSOC4_TCPWM_ADDR        0x40200000
#define PSOC4_CNT_ADDR(x)       (PSOC4_CNT0_ADDR + ((x) << 6))
#define PSOC4_CNT0_ADDR         0x40200100
#define PSOC4_CNT1_ADDR         0x40200140
#define PSOC4_CNT2_ADDR         0x40200180
#define PSOC4_CNT3_ADDR         0x402001c0
#define PSOC4_SCB_ADDR(x)       (PSOC4_SCB0_ADDR + ((x) << 16))
#define PSOC4_SCB0_ADDR         0x40240000
#define PSOC4_SCB1_ADDR         0x40250000
#define PSOC4_CSD_ADDR          0x40280000
#define PSOC4_LCD_ADDR          0x402a0000
#define PSOC4_BLERD_ADDR        0x402e0000
#define PSOC4_BLELL_ADDR        0x402e1000
#define PSOC4_BLESS_ADDR        0x402ef000
#define PSOC4_SAR_ADDR          0x403a0000
#define PSOC4_SFLASH_ADDR       0x0ffff000

#define PSOC4_GPIO_PORT_COUNT 6
/*
  IRQs
 */

#define PSOC4_IRQ_GPIO(x)  (0 + (x))
#define PSOC4_IRQ_GPIO0     0
#define PSOC4_IRQ_GPIO1     1
#define PSOC4_IRQ_GPIO2     2
#define PSOC4_IRQ_GPIO3     3
#define PSOC4_IRQ_GPIO4     4
#define PSOC4_IRQ_GPIO5     5
#define PSOC4_IRQ_GPIO_ALL  6
#define PSOC4_IRQ_LPCOMP    7
#define PSOC4_IRQ_WDT       8
#define PSOC4_IRQ_SCB(x)   (9 + (x))
#define PSOC4_IRQ_SCB0      9
#define PSOC4_IRQ_SCB1      10
#define PSOC4_IRQ_CTBm      11
#define PSOC4_IRQ_BLESS     12
#define PSOC4_IRQ_SPCIF     13
#define PSOC4_IRQ_SRSS      14
#define PSOC4_IRQ_SAR       15
#define PSOC4_IRQ_CSD       16
#define PSOC4_IRQ_TCPWM(x)  (17 + (x))
#define PSOC4_IRQ_TCPWM0    17
#define PSOC4_IRQ_TCPWM1    18
#define PSOC4_IRQ_TCPWM2    19
#define PSOC4_IRQ_TCPWM3    20
#define PSOC4_IRQ_DSI(x)    (21 + (x))

/*
  Clocks
 */

#define PSOC4_DIV_8 0
#define PSOC4_DIV_16 1
#define PSOC4_DIV_16_5 2
#define PSOC4_DIV_24 3

#define PSOC4_DIV(type_, no_) (((PSOC4_DIV_##type_) << 6) | (no_))
#define PSOC4_DIV_NONE 0xff
#define PSOC4_DIV_TYPE(div_) ((div_) >> 6)
#define PSOC4_DIV_NO(div_) ((div_) & 0x3f)

enum psoc4_clock_e {
  // Source endpoints
  PSOC4_CLOCK_SRC_PER_0,
  PSOC4_CLOCK_SRC_PER_1,
  PSOC4_CLOCK_SRC_PER_2,
  PSOC4_CLOCK_SRC_PER_3,
  PSOC4_CLOCK_SRC_PER_4,
  PSOC4_CLOCK_SRC_PER_5,
  PSOC4_CLOCK_SRC_PER_6,
  PSOC4_CLOCK_SRC_PER_7,
  PSOC4_CLOCK_SRC_PER_8,
  PSOC4_CLOCK_SRC_PER_9,
  PSOC4_CLOCK_SRC_PER_10,
  PSOC4_CLOCK_SRC_PER_11,
  PSOC4_CLOCK_SRC_PER_12,
  PSOC4_CLOCK_SRC_PER_13,
  PSOC4_CLOCK_SRC_PER_14,
  PSOC4_CLOCK_SRC_PER_15,
#define PSOC4_CLOCK_PCLK_COUNT 16
  // Also configurable as scaler
  PSOC4_CLOCK_SRC_SYSCLK,
  // Also configurable as mux between IMO, EXTCLK and ECO
  PSOC4_CLOCK_SRC_HFCLK,
  // Mask of all HFCLK-based sources
#define PSOC4_CLOCK_SRC_HF_MASK ((1 << (PSOC4_CLOCK_SRC_HFCLK + 1)) - 1)
#define PSOC4_CLOCK_SRC_NOSLEEP_MASK (PSOC4_CLOCK_SRC_HF_MASK & ~(1 << PSOC4_CLOCK_SRC_SYSCLK))
  // Also configurable as mux between ILO and WCO
  PSOC4_CLOCK_SRC_LFCLK,
#define PSOC4_CLOCK_SRC_COUNT (PSOC4_CLOCK_SRC_LFCLK + 1)
  // Oscillators
  // Configurable frequency and accuracy
  PSOC4_CLOCK_OSC_IMO,
#if defined(CONFIG_DRIVER_PSOC4_CLOCK_EXTCLK)
  PSOC4_CLOCK_OSC_EXTCLK,
#endif
  PSOC4_CLOCK_OSC_ILO,

  PSOC4_CLOCK_NODE_COUNT,
};

#define PSOC4_CLOCK_IMO_SS      PSOC4_CLOCK_SRC_PER_0
#define PSOC4_CLOCK_SCB0        PSOC4_CLOCK_SRC_PER_1
#define PSOC4_CLOCK_SCB1        PSOC4_CLOCK_SRC_PER_2
#define PSOC4_CLOCK_PUMP        PSOC4_CLOCK_SRC_PER_3
#define PSOC4_CLOCK_CSD0        PSOC4_CLOCK_SRC_PER_4
#define PSOC4_CLOCK_CSD1        PSOC4_CLOCK_SRC_PER_5
#define PSOC4_CLOCK_SAR         PSOC4_CLOCK_SRC_PER_6
#define PSOC4_CLOCK_TCPWM0      PSOC4_CLOCK_SRC_PER_7
#define PSOC4_CLOCK_TCPWM1      PSOC4_CLOCK_SRC_PER_8
#define PSOC4_CLOCK_TCPWM2      PSOC4_CLOCK_SRC_PER_9
#define PSOC4_CLOCK_TCPWM3      PSOC4_CLOCK_SRC_PER_10
#define PSOC4_CLOCK_LCD         PSOC4_CLOCK_SRC_PER_15

/*
  Pin mux
 */

#define PSOC4_IO(port_, pin_) (((port_)<<3) | (pin_))
#define PSOC4_IO_PORT(io_) ((io_) >> 3)
#define PSOC4_IO_PIN(io_) ((io_) & 7)

#define PSOC4_P0_0 PSOC4_IO(0, 0)
#define PSOC4_P0_1 PSOC4_IO(0, 1)
#define PSOC4_P0_2 PSOC4_IO(0, 2)
#define PSOC4_P0_3 PSOC4_IO(0, 3)
#define PSOC4_P0_4 PSOC4_IO(0, 4)
#define PSOC4_P0_5 PSOC4_IO(0, 5)
#define PSOC4_P0_6 PSOC4_IO(0, 6)
#define PSOC4_P0_7 PSOC4_IO(0, 7)
#define PSOC4_P1_0 PSOC4_IO(1, 0)
#define PSOC4_P1_1 PSOC4_IO(1, 1)
#define PSOC4_P1_2 PSOC4_IO(1, 2)
#define PSOC4_P1_3 PSOC4_IO(1, 3)
#define PSOC4_P1_4 PSOC4_IO(1, 4)
#define PSOC4_P1_5 PSOC4_IO(1, 5)
#define PSOC4_P1_6 PSOC4_IO(1, 6)
#define PSOC4_P1_7 PSOC4_IO(1, 7)
#define PSOC4_P2_0 PSOC4_IO(2, 0)
#define PSOC4_P2_1 PSOC4_IO(2, 1)
#define PSOC4_P2_2 PSOC4_IO(2, 2)
#define PSOC4_P2_3 PSOC4_IO(2, 3)
#define PSOC4_P2_4 PSOC4_IO(2, 4)
#define PSOC4_P2_5 PSOC4_IO(2, 5)
#define PSOC4_P2_6 PSOC4_IO(2, 6)
#define PSOC4_P2_7 PSOC4_IO(2, 7)
#define PSOC4_P3_0 PSOC4_IO(3, 0)
#define PSOC4_P3_1 PSOC4_IO(3, 1)
#define PSOC4_P3_2 PSOC4_IO(3, 2)
#define PSOC4_P3_3 PSOC4_IO(3, 3)
#define PSOC4_P3_4 PSOC4_IO(3, 4)
#define PSOC4_P3_5 PSOC4_IO(3, 5)
#define PSOC4_P3_6 PSOC4_IO(3, 6)
#define PSOC4_P3_7 PSOC4_IO(3, 7)
#define PSOC4_P4_0 PSOC4_IO(4, 0)
#define PSOC4_P4_1 PSOC4_IO(4, 1)
#define PSOC4_P5_0 PSOC4_IO(5, 0)
#define PSOC4_P5_1 PSOC4_IO(5, 1)

#define PSOC4_P0_0_TCPWM0_P      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P0_0_SCB1_UART_RX  HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P0_0_SCB1_I2C_SDA  HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P0_0_SCB1_SPI_MOSI HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P0_1_TCPWM0_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P0_1_SCB1_UART_TX  HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P0_1_SCB1_I2C_SCL  HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P0_1_SCB1_SPI_MISO HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P0_3_TCPWM1_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P0_3_SCB1_UART_CTS HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P0_3_SCB1_SPI_SCLK HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P0_4_TCPWM1_P      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P0_4_SCB0_UART_RX  HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P0_4_EXT_CLK       HSIOM_PORT_SEL_IO_SEL_ACT_2
#define PSOC4_P0_4_ECO_OUT       HSIOM_PORT_SEL_IO_SEL_ACT_2
#define PSOC4_P0_4_SCB0_I2C_SDA  HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P0_4_SCB0_SPI_MOSI HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P0_5_TCPWM1_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P0_5_SCB0_UART_TX  HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P0_5_SCB0_I2C_SCL  HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P0_5_SCB0_SPI_MISO HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P0_6_TCPWM2_P      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P0_6_SCB0_UART_RTS HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P0_6_SWDIO         HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P0_6_SCB0_SPI_SS0  HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P0_7_TCPWM2_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P0_7_SCB0_UART_CTS HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P0_7_SWDCLK        HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P0_7_SCB0_SPI_SCLK HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P1_0_TCPWM0_P      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P1_0_WCO_OUT       HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P1_1_TCPWM0_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P1_1_SCB1_SPI_SS1  HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P1_2_TCPWM1_P      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P1_2_SCB1_SPI_SS2  HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P1_3_TCPWM1_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P1_3_SCB1_SPI_SS3  HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P1_4_TCPWM2_P      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P1_4_SCB0_UART_RX  HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P1_4_SCB0_I2C_SDA  HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P1_4_SCB0_SPI_MOSI HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P1_5_TCPWM2_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P1_5_SCB0_UART_TX  HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P1_5_SCB0_I2C_SCL  HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P1_5_SCB0_SPI_MISO HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P1_6_TCPWM3_P      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P1_6_SCB0_UART_RTS HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P1_6_SCB0_SPI_SS0  HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P1_7_TCPWM3_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P1_7_SCB0_UART_CTS HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P1_7_SCB0_SPI_SCLK HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P2_0_SCB0_SPI_SS1  HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P2_1_SCB0_SPI_SS2  HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P2_2_WAKEUP        HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P2_2_SCB0_SPI_SS3  HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P2_3_WCO_OUT       HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P2_7_EXT_CLK       HSIOM_PORT_SEL_IO_SEL_ACT_2
#define PSOC4_P2_7_ECO_OUT       HSIOM_PORT_SEL_IO_SEL_ACT_2
#define PSOC4_P3_0_SARMUX        HSIOM_PORT_SEL_IO_SEL_//ANALOG
#define PSOC4_P3_0_TCPWM0_P      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P3_0_SCB0_UART_RX  HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P3_0_SCB0_I2C_SDA  HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P3_1_SARMUX        HSIOM_PORT_SEL_IO_SEL_//ANALOG
#define PSOC4_P3_1_TCPWM0_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P3_1_SCB0_UART_TX  HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P3_1_SCB0_I2C_SCL  HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P3_2_SARMUX        HSIOM_PORT_SEL_IO_SEL_//ANALOG
#define PSOC4_P3_2_TCPWM1_P      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P3_2_SCB0_UART_RTS HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P3_3_SARMUX        HSIOM_PORT_SEL_IO_SEL_//ANALOG
#define PSOC4_P3_3_TCPWM1_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P3_3_SCB0_UART_CTS HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P3_4_SARMUX_4      HSIOM_PORT_SEL_IO_SEL_//ANALOG
#define PSOC4_P3_4_TCPWM2_P      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P3_4_SCB1_UART_RX  HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P3_4_SCB1_I2C_SDA  HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P3_5_SARMUX_5      HSIOM_PORT_SEL_IO_SEL_//ANALOG
#define PSOC4_P3_5_TCPWM2_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P3_5_SCB1_UART_TX  HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P3_5_SCB1_I2C_SCL  HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P3_6_SARMUX_6      HSIOM_PORT_SEL_IO_SEL_//ANALOG
#define PSOC4_P3_6_TCPWM3_P      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P3_6_SCB1_UART_RTS HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P3_7_SARMUX_7      HSIOM_PORT_SEL_IO_SEL_//ANALOG
#define PSOC4_P3_7_TCPWM3_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P3_7_SCB1_UART_CTS HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P3_7_WCO_OUT       HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P4_0_CMOD          HSIOM_PORT_SEL_IO_SEL_//ANALOG
#define PSOC4_P4_0_TCPWM0_P      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P4_0_SCB1_UART_RTS HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P4_0_SCB1_SPI_MOSI HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P4_1_CTANK         HSIOM_PORT_SEL_IO_SEL_//ANALOG
#define PSOC4_P4_1_TCPWM0_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P4_1_SCB1_UART_CTS HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P4_1_SCB1_SPI_MISO HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P5_0_TCPWM3_P      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P5_0_SCB1_UART_RX  HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P5_0_EXTPA_EN      HSIOM_PORT_SEL_IO_SEL_ACT_2
#define PSOC4_P5_0_SCB1_I2C_SDA  HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P5_0_SCB1_SPI_SS0  HSIOM_PORT_SEL_IO_SEL_DS_1
#define PSOC4_P5_1_TCPWM3_N      HSIOM_PORT_SEL_IO_SEL_ACT_0
#define PSOC4_P5_1_SCB1_UART_TX  HSIOM_PORT_SEL_IO_SEL_ACT_1
#define PSOC4_P5_1_EXT_CLK       HSIOM_PORT_SEL_IO_SEL_ACT_2
#define PSOC4_P5_1_ECO_OUT       HSIOM_PORT_SEL_IO_SEL_ACT_2
#define PSOC4_P5_1_SCB1_I2C_SCL  HSIOM_PORT_SEL_IO_SEL_DS_0
#define PSOC4_P5_1_SCB1_SPI_SCLK HSIOM_PORT_SEL_IO_SEL_DS_1

#endif