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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2015
*/

#ifndef PSOC4_PIN_H
#define PSOC4_PIN_H

#include "hsiom.h"

#define PSOC4_P0_0_TCPWM0_P      ACT_0
#define PSOC4_P0_0_SCB1_UART_RX  ACT_1
#define PSOC4_P0_0_SCB1_I2C_SDA  DS_0
#define PSOC4_P0_0_SCB1_SPI_MOSI DS_1
#define PSOC4_P0_1_TCPWM0_N      ACT_0
#define PSOC4_P0_1_SCB1_UART_TX  ACT_1
#define PSOC4_P0_1_SCB1_I2C_SCL  DS_0
#define PSOC4_P0_1_SCB1_SPI_MISO DS_1
#define PSOC4_P0_3_TCPWM1_N      ACT_0
#define PSOC4_P0_3_SCB1_UART_CTS ACT_1
#define PSOC4_P0_3_SCB1_SPI_SCLK DS_1
#define PSOC4_P0_4_TCPWM1_P      ACT_0
#define PSOC4_P0_4_SCB0_UART_RX  ACT_1
#define PSOC4_P0_4_EXT_CLK       ACT_2
#define PSOC4_P0_4_ECO_OUT       ACT_2
#define PSOC4_P0_4_SCB0_I2C_SDA  DS_0
#define PSOC4_P0_4_SCB0_SPI_MOSI DS_1
#define PSOC4_P0_5_TCPWM1_N      ACT_0
#define PSOC4_P0_5_SCB0_UART_TX  ACT_1
#define PSOC4_P0_5_SCB0_I2C_SCL  DS_0
#define PSOC4_P0_5_SCB0_SPI_MISO DS_1
#define PSOC4_P0_6_TCPWM2_P      ACT_0
#define PSOC4_P0_6_SCB0_UART_RTS ACT_1
#define PSOC4_P0_6_SWDIO         DS_0
#define PSOC4_P0_6_SCB0_SPI_SS0  DS_1
#define PSOC4_P0_7_TCPWM2_N      ACT_0
#define PSOC4_P0_7_SCB0_UART_CTS ACT_1
#define PSOC4_P0_7_SWDCLK        DS_0
#define PSOC4_P0_7_SCB0_SPI_SCLK DS_1
#define PSOC4_P1_0_TCPWM0_P      ACT_0
#define PSOC4_P1_0_WCO_OUT       DS_1
#define PSOC4_P1_1_TCPWM0_N      ACT_0
#define PSOC4_P1_1_SCB1_SPI_SS1  DS_1
#define PSOC4_P1_2_TCPWM1_P      ACT_0
#define PSOC4_P1_2_SCB1_SPI_SS2  DS_1
#define PSOC4_P1_3_TCPWM1_N      ACT_0
#define PSOC4_P1_3_SCB1_SPI_SS3  DS_1
#define PSOC4_P1_4_TCPWM2_P      ACT_0
#define PSOC4_P1_4_SCB0_UART_RX  ACT_1
#define PSOC4_P1_4_SCB0_I2C_SDA  DS_0
#define PSOC4_P1_4_SCB0_SPI_MOSI DS_1
#define PSOC4_P1_5_TCPWM2_N      ACT_0
#define PSOC4_P1_5_SCB0_UART_TX  ACT_1
#define PSOC4_P1_5_SCB0_I2C_SCL  DS_0
#define PSOC4_P1_5_SCB0_SPI_MISO DS_1
#define PSOC4_P1_6_TCPWM3_P      ACT_0
#define PSOC4_P1_6_SCB0_UART_RTS ACT_1
#define PSOC4_P1_6_SCB0_SPI_SS0  DS_1
#define PSOC4_P1_7_TCPWM3_N      ACT_0
#define PSOC4_P1_7_SCB0_UART_CTS ACT_1
#define PSOC4_P1_7_SCB0_SPI_SCLK DS_1
#define PSOC4_P2_0_SCB0_SPI_SS1  DS_1
#define PSOC4_P2_1_SCB0_SPI_SS2  DS_1
#define PSOC4_P2_2_WAKEUP        DS_0
#define PSOC4_P2_2_SCB0_SPI_SS3  DS_1
#define PSOC4_P2_3_WCO_OUT       DS_1
#define PSOC4_P2_7_EXT_CLK       ACT_2
#define PSOC4_P2_7_ECO_OUT       ACT_2
#define PSOC4_P3_0_SARMUX        //ANALOG
#define PSOC4_P3_0_TCPWM0_P      ACT_0
#define PSOC4_P3_0_SCB0_UART_RX  ACT_1
#define PSOC4_P3_0_SCB0_I2C_SDA  DS_0
#define PSOC4_P3_1_SARMUX        //ANALOG
#define PSOC4_P3_1_TCPWM0_N      ACT_0
#define PSOC4_P3_1_SCB0_UART_TX  ACT_1
#define PSOC4_P3_1_SCB0_I2C_SCL  DS_0
#define PSOC4_P3_2_SARMUX        //ANALOG
#define PSOC4_P3_2_TCPWM1_P      ACT_0
#define PSOC4_P3_2_SCB0_UART_RTS ACT_1
#define PSOC4_P3_3_SARMUX        //ANALOG
#define PSOC4_P3_3_TCPWM1_N      ACT_0
#define PSOC4_P3_3_SCB0_UART_CTS ACT_1
#define PSOC4_P3_4_SARMUX_4      //ANALOG
#define PSOC4_P3_4_TCPWM2_P      ACT_0
#define PSOC4_P3_4_SCB1_UART_RX  ACT_1
#define PSOC4_P3_4_SCB1_I2C_SDA  DS_0
#define PSOC4_P3_5_SARMUX_5      //ANALOG
#define PSOC4_P3_5_TCPWM2_N      ACT_0
#define PSOC4_P3_5_SCB1_UART_TX  ACT_1
#define PSOC4_P3_5_SCB1_I2C_SCL  DS_0
#define PSOC4_P3_6_SARMUX_6      //ANALOG
#define PSOC4_P3_6_TCPWM3_P      ACT_0
#define PSOC4_P3_6_SCB1_UART_RTS ACT_1
#define PSOC4_P3_7_SARMUX_7      //ANALOG
#define PSOC4_P3_7_TCPWM3_N      ACT_0
#define PSOC4_P3_7_SCB1_UART_CTS ACT_1
#define PSOC4_P3_7_WCO_OUT       DS_1
#define PSOC4_P4_0_CMOD          //ANALOG
#define PSOC4_P4_0_TCPWM0_P      ACT_0
#define PSOC4_P4_0_SCB1_UART_RTS ACT_1
#define PSOC4_P4_0_SCB1_SPI_MOSI DS_1
#define PSOC4_P4_1_CTANK         //ANALOG
#define PSOC4_P4_1_TCPWM0_N      ACT_0
#define PSOC4_P4_1_SCB1_UART_CTS ACT_1
#define PSOC4_P4_1_SCB1_SPI_MISO DS_1
#define PSOC4_P5_0_TCPWM3_P      ACT_0
#define PSOC4_P5_0_SCB1_UART_RX  ACT_1
#define PSOC4_P5_0_EXTPA_EN      ACT_2
#define PSOC4_P5_0_SCB1_I2C_SDA  DS_0
#define PSOC4_P5_0_SCB1_SPI_SS0  DS_1
#define PSOC4_P5_1_TCPWM3_N      ACT_0
#define PSOC4_P5_1_SCB1_UART_TX  ACT_1
#define PSOC4_P5_1_EXT_CLK       ACT_2
#define PSOC4_P5_1_ECO_OUT       ACT_2
#define PSOC4_P5_1_SCB1_I2C_SCL  DS_0
#define PSOC4_P5_1_SCB1_SPI_SCLK DS_1

#define PSOC4_PMUX(port, pin, block, function)        \
  PSOC4_P##port##_##pin##_##block##_##function

#define PSOC4_PMUX_EXP(port, pin, block, function) \
  PSOC4_PMUX(port, pin, block, function)

#define PSOC4_PMUX_SET_EXP(pin, tmp, val) \
  HSIOM_PORT_SEL_IO_SEL_SET(pin, tmp, val)

#define PSOC4_PMUX_SET(tmp, port, pin, block, function)               \
  PSOC4_PMUX_SET_EXP(pin, tmp,                                        \
                       PSOC4_PMUX_EXP(port, pin, block, function))

#endif
