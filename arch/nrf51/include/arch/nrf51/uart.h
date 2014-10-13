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

#ifndef ARCH_NRF51_UART_H_
#define ARCH_NRF51_UART_H_

#include "peripheral.h"
#include "ids.h"

enum nrf51_uart_task {
    NRF51_UART_STARTRX = 0,
    NRF51_UART_STOPRX = 1,
    NRF51_UART_STARTTX = 2,
    NRF51_UART_STOPTX = 3,
    NRF51_UART_UNDOC_TASK4 = 4,
    NRF51_UART_UNDOC_TASK5 = 5,
    NRF51_UART_SUSPEND = 7,
    NRF51_UART_UNDOC_TASK8 = 8,
    NRF51_UART_UNDOC_TASK9 = 9,
    NRF51_UART_UNDOC_TASK10 = 10,
};

enum nrf51_uart_event {
    NRF51_UART_CTS = 0,
    NRF51_UART_NCTS = 1,
    NRF51_UART_RXDRDY = 2,
    NRF51_UART_UNDOC_EVENT3 = 3,
    NRF51_UART_UNDOC_EVENT4 = 4,
    NRF51_UART_UNDOC_EVENT5 = 5,
    NRF51_UART_UNDOC_EVENT6 = 6,
    NRF51_UART_TXDRDY = 7,
    NRF51_UART_UNDOC_EVENT8 = 8,
    NRF51_UART_ERROR = 9,
    NRF51_UART_UNDOC_EVENT10 = 10,
    NRF51_UART_UNDOC_EVENT11 = 11,
    NRF51_UART_UNDOC_EVENT12 = 12,
    NRF51_UART_UNDOC_EVENT13 = 13,
    NRF51_UART_UNDOC_EVENT14 = 14,
    NRF51_UART_UNDOC_EVENT15 = 15,
    NRF51_UART_UNDOC_EVENT16 = 16,
    NRF51_UART_RXTO = 17,
};

enum nrf51_uart_register {
    NRF51_UART_ERRORSRC = 32,
    NRF51_UART_ENABLE = 64,
    NRF51_UART_PSELRTS = 66,
    NRF51_UART_PSELTXD = 67,
    NRF51_UART_PSELCTS = 68,
    NRF51_UART_PSELRXD = 69,
    NRF51_UART_RXD = 70,
    NRF51_UART_TXD = 71,
    NRF51_UART_BAUDRATE = 73,
    NRF51_UART_CONFIG = 91,
};

#define NRF51_UART_ERRORSRC_OVERRUN 0x1
#define NRF51_UART_ERRORSRC_PARITY  0x2
#define NRF51_UART_ERRORSRC_FRAMING 0x4
#define NRF51_UART_ERRORSRC_BREAK   0x8

#define NRF51_UART_CONFIG_CTSRTS_MASK   0x1
#define NRF51_UART_CONFIG_CTSRTS_DISABLED   0x0
#define NRF51_UART_CONFIG_CTSRTS_ENABLED   0x1

#define NRF51_UART_CONFIG_PARITY_MASK  0x6
#define NRF51_UART_CONFIG_PARITY_DISABLED  0x0
#define NRF51_UART_CONFIG_PARITY_ENABLED  0x6

#define NRF51_UART_ENABLE_MASK   0x7
#define NRF51_UART_ENABLE_DISABLED   0x0
#define NRF51_UART_ENABLE_ENABLED   0x4

#define NRF51_UART_BAUDRATE_(x)  ((x) * 268)

#endif
