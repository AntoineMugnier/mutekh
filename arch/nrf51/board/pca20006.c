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
#include <arch/nrf51/ids.h>

#ifdef CONFIG_DRIVER_NRF51_UART

DEV_DECLARE_STATIC(uart_dev, "uart0", 0, nrf51_uart_drv,
                   NRF_STATIC_RES_PERIPHERAL_MEM(NRF51_UART0),
                   DEV_STATIC_RES_IRQ(0, NRF51_UART0, 0, "/cpu"),
                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_UART(1000000, 8, DEV_UART_PARITY_NONE, 1, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, 1, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, 11, 0, 0)
                   );

#endif
