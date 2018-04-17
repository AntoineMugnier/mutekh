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

#include <device/resources.h>
#include <device/irq.h>
#include <device/class/iomux.h>
#include <device/clock.h>
#include <device/class/uart.h>
#include <arch/psoc4/variant.h>

#ifdef CONFIG_DRIVER_PSOC4_UART

DEV_DECLARE_STATIC(uart_dev, "uart0", 0, psoc4_uart_drv,
                   DEV_STATIC_RES_MEM(PSOC4_SCB0_ADDR, PSOC4_SCB0_ADDR + 0x1000),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, PSOC4_IRQ_SCB0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_UART(1000000, 8, DEV_UART_PARITY_NONE, 1, 1),
                   DEV_STATIC_RES_IOMUX("tx", 0, PSOC4_P1_5, PSOC4_P1_5_SCB0_UART_TX, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, PSOC4_P1_4, PSOC4_P1_4_SCB0_UART_RX, 0),
# ifdef CONFIG_DEVICE_CLOCK
                   DEV_STATIC_RES_CLK_SRC("/clock", PSOC4_CLOCK_SCB0, 0),
# else
                   DEV_STATIC_RES_FREQ(32768, 1),
# endif
                   );

#endif
