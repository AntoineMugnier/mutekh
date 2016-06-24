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
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright (c) 2013 Sebastien Cerdan <sebcerdan@gmail.com>

*/

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/class/iomux.h>
#include <device/class/dma.h>
#include <device/irq.h>
#include <arch/pic32/pin.h>
#include <arch/pic32/gpio.h>
#include <arch/pic32/irq.h>
#include <arch/pic32/freq.h>

/* CPU. */
DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, mips_drv,
                   DEV_STATIC_RES_ID(0, 0),
                   DEV_STATIC_RES_FREQ(200000000, 1)
                   );


DEV_DECLARE_STATIC(icu_dev, "icu", 0, pic32_icu_drv,
                   DEV_STATIC_RES_MEM(0xBF810000, 0xBF811000),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, 0, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
//                 DEV_STATIC_RES_IOMUX("ei0", 0, PIC32_PD0, 0, 0),
                   DEV_STATIC_RES_IOMUX("ei1", 0, PIC32_PG9, PIC32_INPUT_MUX_INT1, 0),
                   DEV_STATIC_RES_IOMUX("ei2", 0, PIC32_PB8, PIC32_INPUT_MUX_INT2, 0),
                   DEV_STATIC_RES_IOMUX("ei3", 0, PIC32_PB9, PIC32_INPUT_MUX_INT3, 0),
                   DEV_STATIC_RES_IOMUX("ei4", 0, PIC32_PG7, PIC32_INPUT_MUX_INT4, 0),
                   );

#ifdef CONFIG_DRIVER_PIC32_UART_CHAR 

DEV_DECLARE_STATIC(uart2_dev, "uart2", 0, pic32_uart_drv,
                   DEV_STATIC_RES_MEM(0xBF822200, 0xBF822400),
                   DEV_STATIC_RES_FREQ(PIC32_PB2CLK_FREQ, 1),
                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, PIC32_IRQ_RX_UART2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(1, PIC32_IRQ_TX_UART2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("rx", 0, PIC32_PB15, PIC32_INPUT_MUX_URX2, 0),
                   DEV_STATIC_RES_IOMUX("tx", 0, PIC32_PB14, PIC32_OUTPUT_MUX_U2TX, 0),
                   );

#endif

#ifdef CONFIG_DRIVER_PIC32_TIMER 

DEV_DECLARE_STATIC(timer2_dev, "timer2-3", 0, pic32_timer_drv,
                   DEV_STATIC_RES_FREQ(PIC32_PB3CLK_FREQ, 1),
                   /* Timer 2 and 3 */
                   DEV_STATIC_RES_MEM(0xBF840200, 0xBF840300),
                   /* Output compare 4 */
                   DEV_STATIC_RES_MEM(0xBF844600, 0xBF844800),
                   /* Output compare 5 */
                   DEV_STATIC_RES_MEM(0xBF844800, 0xBF844a00),

                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, PIC32_IRQ_TIMER3, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(1, PIC32_IRQ_OUTPUT_COMPARE_4, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(2, PIC32_IRQ_OUTPUT_COMPARE_5, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   );

#endif

#if defined(CONFIG_DRIVER_PIC32_DMA)

DEV_DECLARE_STATIC(dma_dev, "dma", 0, pic32_dma_drv,
                   DEV_STATIC_RES_MEM(0xBF811000, 0xBF812000),
                   DEV_STATIC_RES_FREQ(PIC32_SYSCLK_FREQ, 1),

                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, PIC32_IRQ_DMA0, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(1, PIC32_IRQ_DMA1, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(2, PIC32_IRQ_DMA2, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(3, PIC32_IRQ_DMA3, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(4, PIC32_IRQ_DMA4, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(5, PIC32_IRQ_DMA5, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(6, PIC32_IRQ_DMA6, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(7, PIC32_IRQ_DMA7, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   );

#endif

#if defined(CONFIG_DRIVER_PIC32_SPI)

DEV_DECLARE_STATIC(spi_dev, "spi1", 0, pic32_spi_drv,

                   DEV_STATIC_RES_MEM(0xBF821000, 0xBF821200),
                   DEV_STATIC_RES_FREQ(PIC32_PB2CLK_FREQ, 1),

#if defined(CONFIG_DRIVER_PIC32_DMA)
                   DEV_STATIC_RES_DMA("/dma", 0, PIC32_IRQ_RX_SPI1),
                   DEV_STATIC_RES_DMA("/dma", 1, PIC32_IRQ_TX_SPI1),
#endif
                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, PIC32_IRQ_TX_SPI1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("clk",  0, PIC32_PD1, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", 0, PIC32_PD14, PIC32_INPUT_MUX_SDI1, 0),
                   DEV_STATIC_RES_IOMUX("mosi", 0, PIC32_PD10, PIC32_OUTPUT_MUX_SDO1, 0),
                   DEV_STATIC_RES_DEV_TIMER("/timer2-3")
                   );
#endif

#ifdef CONFIG_DRIVER_PIC32_GPIO
DEV_DECLARE_STATIC(gpio_dev, "gpio", 0, pic32_gpio_drv,
#ifdef CONFIG_DRIVER_PIC32_GPIO_ICU
                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, PIC32_IRQ_PORTA, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(1, PIC32_IRQ_PORTB, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(2, PIC32_IRQ_PORTC, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(3, PIC32_IRQ_PORTD, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(4, PIC32_IRQ_PORTE, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(5, PIC32_IRQ_PORTF, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(6, PIC32_IRQ_PORTG, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(7, PIC32_IRQ_PORTH, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(8, PIC32_IRQ_PORTJ, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(9, PIC32_IRQ_PORTK, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1)
#endif
                   );

#if defined(CONFIG_DRIVER_PIC32_USBDEV)
DEV_DECLARE_STATIC(usb_dev, "usb", 0, pic32_usbdev_drv,
                   DEV_STATIC_RES_MEM(0xBF8E0000, 0xBF8F0000),
                   DEV_STATIC_RES_DEV_ICU("/icu"),
                   DEV_STATIC_RES_IRQ(0, PIC32_IRQ_USB, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
                   DEV_STATIC_RES_IRQ(1, PIC32_IRQ_USB_DMA, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 1),
);
#endif
#endif
