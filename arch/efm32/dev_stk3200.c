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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include <device/resources.h>
#include <device/class/iomux.h>

#include <arch/efm32_irq.h>
#include <arch/efm32_pin.h>


DEV_DECLARE_STATIC_RESOURCES(cpu_dev_res, 1,
  DEV_STATIC_RES_ID(0, 0),
);

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm_m_drv, cpu_dev_res);


#ifdef CONFIG_DRIVER_EFM32_USART_SPI

DEV_DECLARE_STATIC_RESOURCES(usart1_dev_res, 8,
  DEV_STATIC_RES_MEM(0x4000c400, 0x4000c800),

  DEV_STATIC_RES_IRQ(0, EFM32_IRQ_USART1_RX, 0, "/cpu"),
  DEV_STATIC_RES_IRQ(1, EFM32_IRQ_USART1_TX, 0, "/cpu"),

  DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
  DEV_STATIC_RES_IOMUX("clk",  EFM32_LOC3, EFM32_PC15, 0, 0),
  DEV_STATIC_RES_IOMUX("miso", EFM32_LOC3, EFM32_PD6, 0, 0),
  DEV_STATIC_RES_IOMUX("mosi", EFM32_LOC3, EFM32_PD7, 0, 0),
#if 0
  DEV_STATIC_RES_IOMUX("cs",   EFM32_LOC3, EFM32_PC14, 0, 0),
#endif

#ifdef CONFIG_DRIVER_EFM32_TIMER
  DEV_STATIC_RES_DEV_PARAM("spi-timer", "/timer0"),
#endif
);

DEV_DECLARE_STATIC(usart1_dev, "spi1", 0, efm32_usart_spi_drv, usart1_dev_res);

#endif



#ifdef CONFIG_DRIVER_EFM32_LEUART

DEV_DECLARE_STATIC_RESOURCES(leuart0_dev_res, 5,
  DEV_STATIC_RES_MEM(0x40084000, 0x40084400),
  DEV_STATIC_RES_IRQ(0, EFM32_IRQ_LEUART0, 0, "/cpu"),

  DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
  DEV_STATIC_RES_IOMUX("tx", EFM32_LOC0, EFM32_PD4, 0, 0),
  DEV_STATIC_RES_IOMUX("rx", EFM32_LOC0, EFM32_PD5, 0, 0),
);

DEV_DECLARE_STATIC(leuart0_dev, "uart0", 0, efm32_leuart_drv, leuart0_dev_res);

#endif



#ifdef CONFIG_DRIVER_EFM32_TIMER

DEV_DECLARE_STATIC_RESOURCES(timer0_dev_res, 3,
  DEV_STATIC_RES_MEM(0x40010000, 0x40010400),
  DEV_STATIC_RES_IRQ(0, EFM32_IRQ_TIMER0, 0, "/cpu"),
);

DEV_DECLARE_STATIC(timer0_dev, "timer0", 0, efm32_timer_drv, timer0_dev_res);

#endif



#ifdef CONFIG_DRIVER_EFM32_RTC

DEV_DECLARE_STATIC_RESOURCES(rtc_dev_res, 3,
  DEV_STATIC_RES_MEM(0x40080000, 0x40080400),
  DEV_STATIC_RES_IRQ(0, EFM32_IRQ_RTC, 0, "/cpu"),
);

DEV_DECLARE_STATIC(rtc_dev, "rtc", 0, efm32_rtc_drv, rtc_dev_res);

#endif


#ifdef CONFIG_DRIVER_EFM32_GPIO

DEV_DECLARE_STATIC_RESOURCES(gpio_dev_res, 3,
  DEV_STATIC_RES_MEM(0x40006000, 0x40007000),
  DEV_STATIC_RES_IRQ(0, EFM32_IRQ_GPIO_EVEN, 0, "/cpu"),
  DEV_STATIC_RES_IRQ(1, EFM32_IRQ_GPIO_ODD, 0, "/cpu"),
);

DEV_DECLARE_STATIC(gpio_dev, "gpio", 0, efm32_gpio_drv, gpio_dev_res);

#endif

