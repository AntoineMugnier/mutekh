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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013

*/

#include <mutek/startup.h>

#include <string.h>

#include <mutek/mem_alloc.h>
#include <mutek/memory_allocator.h>

void efm32_mem_init()
{
    default_region = memory_allocator_init(NULL, (void*)CONFIG_STARTUP_HEAP_ADDR,
                                           (void*)(CONFIG_STARTUP_HEAP_ADDR +
                                                   CONFIG_STARTUP_HEAP_SIZE));
}

/////////////////////////////////////////////////////////////////////

#define HFRCO_FREQUENCY         14000000

# include <device/driver.h>
# include <device/device.h>
# include <device/resources.h>
# include <device/class/cpu.h>
# include <arch/efm32_irq.h>

DEV_DECLARE_STATIC_RESOURCES(cpu_dev_res, 1,
  DEV_STATIC_RES_ID(0, 0),
);

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm_m_drv, cpu_dev_res);


#ifdef CONFIG_DRIVER_EFM32_USART_SPI

DEV_DECLARE_STATIC_RESOURCES(usart1_dev_res, 3,
  DEV_STATIC_RES_MEM(0x4000c400, 0x4000c800),
  DEV_STATIC_RES_IRQ(0, EFM32_IRQ_USART1_RX, 0, "/cpu"),
  DEV_STATIC_RES_IRQ(1, EFM32_IRQ_USART1_TX, 0, "/cpu"),
);

DEV_DECLARE_STATIC(usart1_dev, "usart1", 0, efm32_usart_spi_drv, usart1_dev_res);

#endif



#ifdef CONFIG_DRIVER_EFM32_LEUART

DEV_DECLARE_STATIC_RESOURCES(leuart0_dev_res, 2,
  DEV_STATIC_RES_MEM(0x40084000, 0x40084400),
  DEV_STATIC_RES_IRQ(0, EFM32_IRQ_LEUART0, 0, "/cpu"),
);

DEV_DECLARE_STATIC(leuart0_dev, "leuart0", 0, efm32_leuart_drv, leuart0_dev_res);

#endif



#ifdef CONFIG_DRIVER_EFM32_TIMER

DEV_DECLARE_STATIC_RESOURCES(timer0_dev_res, 3,
  DEV_STATIC_RES_MEM(0x40010000, 0x40010400),
  DEV_STATIC_RES_FREQ((uint64_t)HFRCO_FREQUENCY << 24),
  DEV_STATIC_RES_IRQ(0, EFM32_IRQ_TIMER0, 0, "/cpu"),
);

DEV_DECLARE_STATIC(timer0_dev, "timer0", 0, efm32_timer_drv, timer0_dev_res);

#endif



#ifdef CONFIG_DRIVER_EFM32_RTC

DEV_DECLARE_STATIC_RESOURCES(rtc_dev_res, 3,
  DEV_STATIC_RES_MEM(0x40080000, 0x40080400),
  DEV_STATIC_RES_FREQ((uint64_t)32768 << 24),
  DEV_STATIC_RES_IRQ(0, EFM32_IRQ_RTC, 0, "/cpu"),
);

DEV_DECLARE_STATIC(rtc_dev, "rtc", 0, efm32_rtc_drv, rtc_dev_res);

#endif

