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

# include <device/driver.h>
# include <device/device.h>
# include <device/class/cpu.h>

void efm32_hw_enum_init()
{
  static struct device_s cpu_dev;

  device_init(&cpu_dev);
  cpu_dev.node.flags |= DEVICE_FLAG_CPU;
  device_res_add_id(&cpu_dev, 0, 0);
  device_set_name(&cpu_dev, "cpu");
  device_attach(&cpu_dev, NULL);

  extern const struct driver_s arm_drv;

  device_bind_driver(&cpu_dev, &arm_drv);
  device_init_driver(&cpu_dev);

#ifdef CONFIG_DRIVER_EFM32_LEUART
  static struct device_s leuart0_dev;

  device_init(&leuart0_dev);
  device_set_name(&leuart0_dev, "uart0");
  device_res_add_mem(&leuart0_dev, 0x40084000, 0x40084400);
#ifdef CONFIG_HEXO_IRQ
  device_res_add_irq(&leuart0_dev, 0, 24, 0, "/cpu");
#endif
  device_attach(&leuart0_dev, NULL);

  extern const struct driver_s efm32_leuart_drv;

  device_bind_driver(&leuart0_dev, &efm32_leuart_drv);
  device_init_driver(&leuart0_dev);
#endif

#ifdef CONFIG_DRIVER_EFM32_TIMER
  static struct device_s timer0_dev;

  device_init(&timer0_dev);
  device_set_name(&timer0_dev, "timer0");
  device_res_add_mem(&timer0_dev, 0x40010000, 0x40010400);
  device_res_add_frequency(&timer0_dev, (uint64_t)HFRCO_FREQUENCY << 24);
#ifdef CONFIG_HEXO_IRQ
  device_res_add_irq(&timer0_dev, 0, EFM32_IRQ_TIMER0, 0, "/cpu");
#endif
  device_attach(&timer0_dev, NULL);

  extern const struct driver_s efm32_timer_drv;

  device_bind_driver(&timer0_dev, &efm32_timer_drv);
  device_init_driver(&timer0_dev);
#endif

#ifdef CONFIG_DRIVER_EFM32_RTC
  static struct device_s rtc_dev;

  device_init(&rtc_dev);
  device_set_name(&rtc_dev, "rtc");
  device_res_add_mem(&rtc_dev, 0x40080000, 0x40080400);
  device_res_add_frequency(&rtc_dev, (uint64_t)32768 << 24);
#ifdef CONFIG_HEXO_IRQ
  device_res_add_irq(&rtc_dev, 0, EFM32_IRQ_RTC, 0, "/cpu");
#endif
  device_attach(&rtc_dev, NULL);
  extern const struct driver_s efm32_rtc_drv;
  device_bind_driver(&rtc_dev, &efm32_rtc_drv);
  device_init_driver(&rtc_dev);
#endif

}

