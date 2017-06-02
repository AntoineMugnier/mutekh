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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2016

*/

#if defined(CONFIG_DEVICE)
# include <device/resources.h>
# include <device/irq.h>
# include <device/class/iomux.h>
# include <device/class/cmu.h>
# include <device/class/timer.h>
# include <device/class/dma.h>
# include <arch/efm32/dma_source.h>
#endif

#include <hexo/iospace.h>
#include <arch/efm32/irq.h>
#include <arch/efm32/pin.h>
#include <arch/efm32/gpio.h>
#include <arch/efm32/cmu.h>
#include <arch/efm32/devaddr.h>
#include <mutek/startup.h>

#define HFXO_FREQ  19000000

void efm32_board_init()
{
  uint32_t b, x;

  /* Enable GPIO clock */
  b = EFM32_CMU_ADDR;
  cpu_mem_write_32(b + EFM32_CMU_OSCENCMD_ADDR, EFM32_CMU_OSCENCMD_HFRCOEN);
  while (!(cpu_mem_read_32(b + EFM32_CMU_STATUS_ADDR) & EFM32_CMU_STATUS_HFRCORDY))
    ;
  cpu_mem_write_32(b + EFM32_CMU_HFCLKSEL_ADDR, EFM32_CMU_HFCLKSEL_HF(HFRCO));
  x = cpu_mem_read_32(b + EFM32_CMU_HFBUSCLKEN0_ADDR);
  cpu_mem_write_32(b + EFM32_CMU_HFBUSCLKEN0_ADDR, x | EFM32_CMU_HFBUSCLKEN0_GPIO);

  uint32_t gpio = EFM32_GPIO_ADDR;
  uint32_t button_pin = 86;

  /* wait for button to be released */
  uint32_t bank = button_pin / 16;
  uint32_t h = (button_pin >> 1) & 4;

  x = cpu_mem_read_32(b + EFM32_GPIO_MODEL_ADDR(bank) + h);
  EFM32_GPIO_MODEL_MODE_SET(button_pin % 8, x, INPUT);
  cpu_mem_write_32(b + EFM32_GPIO_MODEL_ADDR(bank) + h, x);

  while (!(cpu_mem_read_32(b + EFM32_GPIO_DIN_ADDR(bank))
           & EFM32_GPIO_DIN_DIN(button_pin % 16)))
    ;
}

#if defined(CONFIG_DRIVER_CPU_ARM32M)

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv,
                   DEV_STATIC_RES_ID(0, 0),
                   DEV_STATIC_RES_FREQ(40000000, 1),
                   );

#endif

#if defined(CONFIG_DRIVER_EFM32_USART_CHAR)

DEV_DECLARE_STATIC(uart0_dev, "usart0", 0, efm32_usart_drv,
                   DEV_STATIC_RES_MEM(0x40010000, 0x40010400),
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_USART0_RX, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(1, EFM32_IRQ_USART0_TX, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("rx", EFM32_LOC0, EFM32_PA1, 0, 0),
                   DEV_STATIC_RES_IOMUX("tx", EFM32_LOC0, EFM32_PA0, 0, 0)
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_LEUART_CHAR

DEV_DECLARE_STATIC(leuart0_dev, "leuart0", 0, efm32_leuart_drv,
                   DEV_STATIC_RES_MEM(0x4004a000, 0x4004a400),
                   DEV_STATIC_RES_FREQ(32768, 1),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_LEUART0, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("tx",  EFM32_LOC2, EFM32_PA2, 0, 0),
                   DEV_STATIC_RES_IOMUX("rx",  EFM32_LOC2, EFM32_PA3, 0, 0)
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_GPIO

DEV_DECLARE_STATIC(gpio_dev, "gpio", 0, efm32_gpio_drv,
                   DEV_STATIC_RES_MEM(0x4000a000, 0x4000b000),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_GPIO_EVEN, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   DEV_STATIC_RES_IRQ(1, EFM32_IRQ_GPIO_ODD, DEV_IRQ_SENSE_RISING_EDGE, 0, 1)
                   );

#endif

#ifdef CONFIG_DRIVER_EFM32_TIMER

DEV_DECLARE_STATIC(timer1_dev, "timer0", 0, efm32_timer_drv,
                   DEV_STATIC_RES_MEM(0x40018000, 0x40018400),
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_TIMER0, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
                   );

#endif

#if defined(CONFIG_DRIVER_EFR32_DMA)

DEV_DECLARE_STATIC(dma_dev, "dma", 0, efm32_dma_drv,
                   DEV_STATIC_RES_MEM(0x400e2000, 0x400e3000),
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_DMA, DEV_IRQ_SENSE_RISING_EDGE, 0, 1)
                   );

#endif

#if defined(CONFIG_DRIVER_EFM32_USART_SPI)

DEV_DECLARE_STATIC(usart1_dev, "spi1", 0, efm32_usart_spi_drv,

                   DEV_STATIC_RES_MEM(0x40010400, 0x40010800),
                   DEV_STATIC_RES_FREQ(HFXO_FREQ, 1),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, EFM32_IRQ_USART1_RX, DEV_IRQ_SENSE_RISING_EDGE, 0, 1),
#if defined(CONFIG_DRIVER_EFR32_DMA)
                   DEV_STATIC_RES_DEV_PARAM("dma", "/dma"),
                   /* Read channel must have higher priority than write channel */
                   DEV_STATIC_RES_DMA((1 << 0), (EFM32_DMA_SOURCE_USART1 | (EFM32_DMA_SIGNAL_USART1RXDATAV << 8))),
                   DEV_STATIC_RES_DMA((1 << 1), (EFM32_DMA_SOURCE_USART1 | (EFM32_DMA_SIGNAL_USART1TXEMPTY << 8))),
#endif

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("clk",  EFM32_LOC11, EFM32_PC8, 0, 0),
                   DEV_STATIC_RES_IOMUX("miso", EFM32_LOC11, EFM32_PC7, 0, 0),
                   DEV_STATIC_RES_IOMUX("mosi", EFM32_LOC11, EFM32_PC6, 0, 0),
#if 0
                   DEV_STATIC_RES_IOMUX("cs",   EFM32_LOC8, EFM32_PC9, 0, 0),
#endif

#ifdef CONFIG_DRIVER_EFM32_TIMER
                   DEV_STATIC_RES_DEV_TIMER("/timer0")
#endif
                   );

#endif
