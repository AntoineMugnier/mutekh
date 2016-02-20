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

    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014
    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2012

*/

#if defined(CONFIG_DEVICE)
# include <device/device.h>
# include <device/driver.h>
# include <device/resources.h>
# include <device/irq.h>
# include <device/class/iomux.h>
# include <device/class/cmu.h>
#endif

#include <arch/stm32_memory_map.h>
#include <arch/stm32_irq.h>

#if defined(CONFIG_DRIVER_CPU_ARM32M)

/* CPU. */
DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv,
                   DEV_STATIC_RES_ID(0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_STM32_USART)

#include <device/class/uart.h>

/* USART2. */
DEV_DECLARE_STATIC(usart2_dev, "uart2", 0, stm32_usart_drv,
                   DEV_STATIC_RES_MEM(STM32_USART2_ADDR, STM32_USART2_ADDR + STM32_USART2_SIZE),

                   DEV_STATIC_RES_FREQ(36000000, 1),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_USART2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("tx", 0, /* PA2 */ 0*16+2, 0 /* no remap */, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, /* PA3 */ 0*16+3, 0 /* no remap */, 0),

                   /* default configuration. */
                   DEV_STATIC_RES_UART(115200, 8, DEV_UART_PARITY_NONE, 1, 0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_STM32_GPIO)

/* GPIO A..E. */
DEV_DECLARE_STATIC(gpio_dev, "gpio", 0, stm32_gpio_drv,
                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_EXTI0,     DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),
                   DEV_STATIC_RES_IRQ(1, STM32_IRQ_EXTI1,     DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),
                   DEV_STATIC_RES_IRQ(2, STM32_IRQ_EXTI2,     DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),
                   DEV_STATIC_RES_IRQ(3, STM32_IRQ_EXTI3,     DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),
                   DEV_STATIC_RES_IRQ(4, STM32_IRQ_EXTI4,     DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),
                   DEV_STATIC_RES_IRQ(5, STM32_IRQ_EXTI9_5,   DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),
                   DEV_STATIC_RES_IRQ(6, STM32_IRQ_EXTI15_10, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1)
                   );

#endif

#if defined(CONFIG_DRIVER_STM32_SPI)

DEV_DECLARE_STATIC(spi1_dev, "spi1", 0, stm32_spi_drv,
                   DEV_STATIC_RES_MEM(STM32_SPI1_ADDR, STM32_SPI1_ADDR + STM32_SPI1_SIZE),

                   DEV_STATIC_RES_FREQ(72000000, 1),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_SPI1, DEV_IRQ_SENSE_RISING_EDGE, 0, 0x1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("clk", 0, 0*16+5 /* PA5 */, 0 /* no remap */, 0),
                   DEV_STATIC_RES_IOMUX("miso", 0, 0*16+6 /* PA6 */, 0 /* no remap */, 0),
                   DEV_STATIC_RES_IOMUX("mosi", 0, 0*16+7 /* PA7 */, 0 /* no remap */, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_PUSH_BUTTON)

DEV_DECLARE_STATIC(btn0_dev, "btn0", 0, push_button_drv,
                   DEV_STATIC_RES_UINT_PARAM("release-state", 1),
                   DEV_STATIC_RES_DEV_ICU("/gpio"),
                   DEV_STATIC_RES_IRQ(0, 2*16+13 /* PC13 */, DEV_IRQ_SENSE_ANY_EDGE, 0, 0x1)
                  );

#endif

/////////////////////////////////////////////////////////////////////

#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <arch/stm32f1xx_rcc.h>
#include <mutek/startup.h>

#define __IO volatile

struct stm32f1xx_flash_dev_s
{
  __IO uint32_t FLASH_ACR;
  __IO uint32_t FLASH_KEYR;
  __IO uint32_t FLASH_OPTKEYR;
  __IO uint32_t FLASH_SR;
  __IO uint32_t FLASH_CR;
  __IO uint32_t FLASH_AR;
  __IO uint32_t __reserved0;
  __IO uint32_t FLASH_OBR;
  __IO uint32_t FLASH_WRPR;
} __attribute__ ((packed));

#define STM32_FLASH_LAT_WS(n)   ((n) & 0x7)

/* Set the frequency to the maximam valid frequency (84MHz). */
void stm32_clock_init(void)
{
  uint32_t x;

  __IO struct stm32f1xx_flash_dev_s *flash_dev =
    (struct stm32f1xx_flash_dev_s *) 0x40022000;

  /* disable interrupts. */
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_CIR_ADDR, 0);

  /* PLL OFF, PLLI2S OFF, HSE ON, HSI OFF. */
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_CR_ADDR,
    endian_le32(STM32_RCC_CR_HSEON));

  /* wait for hse stability. */
  do
    {
      x = endian_le32(cpu_mem_read_32(STM32_RCC_ADDR + STM32_RCC_CR_ADDR));
    }
  while (!(x & STM32_RCC_CR_HSERDY));

  /* set pll to 72MHz. */
  x = 0;
  STM32_RCC_CFGR_PLLSRC_SET(x, HSE);
  STM32_RCC_CFGR_PLLXTPRE_SET(x, DIV_1);
  STM32_RCC_CFGR_PLLMUL_SET(x, MUL_9);
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_CFGR_ADDR, endian_le32(x));

  /* switch the pll on. */
  x = endian_le32(cpu_mem_read_32(STM32_RCC_ADDR + STM32_RCC_CR_ADDR));
  STM32_RCC_CR_PLLON_SET(x, 1);
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_CR_ADDR, endian_le32(x));

  /* wait until pll is locked. */
  do
    {
      x = endian_le32(cpu_mem_read_32(STM32_RCC_ADDR + STM32_RCC_CR_ADDR));
    }
  while (!(x & STM32_RCC_CR_PLLRDY));

  /* set wait cycles for the flash. */
  flash_dev->FLASH_ACR = STM32_FLASH_LAT_WS(2);

  /* configure prescalers. */
  x = endian_le32(cpu_mem_read_32(STM32_RCC_ADDR + STM32_RCC_CFGR_ADDR));
  STM32_RCC_CFGR_HPRE_SET(x, DIV_1);    /* 72MHZ */
  STM32_RCC_CFGR_PPRE1_SET(x, DIV_2);   /* 36MHZ */
  STM32_RCC_CFGR_PPRE2_SET(x, DIV_1);   /* 72MHz */

  /* use the pll @ 72MHz for the system clock. */
  STM32_RCC_CFGR_SW_SET(x, PLL);
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_CFGR_ADDR, endian_le32(x));

  /* wait for system clock to be sourced by the pll. */
  do
    {
      x = endian_le32(cpu_mem_read_32(STM32_RCC_ADDR + STM32_RCC_CFGR_ADDR));
    }
  while (!(x & STM32_RCC_CFGR_SWS_PLL));

  /* enable all clock gates. */
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_AHBENR_ADDR, -1);
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_APB1ENR_ADDR, -1);
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_APB2ENR_ADDR, -1);
}
