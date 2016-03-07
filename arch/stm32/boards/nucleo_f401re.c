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

#include <arch/stm32/memory_map.h>
#include <arch/stm32/irq.h>

#if defined(CONFIG_DRIVER_CPU_ARM32M)

/* CPU. */
DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv,
                   DEV_STATIC_RES_ID(0, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_STM32_USART)

#include <device/class/uart.h>

/* USART1. */
DEV_DECLARE_STATIC(usart1_dev, "uart1", 0, stm32_usart_drv,
                   DEV_STATIC_RES_MEM(STM32_USART1_ADDR, STM32_USART1_ADDR + STM32_USART1_SIZE),

                   DEV_STATIC_RES_FREQ(84000000, 1),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_USART1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("tx", 0, /* PA9 */ 0*16+9, /* AF7. */ 7, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, /* PA10 */ 0*16+10, /* AF7. */ 7, 0),
                   );

/* USART2. */
DEV_DECLARE_STATIC(usart2_dev, "uart2", 0, stm32_usart_drv,
                   DEV_STATIC_RES_MEM(STM32_USART2_ADDR, STM32_USART2_ADDR + STM32_USART2_SIZE),

                   DEV_STATIC_RES_FREQ(42000000, 1),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_USART2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("tx", 0, /* PA2 */ 0*16+2, /* AF7. */ 7, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, /* PA3 */ 0*16+3, /* AF7. */ 7, 0),

                   /* default configuration. */
                   DEV_STATIC_RES_UART(115200, 8, DEV_UART_PARITY_NONE, 1, 0, 0)
                   );

/* USART6. */
DEV_DECLARE_STATIC(usart6_dev, "uart6", 0, stm32_usart_drv,
                   DEV_STATIC_RES_MEM(STM32_USART6_ADDR, STM32_USART6_ADDR + STM32_USART6_SIZE),

                   DEV_STATIC_RES_FREQ(84000000, 1),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_USART6, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("tx", 0, /* PA11 */ 0*16+11, /* AF8. */ 8, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, /* PA12 */ 0*16+12, /* AF8. */ 8, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_STM32_I2C)

/* I2C1. */
DEV_DECLARE_STATIC(i2c1_dev, "i2c1", 0, stm32_i2c_ctrl_drv,
                   DEV_STATIC_RES_MEM(STM32_I2C1_ADDR, STM32_I2C1_ADDR + STM32_I2C1_SIZE),

                   DEV_STATIC_RES_FREQ(42000000, 1),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_I2C1_EV, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),
                   DEV_STATIC_RES_IRQ(1, STM32_IRQ_I2C1_ER, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("scl", 0, /* PB8 */ 1*16+8, /* AF4 */ 4, 0),
                   DEV_STATIC_RES_IOMUX("sda", 0, /* PB9 */ 1*16+9, /* AF4 */ 4, 0)
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
                   DEV_STATIC_RES_IRQ(6, STM32_IRQ_EXTI15_10, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),
                   );

#endif

#if defined(CONFIG_DRIVER_STM32_TIMER)

/* TIMER 4. */
DEV_DECLARE_STATIC(timer4_dev, "timer4", 0, stm32_timer_drv,
                   DEV_STATIC_RES_MEM(STM32_TIM4_ADDR, STM32_TIM4_ADDR + STM32_TIM4_SIZE),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_TIM4, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),
                   );

#endif

#if defined(CONFIG_DRIVER_STM32_PWM)

/* PWM on TIMER 2. */
DEV_DECLARE_STATIC(pwm2_dev, "pwm2", 0, stm32_pwm_drv,
                   DEV_STATIC_RES_MEM(STM32_TIM2_ADDR, STM32_TIM2_ADDR + STM32_TIM2_SIZE),

                   DEV_STATIC_RES_FREQ(84000000, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("oc1", 0, /* PA5 */ 0*16+5, /* AF1 */ 1, 0),
                   DEV_STATIC_RES_IOMUX("oc2", 0, /* PB3 */ 1*16+3, /* AF1 */ 1, 0)
                   );

/* PWM on TIMER 3. */
DEV_DECLARE_STATIC(pwm3_dev, "pwm3", 0, stm32_pwm_drv,
                   DEV_STATIC_RES_MEM(STM32_TIM3_ADDR, STM32_TIM3_ADDR + STM32_TIM3_SIZE),

                   DEV_STATIC_RES_FREQ(84000000, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("oc1", 0, /* PA6 */ 0*16+6, /* AF2 */ 2, 0),
                   DEV_STATIC_RES_IOMUX("oc2", 0, /* PA7 */ 0*16+7, /* AF2 */ 2, 0)
                   );

/* PWM on TIMER 5. */
DEV_DECLARE_STATIC(pwm5_dev, "pwm5", 0, stm32_pwm_drv,
                   DEV_STATIC_RES_MEM(STM32_TIM5_ADDR, STM32_TIM5_ADDR + STM32_TIM5_SIZE),

                   DEV_STATIC_RES_FREQ(84000000, 1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("oc1", 0, /* PA0 */ 0*16+0, /* AF2 */ 2, 0),
                   DEV_STATIC_RES_IOMUX("oc2", 0, /* PA1 */ 0*16+1, /* AF2 */ 2, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_STM32_SPI)

DEV_DECLARE_STATIC(spi1_dev, "spi1", 0, stm32_spi_drv,
                   DEV_STATIC_RES_MEM(STM32_SPI1_ADDR, STM32_SPI1_ADDR + STM32_SPI1_SIZE),

                   DEV_STATIC_RES_FREQ(84000000, 1),

                   DEV_STATIC_RES_DEV_ICU("/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_SPI1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),

                   DEV_STATIC_RES_DEV_IOMUX("/gpio"),
                   DEV_STATIC_RES_IOMUX("clk", 0, 0*16+5 /* PA5 */, 5 /* AF5 */, 0),
                   DEV_STATIC_RES_IOMUX("miso", 0, 0*16+6 /* PA6 */, 5 /* AF5 */, 0),
                   DEV_STATIC_RES_IOMUX("mosi", 0, 0*16+7 /* PA7 */, 5 /* AF5 */, 0)
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
#include <arch/stm32/f4xx_rcc.h>
#include <mutek/startup.h>

#define __IO volatile

struct stm32f4xx_flash_dev_s
{
  __IO uint32_t FLASH_ACR;
  __IO uint32_t FLASH_KEYR;
  __IO uint32_t FLASH_OPTKEYR;
  __IO uint32_t FLASH_SR;
  __IO uint32_t FLASH_CR;
  __IO uint32_t FLASH_OPTCR;
} __attribute__ ((packed));

#define STM32_FLASH_ICACHE_EN   (1 << 9)
#define STM32_FLASH_DCACHE_EN   (1 << 10)
#define STM32_FLASH_LAT_WS(n)   ((n) & 0xf)

/* Set the frequency to the maximam valid frequency (84MHz). */
void stm32_clock_init(void)
{
  uint32_t x;

  struct stm32f4xx_flash_dev_s * flash_dev =
    ( struct stm32f4xx_flash_dev_s * ) 0x40023c00;

  /* disable irqs. */
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_CIR_ADDR, 0);

  /* enable hse clock. */
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_CR_ADDR,
    endian_le32(STM32_RCC_CR_HSEON));

  /* wait for the hse clock to stabilize. */
  do
    {
      x = endian_le32(cpu_mem_read_32(STM32_RCC_ADDR + STM32_RCC_CR_ADDR));
    }
  while (!(x & STM32_RCC_CR_HSERDY));

  /* set pll to 84MHz. */
  x = 0;
  STM32_RCC_PLLCFGR_PLLSRC_SET(x, HSE);
  STM32_RCC_PLLCFGR_M_SET(x, 8);
  STM32_RCC_PLLCFGR_N_SET(x, 336);
  STM32_RCC_PLLCFGR_P_SET(x, DIV_4);
  STM32_RCC_PLLCFGR_Q_SET(x, 7);
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_PLLCFGR_ADDR, endian_le32(x));

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

  /* activate caches and set wait cycles for the flash. */
  flash_dev->FLASH_ACR =
    STM32_FLASH_ICACHE_EN |
    STM32_FLASH_DCACHE_EN |
    STM32_FLASH_LAT_WS(2);

  /* set different bus prescalers. */
  x = 0;
  STM32_RCC_CFGR_HPRE_SET(x, DIV_1);    /* 84 MHz */
  STM32_RCC_CFGR_PPRE1_SET(x, DIV_2);   /* 42 MHz */
  STM32_RCC_CFGR_PPRE2_SET(x, DIV_1);   /* 84 MHz */

  /* use the pll @ 84MHz for the system clock. */
  STM32_RCC_CFGR_SW_SET(x, PLL);
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_CFGR_ADDR, endian_le32(x));

  /* wait for system clock to be sourced by the pll. */
  do
    {
      x = endian_le32(cpu_mem_read_32(STM32_RCC_ADDR + STM32_RCC_CFGR_ADDR));
    }
  while (!(x & STM32_RCC_CFGR_SWS_PLL));

  /* enable all clock gates. */
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_AHB1ENR_ADDR, -1);
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_APB1ENR_ADDR, -1);
  cpu_mem_write_32(STM32_RCC_ADDR + STM32_RCC_APB2ENR_ADDR, -1);
}
