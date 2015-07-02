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

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/irq.h>
#include <device/class/iomux.h>
#include <device/class/clock.h>

#include <arch/stm32_memory_map.h>
#include <arch/stm32_irq.h>

/* CPU. */
DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm32m_drv,
                   DEV_STATIC_RES_ID(0, 0)
                   );

#if defined(CONFIG_DRIVER_STM32_USART)

#include <device/class/uart.h>

/* USART1. */
DEV_DECLARE_STATIC(usart1_dev, "uart1", 0, stm32_usart_drv,
                   DEV_STATIC_RES_MEM(
                                      STM32_DEV_MEM_START(USART, 1),
                                      STM32_DEV_MEM_END(USART, 1)
                                      ),

                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_USART1, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),

                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("tx", 0, /* PA9 */ 0*16+9, /* AF7. */ 7, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, /* PA10 */ 0*16+10, /* AF7. */ 7, 0),
                   );

/* USART2. */
DEV_DECLARE_STATIC(usart2_dev, "uart2", 0, stm32_usart_drv,
                   DEV_STATIC_RES_MEM(
                                      STM32_DEV_MEM_START(USART, 2),
                                      STM32_DEV_MEM_END(USART, 2)
                                      ),

                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_USART2, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),

                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("tx", 0, /* PA2 */ 0*16+2, /* AF7. */ 7, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, /* PA3 */ 0*16+3, /* AF7. */ 7, 0),

                   /* default configuration. */
                   DEV_STATIC_RES_UART(115200, 8, DEV_UART_PARITY_NONE, 1, 0, 0)
                   );

/* USART6. */
DEV_DECLARE_STATIC(usart6_dev, "uart6", 0, stm32_usart_drv,
                   DEV_STATIC_RES_MEM(
                                      STM32_DEV_MEM_START(USART, 6),
                                      STM32_DEV_MEM_END(USART, 6)
                                      ),

                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_USART6, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),

                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("tx", 0, /* PA11 */ 0*16+11, /* AF8. */ 8, 0),
                   DEV_STATIC_RES_IOMUX("rx", 0, /* PA12 */ 0*16+12, /* AF8. */ 8, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_STM32_I2C)

/* I2C1. */
DEV_DECLARE_STATIC(i2c1_dev, "i2c1", 0, stm32_i2c_ctrl_drv,
                   DEV_STATIC_RES_MEM(
                                      STM32_DEV_MEM_START(I2C, 1),
                                      STM32_DEV_MEM_END(I2C, 1)
                                      ),

                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_I2C1_EV, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),
                   DEV_STATIC_RES_IRQ(1, STM32_IRQ_I2C1_ER, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),

                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("scl", 0, /* PB8 */ 1*16+8, /* AF4 */ 4, 0),
                   DEV_STATIC_RES_IOMUX("sda", 0, /* PB9 */ 1*16+9, /* AF4 */ 4, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_STM32_GPIO)

/* GPIO A..E. */
DEV_DECLARE_STATIC(gpio_dev, "gpio", 0, stm32_gpio_drv,
                   DEV_STATIC_RES_MEM(
                                      STM32_DEV_MEM_START(GPIO, A),
                                      STM32_DEV_MEM_END(GPIO, E)
                                      ),

                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
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
                   DEV_STATIC_RES_MEM(
                                      STM32_DEV_MEM_START(TIM, 4),
                                      STM32_DEV_MEM_END(TIM, 4)
                                      ),

                   DEV_STATIC_RES_DEV_PARAM("icu", "/cpu"),
                   DEV_STATIC_RES_IRQ(0, STM32_IRQ_TIM4, DEV_IRQ_SENSE_HIGH_LEVEL, 0, 0x1),
                   );

#endif

#if defined(CONFIG_DRIVER_STM32_PWM)

/* PWM on TIMER 2. */
DEV_DECLARE_STATIC(pwm2_dev, "pwm2", 0, stm32_pwm_drv,
                   DEV_STATIC_RES_MEM(
                                      STM32_DEV_MEM_START(TIM, 2),
                                      STM32_DEV_MEM_END(TIM, 2)
                                      ),

                   DEV_STATIC_RES_FREQ(84000000, 1),

                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("oc1", 0, /* PA5 */ 0*16+5, /* AF1 */ 1, 0),
                   DEV_STATIC_RES_IOMUX("oc2", 0, /* PB3 */ 1*16+3, /* AF1 */ 1, 0)
                   );

/* PWM on TIMER 3. */
DEV_DECLARE_STATIC(pwm3_dev, "pwm3", 0, stm32_pwm_drv,
                   DEV_STATIC_RES_MEM(
                                      STM32_DEV_MEM_START(TIM, 3),
                                      STM32_DEV_MEM_END(TIM, 3)
                                      ),

                   DEV_STATIC_RES_FREQ(84000000, 1),

                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("oc1", 0, /* PA6 */ 0*16+6, /* AF2 */ 2, 0),
                   DEV_STATIC_RES_IOMUX("oc2", 0, /* PA7 */ 0*16+7, /* AF2 */ 2, 0)
                   );

/* PWM on TIMER 5. */
DEV_DECLARE_STATIC(pwm5_dev, "pwm5", 0, stm32_pwm_drv,
                   DEV_STATIC_RES_MEM(
                                      STM32_DEV_MEM_START(TIM, 5),
                                      STM32_DEV_MEM_END(TIM, 5)
                                      ),

                   DEV_STATIC_RES_FREQ(84000000, 1),

                   DEV_STATIC_RES_DEV_PARAM("iomux", "/gpio"),
                   DEV_STATIC_RES_IOMUX("oc1", 0, /* PA0 */ 0*16+0, /* AF2 */ 2, 0),
                   DEV_STATIC_RES_IOMUX("oc2", 0, /* PA1 */ 0*16+1, /* AF2 */ 2, 0)
                   );

#endif

#if defined(CONFIG_DRIVER_PUSH_BUTTON)

DEV_DECLARE_STATIC(btn0_dev, "btn0", 0, push_button_drv,
                   DEV_STATIC_RES_UINT_PARAM("release-state", 1),
                   DEV_STATIC_RES_DEV_PARAM("icu", "/gpio"),
                   DEV_STATIC_RES_IRQ(0, 2*16+13 /* PC13 */, DEV_IRQ_SENSE_ANY_EDGE, 0, 0x1)
                  );

#endif

/////////////////////////////////////////////////////////////////////

uint32_t stm32f4xx_clock_freq_ahb1 = 16000000; /* 16MHz on reset. */
uint32_t stm32f4xx_clock_freq_apb1 = 16000000; /* 16MHz on reset. */
uint32_t stm32f4xx_clock_freq_apb2 = 16000000; /* 16MHz on reset. */

#define __IO volatile

struct stm32f4xx_rcc_dev_s
{
  __IO uint32_t RCC_CR;
  __IO uint32_t RCC_PLLCFGR;
  __IO uint32_t RCC_CFGR;
  __IO uint32_t RCC_CIR;
  __IO uint32_t RCC_AHB1RSTR;
  __IO uint32_t RCC_AHB2RSTR;
       uint32_t RCC_reserved1[2];
  __IO uint32_t RCC_APB1RSTR;
  __IO uint32_t RCC_APB2RSTR;
       uint32_t RCC_reserved2[2];
  __IO uint32_t RCC_AHB1ENR;
  __IO uint32_t RCC_AHB2ENR;
       uint32_t RCC_reserved3[2];
  __IO uint32_t RCC_APB1ENR;
  __IO uint32_t RCC_APB2ENR;
       uint32_t RCC_reserved4[2];
  __IO uint32_t RCC_AHB1LPENR;
  __IO uint32_t RCC_AHB2LPENR;
       uint32_t RCC_reserved5[2];
  __IO uint32_t RCC_APB1LPENR;
  __IO uint32_t RCC_APB2LPENR;
       uint32_t RCC_reserved6[2];
  __IO uint32_t RCC_BDCR;
  __IO uint32_t RCC_CSR;
       uint32_t RCC_reserved7[2];
  __IO uint32_t RCC_SSCGR;
  __IO uint32_t RCC_PLLI2SCFGR;
       uint32_t RCC_reserved8[1];
  __IO uint32_t RCC_DCKCFGR;
} __attribute__ ((packed));

#define STM32_RCC_PLLSRC_HSI    0
#define STM32_RCC_PLLSRC_HSE    (1 << 22)

#define STM32_RCC_PLL_M_DIV(n)  ((n) << 0)
#define STM32_RCC_PLL_N_SCAL(n) ((n) << 6)
#define STM32_RCC_PLL_Q_DIV(n)  ((n) << 24)

#define STM32_RCC_PLL_P_DIV_2   0
#define STM32_RCC_PLL_P_DIV_4   (1 << 16)
#define STM32_RCC_PLL_P_DIV_6   (2 << 16)
#define STM32_RCC_PLL_P_DIV_8   (3 << 16)

#define STM32_RCC_PLL_RDY       (1 << 25)
#define STM32_RCC_PLL_SWS       0xc
#define STM32_RCC_SYSCLK_PLL    0x8

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
void stm32_clock_init()
{
  struct stm32f4xx_rcc_dev_s * rcc_dev =
    ( struct stm32f4xx_rcc_dev_s * ) 0x40023800;

  struct stm32f4xx_flash_dev_s * flash_dev =
    ( struct stm32f4xx_flash_dev_s * ) 0x40023c00;

  /* reset configuration. */
  rcc_dev->RCC_CR = 1 << 16; /* PLL OFF, PLLI2S OFF, HSE ONN, HSI OFF. */
  //rcc_dev->RCC_CR |= (16 << 3); /* set trim for HSI clock. */

  /* configure the pll parameters (HSI 16MHz -> 84MHz). */
  rcc_dev->RCC_PLLCFGR =
    STM32_RCC_PLLSRC_HSE      |
    STM32_RCC_PLL_M_DIV(8)    |
    STM32_RCC_PLL_N_SCAL(336) |
    STM32_RCC_PLL_P_DIV_4     | /* System clock @ 84MHz. */
    STM32_RCC_PLL_Q_DIV(7)      /* Peripheral clock @ 48MHz. */
  ;

  /* disable interrupts. */
  rcc_dev->RCC_CIR = 0x0;

  /* switch the pll on. */
  rcc_dev->RCC_CR |= 1 << 24;

  /* wait until pll is locked. */
  while ((rcc_dev->RCC_CR & STM32_RCC_PLL_RDY) == 0);

  /* activate caches this is important for instruction fetch. */
  flash_dev->FLASH_ACR =
    STM32_FLASH_ICACHE_EN |
    STM32_FLASH_DCACHE_EN |
    STM32_FLASH_LAT_WS(2)
  ;

  /* use the pll @ 84MHz for the system clock. */
  rcc_dev->RCC_CFGR |= 0x2;

  /* wait for system clock to be sourced by the pll. */
  while (
    (rcc_dev->RCC_CFGR & STM32_RCC_PLL_SWS) !=
    STM32_RCC_SYSCLK_PLL
  );

  /* configure AHB prescaler (not divided). */
  rcc_dev->RCC_CFGR &= ~0xf0;

  /* configure APB1 @ 42MHz and APB2 @ 84MHz prescaler. */
  rcc_dev->RCC_CFGR &= ~0xfc00;
  rcc_dev->RCC_CFGR |= (4 << 10); /* APB1 @ 42MHz */

  /* update working frequency. */
  stm32f4xx_clock_freq_ahb1 = 84000000;
  stm32f4xx_clock_freq_apb1 = 42000000;
  stm32f4xx_clock_freq_apb2 = 84000000;
}

