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

#include <mutek/startup.h>

#include <string.h>


/////////////////////////////////////////////////////////////////////

#include <mutek/mem_alloc.h>
#include <mutek/memory_allocator.h>

void stm32_mem_init()
{
  default_region = memory_allocator_init(NULL, (void*)CONFIG_STARTUP_HEAP_ADDR,
                                         (void*)(CONFIG_STARTUP_HEAP_ADDR +
                                                 CONFIG_STARTUP_HEAP_SIZE));
}

/////////////////////////////////////////////////////////////////////

# include <device/driver.h>
# include <device/resources.h>
# include <device/device.h>

#include <arch/stm32f4xx_irq.h>

/* CPU. */
DEV_DECLARE_STATIC_RESOURCES(cpu_dev_res, 1,
  DEV_STATIC_RES_ID(0, 0),
);

DEV_DECLARE_STATIC(cpu_dev, "cpu", DEVICE_FLAG_CPU, arm_m_drv, cpu_dev_res);

#if defined(CONFIG_DRIVER_STM32_USART)

/* USART1. */
DEV_DECLARE_STATIC_RESOURCES(usart1_dev_res, 2,
  DEV_STATIC_RES_MEM(0x40011000, 0x400113ff),
  DEV_STATIC_RES_IRQ(0, STM32F4xx_IRQ_USART1, 0, "/cpu"),
);

DEV_DECLARE_STATIC(
  usart1_dev,
  "uart1",
  0,
  stm32f4xx_usart_drv,
  usart1_dev_res
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

#define STM32F4xx_RCC_PLLSRC_HSI    0
#define STM32F4xx_RCC_PLLSRC_HSE    (1 << 22)

#define STM32F4xx_RCC_PLL_M_DIV(n)  ((n) << 0)
#define STM32F4xx_RCC_PLL_N_SCAL(n) ((n) << 6)
#define STM32F4xx_RCC_PLL_Q_DIV(n)  ((n) << 24)

#define STM32F4xx_RCC_PLL_P_DIV_2   0
#define STM32F4xx_RCC_PLL_P_DIV_4   (1 << 16)
#define STM32F4xx_RCC_PLL_P_DIV_6   (2 << 16)
#define STM32F4xx_RCC_PLL_P_DIV_8   (3 << 16)

#define STM32F4xx_RCC_PLL_RDY       (1 << 25)
#define STM32F4xx_RCC_PLL_SWS       0xc
#define STM32F4xx_RCC_SYSCLK_PLL    0x8

struct stm32f4xx_flash_dev_s
{
  __IO uint32_t FLASH_ACR;
  __IO uint32_t FLASH_KEYR;
  __IO uint32_t FLASH_OPTKEYR;
  __IO uint32_t FLASH_SR;
  __IO uint32_t FLASH_CR;
  __IO uint32_t FLASH_OPTCR;
} __attribute__ ((packed));

#define STM32F4xx_FLASH_ICACHE_EN   (1 << 9)
#define STM32F4xx_FLASH_DCACHE_EN   (1 << 10)
#define STM32F4xx_FLASH_LAT_WS(n)   ((n) & 0xf)

/* Set the frequency to the maximam valid frequency (84MHz). */
void stm32_freq_scaling_init()
{
  struct stm32f4xx_rcc_dev_s * rcc_dev =
    ( struct stm32f4xx_rcc_dev_s * ) 0x40023800;

  struct stm32f4xx_flash_dev_s * flash_dev =
    ( struct stm32f4xx_flash_dev_s * ) 0x40023c00;

  /* reset configuration. */
  rcc_dev->RCC_CR = 0x1; /* PLL OFF, PLLI2S OFF, HSE OFF, HSI ON. */
  //rcc_dev->RCC_CR |= (16 << 3); /* set trim for HSI clock. */

  /* configure the pll parameters (HSI 16MHz -> 84MHz). */
  rcc_dev->RCC_PLLCFGR =
    STM32F4xx_RCC_PLLSRC_HSI      |
    STM32F4xx_RCC_PLL_M_DIV(16)   |
    STM32F4xx_RCC_PLL_N_SCAL(336) |
    STM32F4xx_RCC_PLL_P_DIV_4     | /* System clock @ 84MHz. */
    STM32F4xx_RCC_PLL_Q_DIV(7)      /* Peripheral clock @ 48MHz. */
  ;

  /* disable interrupts. */
  rcc_dev->RCC_CIR = 0x0;

  /* switch the pll on. */
  rcc_dev->RCC_CR |= 1 << 24;

  /* wait until pll is locked. */
  while ((rcc_dev->RCC_CR & STM32F4xx_RCC_PLL_RDY) == 0);

  /* activate caches this is important for instruction fetch. */
  flash_dev->FLASH_ACR =
    STM32F4xx_FLASH_ICACHE_EN |
    STM32F4xx_FLASH_DCACHE_EN |
    STM32F4xx_FLASH_LAT_WS(2)
  ;

  /* use the pll @ 84MHz for the system clock. */
  rcc_dev->RCC_CFGR |= 0x2;

  /* wait for system clock to be sourced by the pll. */
  while (
    (rcc_dev->RCC_CFGR & STM32F4xx_RCC_PLL_SWS) !=
    STM32F4xx_RCC_SYSCLK_PLL
  );

  /* configure AHB prescaler (not divided). */
  rcc_dev->RCC_CFGR &= ~0xf0;

  /* configure APB1 @ 42MHz and APB2 @ 84MHz prescaler. */
  rcc_dev->RCC_CFGR &= ~0xfc00;
  rcc_dev->RCC_CFGR |= (4 << 10);

  /* update working frequency. */
  stm32f4xx_clock_freq_ahb1 = 84000000;
  stm32f4xx_clock_freq_apb1 = 42000000;
  stm32f4xx_clock_freq_apb2 = 84000000;
}

