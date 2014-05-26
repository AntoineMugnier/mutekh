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

*/

#ifndef _STM32F401RE_MEMORY_MAP_H
#define _STM32F401RE_MEMORY_MAP_H

#define STM32F4xx_ARM_PERIPH_ADDR       0xe0000000
#define STM32F4xx_ARM_PERIPH_SIZE       0x00180000

#define STM32F4xx_USB_OTG_FS_ADDR       0x50000000
#define STM32F4xx_USB_OTG_FS_SIZE       0x00040000

#define STM32F4xx_DMA2_ADDR             0x40026400
#define STM32F4xx_DMA2_SIZE             0x00000400

#define STM32F4xx_DMA1_ADDR             0x40026000
#define STM32F4xx_DMA1_SIZE             0x00000400

#define STM32F4xx_FLASH_ADDR            0x40023c00
#define STM32F4xx_FLASH_SIZE            0x00000400

#define STM32F4xx_RCC_ADDR              0x40023800
#define STM32F4xx_RCC_SIZE              0x00000400

#define STM32F4xx_CRC_ADDR              0x40023000
#define STM32F4xx_CRC_SIZE              0x00000400

#define STM32F4xx_GPIOH_ADDR            0x40021c00
#define STM32F4xx_GPIOH_SIZE            0x00000400

#define STM32F4xx_GPIOE_ADDR            0x40021000
#define STM32F4xx_GPIOE_SIZE            0x00000400

#define STM32F4xx_GPIOD_ADDR            0x4001c000
#define STM32F4xx_GPIOD_SIZE            0x00000400

#define STM32F4xx_GPIOC_ADDR            0x40028000
#define STM32F4xx_GPIOC_SIZE            0x00000400

#define STM32F4xx_GPIOB_ADDR            0x40024000
#define STM32F4xx_GPIOB_SIZE            0x00000400

#define STM32F4xx_GPIOA_ADDR            0x40020000
#define STM32F4xx_GPIOA_SIZE            0x00000400

#define STM32F4xx_TIM11_ADDR            0x40014800
#define STM32F4xx_TIM11_SIZE            0x00000400

#define STM32F4xx_TIM10_ADDR            0x40014400
#define STM32F4xx_TIM10_SIZE            0x00000400

#define STM32F4xx_TIM9_ADDR             0x40014000
#define STM32F4xx_TIM9_SIZE             0x00000400

#define STM32F4xx_EXTI_ADDR             0x40013c00
#define STM32F4xx_EXTI_SIZE             0x00000400

#define STM32F4xx_SYSCFG_ADDR           0x40013800
#define STM32F4xx_SYSCFG_SIZE           0x00000400

#define STM32F4xx_SPI4_ADDR             0x40013400
#define STM32F4xx_SPI4_SIZE             0x00000400

#define STM32F4xx_I2S4_ADDR             0x40013400
#define STM32F4xx_I2S4_SIZE             0x00000400

#define STM32F4xx_SPI1_ADDR             0x40013000
#define STM32F4xx_SPI1_SIZE             0x00000400

#define STM32F4xx_SDIO_ADDR             0x40012c00
#define STM32F4xx_SDIO_SIZE             0x00000400

#define STM32F4xx_ADC1_ADDR             0x40012000
#define STM32F4xx_ADC1_SIZE             0x00000400

#define STM32F4xx_USART6_ADDR           0x40011400
#define STM32F4xx_USART6_SIZE           0x00000400

#define STM32F4xx_USART1_ADDR           0x40011000
#define STM32F4xx_USART1_SIZE           0x00000400

#define STM32F4xx_TIM8_ADDR             0x40010400
#define STM32F4xx_TIM8_SIZE             0x00000400

#define STM32F4xx_TIM1_ADDR             0x40010000
#define STM32F4xx_TIM1_SIZE             0x00000400

#define STM32F4xx_PWR_ADDR              0x40007000
#define STM32F4xx_PWR_SIZE              0x00000400

#define STM32F4xx_I2C3_ADDR             0x40005c00
#define STM32F4xx_I2C3_SIZE             0x00000400

#define STM32F4xx_I2C2_ADDR             0x40005800
#define STM32F4xx_I2C2_SIZE             0x00000400

#define STM32F4xx_I2C1_ADDR             0x40005400
#define STM32F4xx_I2C1_SIZE             0x00000400

#define STM32F4xx_USART2_ADDR           0x40004400
#define STM32F4xx_USART2_SIZE           0x00000400

#define STM32F4xx_I2S3EXT_ADDR          0x40004000
#define STM32F4xx_I2S3EXT_SIZE          0x00000400

#define STM32F4xx_SPI3_ADDR             0x40003c00
#define STM32F4xx_SPI3_SIZE             0x00000400

#define STM32F4xx_I2S3_ADDR             0x40003c00
#define STM32F4xx_I2S3_SIZE             0x00000400

#define STM32F4xx_SPI2_ADDR             0x40003800
#define STM32F4xx_SPI2_SIZE             0x00000400

#define STM32F4xx_I2S2_ADDR             0x40003800
#define STM32F4xx_I2S2_SIZE             0x00000400

#define STM32F4xx_I2S2EXT_ADDR          0x40003400
#define STM32F4xx_I2S2EXT_SIZE          0x00000400

#define STM32F4xx_IWDG_ADDR             0x40003000
#define STM32F4xx_IWDG_SIZE             0x00000400

#define STM32F4xx_WWDG_ADDR             0x40002c00
#define STM32F4xx_WWDG_SIZE             0x00000400

#define STM32F4xx_RTC_BKP_ADDR          0x40002800
#define STM32F4xx_RTC_BKP_SIZE          0x00000400

#define STM32F4xx_TIM5_ADDR             0x40000c00
#define STM32F4xx_TIM5_SIZE             0x00000400

#define STM32F4xx_TIM4_ADDR             0x40000800
#define STM32F4xx_TIM4_SIZE             0x00000400

#define STM32F4xx_TIM3_ADDR             0x40000400
#define STM32F4xx_TIM3_SIZE             0x00000400

#define STM32F4xx_TIM2_ADDR             0x40000000
#define STM32F4xx_TIM2_SIZE             0x00000400

#endif

