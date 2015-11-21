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

#ifndef _STM32_MEMORY_MAP_H_
#define _STM32_MEMORY_MAP_H_

#if CONFIG_STM32_FAMILY == 1

#define STM32_ARM_PERIPH_ADDR       0xe0000000
#define STM32_ARM_PERIPH_SIZE       0x00180000

#define STM32_CRC_ADDR              0x40023000
#define STM32_CRC_SIZE              0x00000400

#define STM32_FLASH_ADDR            0x40022000
#define STM32_FLASH_SIZE            0x00000400

#define STM32_RCC_ADDR              0x40021000
#define STM32_RCC_SIZE              0x00000400

#define STM32_DMA2_ADDR             0x40020400
#define STM32_DMA2_SIZE             0x00000400

#define STM32_DMA1_ADDR             0x40020000
#define STM32_DMA1_SIZE             0x00000400

#define STM32_SDIO_ADDR             0x40018000
#define STM32_SDIO_SIZE             0x00000400

#define STM32_TIM11_ADDR            0x40015400
#define STM32_TIM11_SIZE            0x00000400

#define STM32_TIM10_ADDR            0x40015000
#define STM32_TIM10_SIZE            0x00000400

#define STM32_TIM09_ADDR            0x40014c00
#define STM32_TIM09_SIZE            0x00000400

#define STM32_ADC3_ADDR             0x40013c00
#define STM32_ADC3_SIZE             0x00000400

#define STM32_USART1_ADDR           0x40013800
#define STM32_USART1_SIZE           0x00000400

#define STM32_TIM8_ADDR             0x40013400
#define STM32_TIM8_SIZE             0x00000400

#define STM32_SPI1_ADDR             0x40013000
#define STM32_SPI1_SIZE             0x00000400

#define STM32_TIM1_ADDR             0x40012c00
#define STM32_TIM1_SIZE             0x00000400

#define STM32_ADC2_ADDR             0x40012800
#define STM32_ADC2_SIZE             0x00000400

#define STM32_ADC1_ADDR             0x40012400
#define STM32_ADC1_SIZE             0x00000400

#define STM32_GPIO_ADDR             0x40010800
#define STM32_GPIO_SIZE             0x00001c00

#define STM32_EXTI_ADDR             0x40010400
#define STM32_EXTI_SIZE             0x00000400

#define STM32_AFIO_ADDR             0x40010000
#define STM32_AFIO_SIZE             0x00000400

#define STM32_DAC_ADDR              0x40007400
#define STM32_DAC_SIZE              0x00000400

#define STM32_PWR_ADDR              0x40007000
#define STM32_PWR_SIZE              0x00000400

#define STM32_BKP_ADDR              0x40006c00
#define STM32_BKP_SIZE              0x00000400

#define STM32_BXCAN_ADDR            0x40006400
#define STM32_BXCAN_SIZE            0x00000400

#define STM32_SRAM_ADDR             0x40006000
#define STM32_SRAM_SIZE             0x00000400

#define STM32_USB_ADDR              0x40005c00
#define STM32_USB_SIZE              0x00000400

#define STM32_I2C2_ADDR             0x40005800
#define STM32_I2C2_SIZE             0x00000400

#define STM32_I2C1_ADDR             0x40005400
#define STM32_I2C1_SIZE             0X00000400

#define STM32_UART5_ADDR            0x40005000
#define STM32_UART5_SIZE            0x00000400

#define STM32_UART4_ADDR            0x40004c00
#define STM32_UART4_SIZE            0x00000400

#define STM32_USART3_ADDR           0x40004800
#define STM32_USART3_SIZE           0x00000400

#define STM32_USART2_ADDR           0x40004400
#define STM32_USART2_SIZE           0x00000400

#define STM32_SPI3_ADDR             0x40003c00
#define STM32_SPI3_SIZE             0x00000400

#define STM32_I2S3_ADDR             0x40003c00
#define STM32_I2S3_SIZE             0x00000400

#define STM32_SPI2_ADDR             0x40003800
#define STM32_SPI2_SIZE             0x00000400

#define STM32_I2S2_ADDR             0x40003800
#define STM32_I2S2_SIZE             0x00000400

#define STM32_IWDG_ADDR             0x40003000
#define STM32_IWDG_SIZE             0x00000400

#define STM32_WWDG_ADDR             0x40002c00
#define STM32_WWDG_SIZE             0x00000400

#define STM32_RTC_ADDR              0x40002800
#define STM32_RTC_SIZE              0x00000400

#define STM32_TIM14_ADDR            0x40002000
#define STM32_TIM14_SIZE            0x00000400

#define STM32_TIM13_ADDR            0x40001c00
#define STM32_TIM13_SIZE            0x00000400

#define STM32_TIM12_ADDR            0x40001800
#define STM32_TIM12_SIZE            0x00000400

#define STM32_TIM7_ADDR             0x40001400
#define STM32_TIM7_SIZE             0x00000400

#define STM32_TIM6_ADDR             0x40001000
#define STM32_TIM6_SIZE             0x00000400

#define STM32_TIM5_ADDR             0x40000c00
#define STM32_TIM5_SIZE             0x00000400

#define STM32_TIM4_ADDR             0x40000800
#define STM32_TIM4_SIZE             0x00000400

#define STM32_TIM3_ADDR             0x40000400
#define STM32_TIM3_SIZE             0x00000400

#define STM32_TIM2_ADDR             0x40000000
#define STM32_TIM2_SIZE             0x00000400

#elif CONFIG_STM32_FAMILY == 4

#define STM32_ARM_PERIPH_ADDR       0xe0000000
#define STM32_ARM_PERIPH_SIZE       0x00180000

#define STM32_USB_OTG_FS_ADDR       0x50000000
#define STM32_USB_OTG_FS_SIZE       0x00040000

#define STM32_DMA2_ADDR             0x40026400
#define STM32_DMA2_SIZE             0x00000400

#define STM32_DMA1_ADDR             0x40026000
#define STM32_DMA1_SIZE             0x00000400

#define STM32_FLASH_ADDR            0x40023c00
#define STM32_FLASH_SIZE            0x00000400

#define STM32_RCC_ADDR              0x40023800
#define STM32_RCC_SIZE              0x00000400

#define STM32_CRC_ADDR              0x40023000
#define STM32_CRC_SIZE              0x00000400

#define STM32_GPIO_ADDR             0x40020000
#define STM32_GPIO_SIZE             0x00002000

#define STM32_TIM11_ADDR            0x40014800
#define STM32_TIM11_SIZE            0x00000400

#define STM32_TIM10_ADDR            0x40014400
#define STM32_TIM10_SIZE            0x00000400

#define STM32_TIM9_ADDR             0x40014000
#define STM32_TIM9_SIZE             0x00000400

#define STM32_EXTI_ADDR             0x40013c00
#define STM32_EXTI_SIZE             0x00000400

#define STM32_SYSCFG_ADDR           0x40013800
#define STM32_SYSCFG_SIZE           0x00000400

#define STM32_SPI4_ADDR             0x40013400
#define STM32_SPI4_SIZE             0x00000400

#define STM32_I2S4_ADDR             0x40013400
#define STM32_I2S4_SIZE             0x00000400

#define STM32_SPI1_ADDR             0x40013000
#define STM32_SPI1_SIZE             0x00000400

#define STM32_SDIO_ADDR             0x40012c00
#define STM32_SDIO_SIZE             0x00000400

#define STM32_ADC1_ADDR             0x40012000
#define STM32_ADC1_SIZE             0x00000400

#define STM32_USART6_ADDR           0x40011400
#define STM32_USART6_SIZE           0x00000400

#define STM32_USART1_ADDR           0x40011000
#define STM32_USART1_SIZE           0x00000400

#define STM32_TIM8_ADDR             0x40010400
#define STM32_TIM8_SIZE             0x00000400

#define STM32_TIM1_ADDR             0x40010000
#define STM32_TIM1_SIZE             0x00000400

#define STM32_PWR_ADDR              0x40007000
#define STM32_PWR_SIZE              0x00000400

#define STM32_I2C3_ADDR             0x40005c00
#define STM32_I2C3_SIZE             0x00000400

#define STM32_I2C2_ADDR             0x40005800
#define STM32_I2C2_SIZE             0x00000400

#define STM32_I2C1_ADDR             0x40005400
#define STM32_I2C1_SIZE             0x00000400

#define STM32_USART2_ADDR           0x40004400
#define STM32_USART2_SIZE           0x00000400

#define STM32_I2S3EXT_ADDR          0x40004000
#define STM32_I2S3EXT_SIZE          0x00000400

#define STM32_SPI3_ADDR             0x40003c00
#define STM32_SPI3_SIZE             0x00000400

#define STM32_I2S3_ADDR             0x40003c00
#define STM32_I2S3_SIZE             0x00000400

#define STM32_SPI2_ADDR             0x40003800
#define STM32_SPI2_SIZE             0x00000400

#define STM32_I2S2_ADDR             0x40003800
#define STM32_I2S2_SIZE             0x00000400

#define STM32_I2S2EXT_ADDR          0x40003400
#define STM32_I2S2EXT_SIZE          0x00000400

#define STM32_IWDG_ADDR             0x40003000
#define STM32_IWDG_SIZE             0x00000400

#define STM32_WWDG_ADDR             0x40002c00
#define STM32_WWDG_SIZE             0x00000400

#define STM32_RTC_BKP_ADDR          0x40002800
#define STM32_RTC_BKP_SIZE          0x00000400

#define STM32_TIM5_ADDR             0x40000c00
#define STM32_TIM5_SIZE             0x00000400

#define STM32_TIM4_ADDR             0x40000800
#define STM32_TIM4_SIZE             0x00000400

#define STM32_TIM3_ADDR             0x40000400
#define STM32_TIM3_SIZE             0x00000400

#define STM32_TIM2_ADDR             0x40000000
#define STM32_TIM2_SIZE             0x00000400

#endif

#endif

