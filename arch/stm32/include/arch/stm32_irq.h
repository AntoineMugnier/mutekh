#ifndef _STM32_IRQ_H_
#define _STM32_IRQ_H_

#if CONFIG_STM32_FAMILY == 1

/* XL-devices Table 62 p. 200 of CD00171190 manual. */
#if CONFIG_STM32_FLASHSIZE >= 0x000c0000 /* 768Kb */

#define STM32_IRQ_WWDG                  0
#define STM32_IRQ_PVD                   1
#define STM32_IRQ_TAMPER                2
#define STM32_IRQ_RTC                   3
#define STM32_IRQ_FLASH                 4
#define STM32_IRQ_RCC                   5
#define STM32_IRQ_EXTI0                 6
#define STM32_IRQ_EXTI1                 7
#define STM32_IRQ_EXTI2                 8
#define STM32_IRQ_EXTI3                 9
#define STM32_IRQ_EXTI4                 10
#define STM32_IRQ_DMA1_CHANNEL1         11
#define STM32_IRQ_DMA1_CHANNEL2         12
#define STM32_IRQ_DMA1_CHANNEL3         13
#define STM32_IRQ_DMA1_CHANNEL4         14
#define STM32_IRQ_DMA1_CHANNEL5         15
#define STM32_IRQ_DMA1_CHANNEL6         16
#define STM32_IRQ_DMA1_CHANNEL7         17
#define STM32_IRQ_ADC1_2                18
#define STM32_IRQ_USB_HP_CAN_TX         19
#define STM32_IRQ_USB_LP_CAN_RX0        20
#define STM32_IRQ_CAN_RX1               21
#define STM32_IRQ_CAN_SCE               22
#define STM32_IRQ_EXTI9_5               23
#define STM32_IRQ_TIM1_BRK_TIM9         24
#define STM32_IRQ_TIM1_UP_TIM10         25
#define STM32_IRQ_TIM1_TRG_COM_TIM11    26
#define STM32_IRQ_TIM1_CC               27
#define STM32_IRQ_TIM2                  28
#define STM32_IRQ_TIM3                  29
#define STM32_IRQ_TIM4                  30
#define STM32_IRQ_I2C1_EV               31
#define STM32_IRQ_I2C1_ER               32
#define STM32_IRQ_I2C2_EV               33
#define STM32_IRQ_I2C2_ER               34
#define STM32_IRQ_SPI1                  35
#define STM32_IRQ_SPI2                  36
#define STM32_IRQ_USART1                37
#define STM32_IRQ_USART2                38
#define STM32_IRQ_USART3                39
#define STM32_IRQ_EXTI15_10             40
#define STM32_IRQ_RTCALARM              41
#define STM32_IRQ_USB_WAKE_UP           42
#define STM32_IRQ_TIM8_BRK_TIM12        43
#define STM32_IRQ_TIM8_UP_TIM13         44
#define STM32_IRQ_TIM8_TRG_COM_TIM14    45
#define STM32_IRQ_TIM8_CC               46
#define STM32_IRQ_ADC3                  47
#define STM32_IRQ_FSMC                  48
#define STM32_IRQ_SDIO                  49
#define STM32_IRQ_TIM5                  50
#define STM32_IRQ_SPI3                  51
#define STM32_IRQ_UART4                 52
#define STM32_IRQ_UART5                 53
#define STM32_IRQ_TIM6                  54
#define STM32_IRQ_TIM7                  55
#define STM32_IRQ_DMA2_CHANNEL1         56
#define STM32_IRQ_DMA2_CHANNEL2         57
#define STM32_IRQ_DMA2_CHANNEL3         58
#define STM32_IRQ_DMA2_CHANNEL4_5       59

#endif
#endif

#if CONFIG_STM32_FAMILY == 4

#define STM32_IRQ_WWDG          0
#define STM32_IRQ_EXTI16        1
#define STM32_IRQ_PVD           1
#define STM32_IRQ_EXTI21        2
#define STM32_IRQ_TAMP_STAMP    2
#define STM32_IRQ_EXTI22        3
#define STM32_IRQ_RTC_WKUP      3
#define STM32_IRQ_FLASH         4
#define STM32_IRQ_RCC           5
#define STM32_IRQ_EXTI0         6
#define STM32_IRQ_EXTI1         7
#define STM32_IRQ_EXTI2         8
#define STM32_IRQ_EXTI3         9
#define STM32_IRQ_EXTI4         10
#define STM32_IRQ_DMA1_STR0     11
#define STM32_IRQ_DMA1_STR1     12
#define STM32_IRQ_DMA1_STR2     13
#define STM32_IRQ_DMA1_STR3     14
#define STM32_IRQ_DMA1_STR4     15
#define STM32_IRQ_DMA1_STR5     16
#define STM32_IRQ_DMA1_STR6     17
#define STM32_IRQ_ADC           18
#define STM32_IRQ_EXTI_9_5      23
#define STM32_IRQ_TM1_BRK       24
#define STM32_IRQ_TM9           24
#define STM32_IRQ_TIM1_UPDT     25
#define STM32_IRQ_TIM10         25
#define STM32_IRQ_TIM1_TRG_COM  26
#define STM32_IRQ_TIM11         26
#define STM32_IRQ_TIM1_CC       27
#define STM32_IRQ_TIM2          28
#define STM32_IRQ_TIM3          29
#define STM32_IRQ_TIM4          30
#define STM32_IRQ_I2C1_EV       31
#define STM32_IRQ_I2C1_ER       32
#define STM32_IRQ_I2C2_EV       33
#define STM32_IRQ_I2C2_ER       34
#define STM32_IRQ_SPI1          35
#define STM32_IRQ_SPI2          36
#define STM32_IRQ_USART1        37
#define STM32_IRQ_USART2        38
#define STM32_IRQ_EXTI_15_10    40
#define STM32_IRQ_EXTI17        41
#define STM32_IRQ_RTC_ALARM     41
#define STM32_IRQ_EXTI18        42
#define STM32_IRQ_OTG_FS_WKUP   42
#define STM32_IRQ_DMA1_STR7     47
#define STM32_IRQ_SDIO          49
#define STM32_IRQ_TIM5          50
#define STM32_IRQ_SPI3          51
#define STM32_IRQ_DMA2_STR0     56
#define STM32_IRQ_DMA2_STR1     57
#define STM32_IRQ_DMA2_STR2     58
#define STM32_IRQ_DMA2_STR3     59
#define STM32_IRQ_DMA2_STR4     60
#define STM32_IRQ_OTG_FS        67
#define STM32_IRQ_DMA2_STR5     68
#define STM32_IRQ_DMA2_STR6     69
#define STM32_IRQ_DMA2_STR7     70
#define STM32_IRQ_USART6        71
#define STM32_IRQ_I2C3_EV       72
#define STM32_IRQ_I2C3_ER       73
#define STM32_IRQ_FPU           81
#define STM32_IRQ_SPI4          84

#endif // CONFIG_STM32_FAMILY == 4

#endif
