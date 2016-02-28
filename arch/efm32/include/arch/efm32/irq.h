
#ifndef EFM32_IRQ_H_
#define EFM32_IRQ_H_

#ifdef CONFIG_EFM32_GECKO
#define EFM32_IRQ_DMA             0
#define EFM32_IRQ_GPIO_EVEN       1
#define EFM32_IRQ_TIMER0          2
#define EFM32_IRQ_USART0_RX       3
#define EFM32_IRQ_USART0_TX       4
#define EFM32_IRQ_ACPM            5
#define EFM32_IRQ_ADC0            6
#define EFM32_IRQ_DAC0            7
#define EFM32_IRQ_I2C0            8
#define EFM32_IRQ_GPIO_ODD        9
#define EFM32_IRQ_TIMER1         10
#define EFM32_IRQ_TIMER2         11
#define EFM32_IRQ_USART1_RX      12
#define EFM32_IRQ_USART1_TX      13
#define EFM32_IRQ_USART2_RX      14
#define EFM32_IRQ_USART2_TX      15
#define EFM32_IRQ_UART0_RX       16
#define EFM32_IRQ_UART0_TX       17
#define EFM32_IRQ_LEUART0        18
#define EFM32_IRQ_LEUART1        19
#define EFM32_IRQ_LETIMER0       20
#define EFM32_IRQ_PCNT0          21
#define EFM32_IRQ_PCNT1          22
#define EFM32_IRQ_PCNT2          23
#define EFM32_IRQ_RTC            24
#define EFM32_IRQ_CMU            25
#define EFM32_IRQ_VCMP           26
#define EFM32_IRQ_LCD            27
#define EFM32_IRQ_MSC            28
#define EFM32_IRQ_AES            29
#endif

#ifdef CONFIG_EFM32_ZERO_GECKO
#define EFM32_IRQ_DMA            0
#define EFM32_IRQ_GPIO_EVEN      1
#define EFM32_IRQ_TIMER0         2
#define EFM32_IRQ_ACPM0          3
#define EFM32_IRQ_ADC0           4
#define EFM32_IRQ_I2C0           5
#define EFM32_IRQ_GPIO_ODD       6
#define EFM32_IRQ_TIMER1         7
#define EFM32_IRQ_USART1_RX      8
#define EFM32_IRQ_USART1_TX      9
#define EFM32_IRQ_LEUART0        10
#define EFM32_IRQ_PCNT0          11
#define EFM32_IRQ_RTC            12
#define EFM32_IRQ_CMU            13
#define EFM32_IRQ_VCMP           14
#define EFM32_IRQ_MSC            15
#define EFM32_IRQ_AES            16
#endif

#if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO)
#define EFM32_IRQ_DMA            0
#define EFM32_IRQ_GPIO_EVEN      1
#define EFM32_IRQ_TIMER0         2
#define EFM32_IRQ_USART0_RX      3
#define EFM32_IRQ_USART0_TX      4
#define EFM32_IRQ_USB            5
#define EFM32_IRQ_ACMP0          6
#define EFM32_IRQ_ACMP1          6
#define EFM32_IRQ_ADC0           7
#define EFM32_IRQ_DAC0           8
#define EFM32_IRQ_I2C0           9
#define EFM32_IRQ_I2C1           10
#define EFM32_IRQ_GPIO_ODD       11
#define EFM32_IRQ_TIMER1         12
#define EFM32_IRQ_TIMER2         13
#define EFM32_IRQ_TIMER3         14
#define EFM32_IRQ_USART1_RX      15
#define EFM32_IRQ_USART1_TX      16
#define EFM32_IRQ_LESENSE        17
#define EFM32_IRQ_USART2_RX      18
#define EFM32_IRQ_USART2_TX      19            
#define EFM32_IRQ_UART0_RX       20
#define EFM32_IRQ_UART0_TX       21
#define EFM32_IRQ_UART1_RX       22
#define EFM32_IRQ_UART1_TX       23
#define EFM32_IRQ_LEUART0        24
#define EFM32_IRQ_LEUART1        25
#define EFM32_IRQ_LETIMER0       26
#define EFM32_IRQ_PCNT0          27
#define EFM32_IRQ_PCNT1          28
#define EFM32_IRQ_PCNT2          29
#define EFM32_IRQ_RTC            30
#define EFM32_IRQ_BURTC          31
#define EFM32_IRQ_CMU            32
#define EFM32_IRQ_VCMP           33
#define EFM32_IRQ_LCD            34
#define EFM32_IRQ_MSC            35
#define EFM32_IRQ_AES            36
#define EFM32_IRQ_EBI            37
#define EFM32_IRQ_EMU            38
#endif

#endif
