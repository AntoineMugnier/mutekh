#ifndef EFM32_DMA_SOURCE_H_
#define EFM32_DMA_SOURCE_H_

#define EFM32_DMA_SOURCE_NONE     0
#define EFM32_DMA_SOURCE_ADC0     8 
#define EFM32_DMA_SOURCE_DAC0     10 
#define EFM32_DMA_SOURCE_USART0   12
#define EFM32_DMA_SOURCE_USART1   13
#define EFM32_DMA_SOURCE_USART2   14
#define EFM32_DMA_SOURCE_LEUART0  16
#define EFM32_DMA_SOURCE_LEUART1  17
#define EFM32_DMA_SOURCE_I2C0     20
#define EFM32_DMA_SOURCE_I2C1     21
#define EFM32_DMA_SOURCE_TIMER0   24
#define EFM32_DMA_SOURCE_TIMER1   25
#define EFM32_DMA_SOURCE_TIMER2   26
#define EFM32_DMA_SOURCE_TIMER3   27
#define EFM32_DMA_SOURCE_UART0    44
#define EFM32_DMA_SOURCE_UART1    45
#define EFM32_DMA_SOURCE_MSC      48
#define EFM32_DMA_SOURCE_AES      49
#define EFM32_DMA_SOURCE_LESENSE  50
#define EFM32_DMA_SOURCE_EBI      51

#define EFM32_DMA_SIGNAL_ADC0SINGLE         0
#define EFM32_DMA_SIGNAL_ADC0SCAN           1

#define EFM32_DMA_SIGNAL_DAC0CH0            0
#define EFM32_DMA_SIGNAL_DAC0CH1            1

#define EFM32_DMA_SIGNAL_USART0RXDATAV      0
#define EFM32_DMA_SIGNAL_USART0TXBL         1
#define EFM32_DMA_SIGNAL_USART0TXEMPTY      2

#define EFM32_DMA_SIGNAL_USART1RXDATAV      0
#define EFM32_DMA_SIGNAL_USART1TXBL         1
#define EFM32_DMA_SIGNAL_USART1TXEMPTY      2
#define EFM32_DMA_SIGNAL_USART1RXDATAVRIGHT 3
#define EFM32_DMA_SIGNAL_USART1TXBLRIGHT    4

#define EFM32_DMA_SIGNAL_USART2RXDATAV      0
#define EFM32_DMA_SIGNAL_USART2TXBL         1
#define EFM32_DMA_SIGNAL_USART2TXEMPTY      2
#define EFM32_DMA_SIGNAL_USART2RXDATAVRIGHT 3
#define EFM32_DMA_SIGNAL_USART2TXBLRIGHT    4

#define EFM32_DMA_SIGNAL_LEUART0RXDATAV     0
#define EFM32_DMA_SIGNAL_LEUART0TXBL        1
#define EFM32_DMA_SIGNAL_LEUART0TXEMPTY     2

#define EFM32_DMA_SIGNAL_LEUART1RXDATAV     0
#define EFM32_DMA_SIGNAL_LEUART1TXBL        1
#define EFM32_DMA_SIGNAL_LEUART1TXEMPTY     2

#define EFM32_DMA_SIGNAL_I2C0RXDATAV        0
#define EFM32_DMA_SIGNAL_I2C0TXBL           1

#define EFM32_DMA_SIGNAL_I2C1RXDATAV        0
#define EFM32_DMA_SIGNAL_I2C1TXBL           1

#define EFM32_DMA_SIGNAL_TIMER0UFOF         0
#define EFM32_DMA_SIGNAL_TIMER0CC0          1
#define EFM32_DMA_SIGNAL_TIMER0CC1          2
#define EFM32_DMA_SIGNAL_TIMER0CC2          3

#define EFM32_DMA_SIGNAL_TIMER1UFOF         0
#define EFM32_DMA_SIGNAL_TIMER1CC0          1
#define EFM32_DMA_SIGNAL_TIMER1CC1          2
#define EFM32_DMA_SIGNAL_TIMER1CC2          3

#define EFM32_DMA_SIGNAL_TIMER2UFOF         0
#define EFM32_DMA_SIGNAL_TIMER2CC0          1
#define EFM32_DMA_SIGNAL_TIMER2CC1          2
#define EFM32_DMA_SIGNAL_TIMER2CC2          3

#define EFM32_DMA_SIGNAL_TIMER3UFOF         0
#define EFM32_DMA_SIGNAL_TIMER3CC0          1
#define EFM32_DMA_SIGNAL_TIMER3CC1          2
#define EFM32_DMA_SIGNAL_TIMER3CC2          3

#define EFM32_DMA_SIGNAL_UART0RXDATAV       0
#define EFM32_DMA_SIGNAL_UART0TXBL          1
#define EFM32_DMA_SIGNAL_UART0TXEMPTY       2

#define EFM32_DMA_SIGNAL_UART1RXDATAV       0
#define EFM32_DMA_SIGNAL_UART1TXBL          1
#define EFM32_DMA_SIGNAL_UART1TXEMPTY       2

#define EFM32_DMA_SIGNAL_LESENSEBUFDATAV    0

#define EFM32_DMA_SIGNAL_EBIPXL0EMPTY       0
#define EFM32_DMA_SIGNAL_EBIPXL1EMPTY       1
#define EFM32_DMA_SIGNAL_EBIPXLFULL         2
#define EFM32_DMA_SIGNAL_EBIDDEMPTY         3

#define EFM32_DMA_SIGNAL_MSCWDATA           0

#define EFM32_DMA_SIGNAL_AESDATAWR          0
#define EFM32_DMA_SIGNAL_AESXORDATAWR       1
#define EFM32_DMA_SIGNAL_AESDATARD          2
#define EFM32_DMA_SIGNAL_AESKEYWR           3


#endif
