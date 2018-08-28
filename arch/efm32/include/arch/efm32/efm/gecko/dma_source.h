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
#define EFM32_DMA_SOURCE_TIMER0   24
#define EFM32_DMA_SOURCE_TIMER1   25
#define EFM32_DMA_SOURCE_TIMER2   26
#define EFM32_DMA_SOURCE_UART0    44
#define EFM32_DMA_SOURCE_MSC      48
#define EFM32_DMA_SOURCE_AES      49

#define EFM32_DMA_SIGNAL_ADC0SINGLE         0
#define EFM32_DMA_SIGNAL_ADC0SCAN           1

#define EFM32_DMA_SIGNAL_DAC0CH0            0
#define EFM32_DMA_SIGNAL_DAC0CH1            1

#define EFM32_DMA_SIGNAL_USARTRXDATAV      0
#define EFM32_DMA_SIGNAL_USARTTXBL         1
#define EFM32_DMA_SIGNAL_USARTTXEMPTY      2

#define EFM32_DMA_SIGNAL_LEUARTRXDATAV     0
#define EFM32_DMA_SIGNAL_LEUARTTXBL        1
#define EFM32_DMA_SIGNAL_LEUARTTXEMPTY     2

#define EFM32_DMA_SIGNAL_I2C0RXDATAV        0
#define EFM32_DMA_SIGNAL_I2C0TXBL           1

#define EFM32_DMA_SIGNAL_TIMERUFOF         0
#define EFM32_DMA_SIGNAL_TIMERCC0          1
#define EFM32_DMA_SIGNAL_TIMERCC1          2
#define EFM32_DMA_SIGNAL_TIMERCC2          3

#define EFM32_DMA_SIGNAL_MSCWDATA           0

#define EFM32_DMA_SIGNAL_AESDATAWR          0
#define EFM32_DMA_SIGNAL_AESXORDATAWR       1
#define EFM32_DMA_SIGNAL_AESDATARD          2
#define EFM32_DMA_SIGNAL_AESKEYWR           3

#endif

