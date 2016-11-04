#ifndef EFM32_CLOCK_H_
#define EFM32_CLOCK_H_

#define EFM32_CLOCK_ENUM_NEXT(N)  /** @hidden */ N,  N##_ = N - 1
#define EFM32_CLOCK_ENUM_PREV(N)  /** @hidden */ N##_,  N = N##_ - 1

enum efm32_clock_node_e {
  /* oscillators */
  EFM32_CLOCK_HFXO,             /* internal */
  EFM32_CLOCK_HFRCO,            /* internal */

  /* internal clock signals */
  EFM32_CLOCK_HFCLK,            /* internal */
  EFM32_CLOCK_HFCLKDIV,         /* internal */
  EFM32_CLOCK_HFCORECLK,        /* internal */
  EFM32_CLOCK_HFPERCLK,         /* internal */
  EFM32_CLOCK_LE,               /* internal */
  EFM32_CLOCK_LFACLK,           /* internal */
  EFM32_CLOCK_LFBCLK,           /* internal */

  EFM32_CLOCK_LFXO,             /* internal or source ep */
  EFM32_CLOCK_LFRCO,            /* internal or source ep */
  EFM32_CLOCK_ULFRCO,           /* internal or source ep */

  EFM32_CLOCK_OUT0,             /* source ep */
  EFM32_CLOCK_OUT1,             /* source ep */

#ifdef CONFIG_DRIVER_USB_SYNOPSYS_EFM32
  EFM32_CLOCK_USBC,             /* source ep */
# define EFM32_CLOCK_USBC EFM32_CLOCK_USBC
#endif

  /* source endpoints under HFCORECLK */
  EFM32_CLOCK_ENUM_NEXT(EFM32_CLOCK_HFCORECLK_first),

  EFM32_CLOCK_CPU,

  EFM32_CLOCK_AES,
# define EFM32_CLOCK_AES EFM32_CLOCK_AES
#ifdef CONFIG_DRIVER_USB_SYNOPSYS_EFM32
  EFM32_CLOCK_USB,
# define EFM32_CLOCK_USB EFM32_CLOCK_USB
#endif

#ifdef CONFIG_DRIVER_EFM32_DMA
  EFM32_CLOCK_DMA,
# define EFM32_CLOCK_DMA EFM32_CLOCK_DMA
#endif

#if 0
  EFM32_CLOCK_EBI,
# define EFM32_CLOCK_EBI EFM32_CLOCK_EBI
#endif

  EFM32_CLOCK_ENUM_PREV(EFM32_CLOCK_HFCORECLK_last),

  /* source endpoints under HFPERCLK */
  EFM32_CLOCK_ENUM_NEXT(EFM32_CLOCK_HFPERCLK_first),

#if defined(CONFIG_DRIVER_EFM32_TIMER) || defined(CONFIG_DRIVER_EFM32_PWM)
  EFM32_CLOCK_TIMER0,
# define EFM32_CLOCK_TIMER0 EFM32_CLOCK_TIMER0
  EFM32_CLOCK_TIMER1,
# define EFM32_CLOCK_TIMER1 EFM32_CLOCK_TIMER1
  EFM32_CLOCK_TIMER2,
# define EFM32_CLOCK_TIMER2 EFM32_CLOCK_TIMER2
#endif

#ifdef CONFIG_DRIVER_EFM32_USART
  EFM32_CLOCK_USART1,
# define EFM32_CLOCK_USART1 EFM32_CLOCK_USART1
  EFM32_CLOCK_USART0,
# define EFM32_CLOCK_USART0 EFM32_CLOCK_USART0
  EFM32_CLOCK_USART2,
# define EFM32_CLOCK_USART2 EFM32_CLOCK_USART2
  EFM32_CLOCK_UART0,
# define EFM32_CLOCK_UART0 EFM32_CLOCK_UART0
#endif

#ifdef CONFIG_DRIVER_EFM32_I2C
  EFM32_CLOCK_I2C0,
# define EFM32_CLOCK_I2C0 EFM32_CLOCK_I2C0
#endif

#ifdef CONFIG_DRIVER_EFM32_GPIO
  EFM32_CLOCK_GPIO,
# define EFM32_CLOCK_GPIO EFM32_CLOCK_GPIO
#endif

#ifdef CONFIG_DRIVER_EFM32_ADC
  EFM32_CLOCK_ADC0,
# define EFM32_CLOCK_ADC0 EFM32_CLOCK_ADC0
#endif

  EFM32_CLOCK_PRS,
# define EFM32_CLOCK_PRS EFM32_CLOCK_PRS

#if 0
  EFM32_CLOCK_VCMP,
# define EFM32_CLOCK_VCMP EFM32_CLOCK_VCMP
  EFM32_CLOCK_DAC0,
# define EFM32_CLOCK_DAC0 EFM32_CLOCK_DAC0
  EFM32_CLOCK_ACMP0,
# define EFM32_CLOCK_ACMP0 EFM32_CLOCK_ACMP0
  EFM32_CLOCK_ACMP1,
# define EFM32_CLOCK_ACMP1 EFM32_CLOCK_ACMP1
  EFM32_CLOCK_PCNT,
# define EFM32_CLOCK_PCNT EFM32_CLOCK_PCNT
#endif

  EFM32_CLOCK_ENUM_PREV(EFM32_CLOCK_HFPERCLK_last),

  /* source endpoints under LFACLK */

  EFM32_CLOCK_ENUM_NEXT(EFM32_CLOCK_LFACLK_first),

#ifdef CONFIG_DRIVER_EFM32_RTC
  EFM32_CLOCK_RTC,
# define EFM32_CLOCK_RTC EFM32_CLOCK_RTC
#endif

#if 0
  EFM32_CLOCK_LESENSE,
# define EFM32_CLOCK_LESENSE EFM32_CLOCK_LESENSE
  EFM32_CLOCK_LETIMER,
# define EFM32_CLOCK_LETIMER EFM32_CLOCK_LETIMER
  EFM32_CLOCK_LCD,
# define EFM32_CLOCK_LCD EFM32_CLOCK_LCD
#endif

  EFM32_CLOCK_ENUM_PREV(EFM32_CLOCK_LFACLK_last),

  /* source endpoints under LFBCLK */

  EFM32_CLOCK_ENUM_NEXT(EFM32_CLOCK_LFBCLK_first),

#if defined(CONFIG_DRIVER_EFM32_LEUART)
  EFM32_CLOCK_LEUART0,
# define EFM32_CLOCK_LEUART0 EFM32_CLOCK_LEUART0
  EFM32_CLOCK_LEUART1,
# define EFM32_CLOCK_LEUART1 EFM32_CLOCK_LEUART1
#endif

  EFM32_CLOCK_ENUM_PREV(EFM32_CLOCK_LFBCLK_last),

  EFM32_CLOCK_count,
};

#define EFM32_CLOCK_OSC_COUNT 6
#define EFM32_CLOCK_INTL_COUNT 6

#if defined(CONFIG_DRIVER_EFM32_BURTC) || defined(CONFIG_DRIVER_EFM32_WDOG)
# define EFM32_CLOCK_FIRST_EP EFM32_CLOCK_LFXO
#else
# define EFM32_CLOCK_FIRST_EP EFM32_CLOCK_OUT0
#endif

#define EFM32_CLOCK_EP_COUNT  (EFM32_CLOCK_count - EFM32_CLOCK_FIRST_EP)

#endif

