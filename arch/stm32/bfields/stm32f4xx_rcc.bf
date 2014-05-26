name: rcc
longname: STM32F4xx Reset and clock control (RCC)

bound: 32

%cr: 32
  longname: RCC control register

  %%hsi: 1
    longname: Internal clock (16MHz) enable

    0b0: Off
    0b1: On

  %%hsidry
    longname: Internal clock ready flag
    direction: r

  %%__padding__

  %%hsitrim: 5
    longname: Internal clock trim

  %%hsical: 8
    longname: Internal clock calibration

  %%hse: 1
    longname: External clock enable

    0b0: Off
    0b1: On

  %%hserdy
    longname: External clock ready
    direction: r

  %%hsebyp
    longname: External clock bypass

  %%css: 1
    longname: Clock security system enable

    0b0: Off
    0b1: On

  %%__padding__: 4

  %%pll: 1
    longname: PLL enable

    0b0: Off
    0b1: On

  %%pllrdy
    longname: PLL ready flag
    direction: r

  %%plli2s: 1
    longname: PLL I2C enable

    0b0: Off
    0b1: On

%pllcfgr: 32
  longname: PLL configuration register

  %%m: 6
    longname: PLL/PLLI2S input clock divider

  %%n: 9
    longname: PLL clock multiplier

  %%p: 2
    longname: PLL AHBx clock divider

    0b00: 2
    0b01: 4
    0b10: 6
    0b11: 8

  %%__padding__: 4

  %%pllsrc: 1
    longname: PLL clock source

    0b0: HSE
    0b1: HSI

  %%__padding__

  %%q: 4
    longname: PLL peripheral clock divider

%cfgr: 32
  longname: RCC configuration register

  %%sw: 2
    longname: System clock source switch

    0b00: HSI
    0b01: HSE
    0b10: PLL

  %%sws: 2
    longname: System clock source status
    direction: r

    0b00: HSI
    0b01: HSE
    0b10: PLL

  %%hpre: 4
    longname: AHB bus clock prescaler

    0b0000: DIV_1
    0b1000: DIV_2
    0b1001: DIV_4
    0b1010: DIV_8
    0b1011: DIV_16
    0b1100: DIV_64
    0b1101: DIV_128
    0b1110: DIV_256
    0b1111: DIV_512

  %%ppre1: 3
    longname: APB1 low-speed bus clock prescaler

    0b000: DIV_1
    0b100: DIV_2
    0b101: DIV_4
    0b110: DIV_8
    0b111: DIV_16

  %%ppre2: 3
    longname: APB1 high-speed bus clock prescaler

    0b000: DIV_1
    0b100: DIV_2
    0b101: DIV_4
    0b110: DIV_8
    0b111: DIV_16

  %%rtcpre: 5
    longname: HSE prescaler for RTC clock

    0b00000: No clock 
    0b00010: HSE_DIV_2
    0b00011: HSE_DIV_3
    0b00100: HSE_DIV_4

  %%mco1: 2
    longname: Microcontroller output clock 1

    0b00: HSI
    0b01: LSE
    0b10: HSE
    0b11: PLL

  %%i2ssrc: 1
    longname: I2S clock selection

    0b0: PLLI2S
    0b1: Ext clock

  %%mco1pre: 3
    longname: MCO1 prescaler

    0b000: DIV_1
    0b100: DIV_2
    0b101: DIV_3
    0b110: DIV_4
    0b111: DIV_5

  %%mco2pre: 3
    longname: MCO2 prescaler

    0b000: DIV_1
    0b100: DIV_2
    0b101: DIV_3
    0b110: DIV_4
    0b111: DIV_5

  %%mco2: 2
    longname: Microcontroller output clock 2

    0b00: Sys clock
    0b01: PLLI2S
    0b10: HSE
    0b11: PLL

%cir: 32
  longname: RCC clock interrupt register

  %%lsirdy
    longname: LSI ready flag
    direction: r

  %%lserdy
    longname: LSE ready flag
    direction: r

  %%hsirdy
    longname: HSI ready flag
    direction: r

  %%hserdy
    longname: HSE ready flag
    direction: r

  %%pllrdy
    longname: PLL ready flag
    direction: r

  %%plli2srdy
    longname: PLLI2S ready flag

  %%__padding__

  %%cssrdy
    longname: CSS ready flag
    direction: r

  %%lsirdyie
    longname: LSI ready interrupt enable

  %%lserdyie
    longname: LSE ready interrupt enable

  %%hsirdyie
    longname: HSI ready interrupt enable

  %%hserdyie
    longname: HSE ready interrupt enable

  %%pllrdyie
    longname: PLL ready interrupt enable

  %%plli2srdyie
    longname: PLLI2S ready interrupt enable

  %%__padding__: 2

  %%lsirdyc
    longname: LSI ready interrupt clear
    direction: w

  %%lserdyc
    longname: LSE ready interrupt clear
    direction: w

  %%hsirdyc
    longname: HSI ready interrupt clear
    direction: w

  %%hserdyc
    longname: HSE ready interrupt clear
    direction: w

  %%pllrdyc
    longname: PLL ready interrupt clear
    direction: w

  %%plli2srdyc
    longname: PLLI2S ready interrupt clear
    direction: w

  %%__padding__

  %%cssc
    longname: CSS ready interrupt clear
    direction: w

%ahb1rstr: 32
  longname: RCC AHB1 peripheral reset register

  %%gpioarst
    longname: IO port A reset

  %%gpiobrst
    longname: IO port B reset

  %%gpiocrst
    longname: IO port C reset

  %%gpiodrst
    longname: IO port D reset

  %%gpioerst
    longname: IO port E reset

  %%__padding__: 2

  %%gpiohrst
    longname: IO port H reset

  %%__padding__: 4

  %%crcrst
    longname: CRC reset

  %%__padding__: 5

  %%dma1rst
    longname: DMA 1 reset

  %%dma2rst
    longname: DMA 2 reset

%ahb2rstr: 32
  longname: RCC AHB2 peripheral reset register

  %%__padding__: 7

  %%otgfsrst
    longname: OTG FS reset

%ahb1enr: 32
  longname: RCC AHB1 peripheral enable register
  address: 0x30

  %%gpioaen
    longname: IO port A enable

  %%gpioben
    longname: IO port B enable

  %%gpiocen
    longname: IO port C enable

  %%gpioden
    longname: IO port D enable

  %%gpioeen
    longname: IO port E enable

  %%__padding__: 2

  %%gpiohen
    longname: IO port H enable

  %%__padding__: 4

  %%crcen
    longname: CRC enable

  %%__padding__: 8

  %%dma1en
    longname: DMA 1 enable

  %%dma2en
    longname: DMA 2 enable

%ahb2enr: 32
  longname: RCC AHB2 peripheral enable register

  %%otgfsen
    longname: OTG FS enable

%apb1enr: 32
  longname: RCC APB1 peripheral enable register
  address: 0x40

  %%tim2en
    longname: Timer 2 enable

  %%tim3en
    longname: Timer 3 enable

  %%tim4en
    longname: Timer 4 enable

  %%tim5en
    longname: Timer 5 enable

  %%__padding__: 7

  %%wwdgen
    longname: Window watchdog enable

  %%__padding__: 2

  %%spi2en
    longname: SPI 2 enable

  %%spi3en
    longname: SPI 3 enable

  %%__padding__

  %%usart2en
    longname: USART 2 enable

  %%__padding__: 3

  %%i2c1en
    longname: I2C 1 enable

  %%i2c2en
    longname: I2C 2 enable

  %%i2c3en
    longname: I2C 3 enable

  %%__padding__: 4

  %%pwren
    longname: Power interface enable

%apb2enr: 32
  longname: RCC APB2 peripheral enable register

  %%tim1en
    longname: Timer 1 enable

  %%__padding__: 3

  %%usart1en
    longname: USART 1 enable

  %%usart6en
    longname: USART 6 enable

  %%__padding__: 2

  %%adc1en
    longname: ADC 1 enable

  %%__padding__: 2

  %%sdioen
    longname: SDIO enable

  %%spi1en
    longname: SPI 1 enable

  %%spi4en
    longname: SPI 4 enable 

  %%syscfgen
    longname: SYSCFG enable

  %%__padding__

  %%tim9en
    longname: Timer 9 enable

  %%tim10en
    longname: Timer 10 enable

  %%tim11en
    longname: Timer 11 enable

