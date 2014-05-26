name: gpio
longname: STM32F4xx GPIO controller

%moder: 32
  longname: GPIO port mode register

  %%mode: 2
   longname: Port mode

    count: 16
    default: 0b00

    0b00: Input
    0b01: Ouput
    0b10: Alt
    0b11: Analog

%otyper: 32
  longname: GPIO port output type register

  %%ot: 1
    longname: Output type

    count: 16
    default: 0

    0b0: Pushpull
    0b1: Open

%ospeeder: 32
  longname: GPIO port output speed register

  %%ospeed: 2
    longname: Port speed

    count: 16

    0b00: Low
    0b01: Medium
    0b10: Fast
    0b11: High

%pupdr: 32
  longname: GPIO port pull-up/pull-down register

  %%pupd: 2
    longname: Pull-up/pull-down

    count: 16

    0b00: None
    0b01: Pullup
    0b10: Pulldown

%idr: 32
  longname: GPIO port input data register

  %%id
    longname: Input value

    count: 16
    direction: r

%odr: 32
  longname: GPIO port output data register

  %%od
    longname: Output value

    count: 16

%bsrr: 32
  longname: GPIO port bit set/reset register

  %%bs: 1
    longname: Bit set

    count: 16
    direction: w

    0b0: None
    0b1: ONE

  %%br: 1
    longname: Bit clear

    count: 16
    direction: w

    0b0: None
    0b1: ZERO

%lckr: 32
  longname: GPIO port configuration lock register

  %%lck
    longname: Lock bit

    count: 16

  %%lckk
    longname: Lock key

%afrl: 32
  longname: GPIO port alternation function low register

  %%af: 4
    longname: Alternate function

    count: 8

%afrh: 32
  longname: GPIO port alternation function high register

  %%af: 4
    longname: Alternate function

    count: 8

