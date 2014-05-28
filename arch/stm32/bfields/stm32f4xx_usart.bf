name: usart
longname: STM32F4xx USART controller

bound: 32

%sr: 32
  longname: USART status register

  %%pe
    longname: Parity error
    direction: r

  %%fe
    longname: Framing error
    direction: r

  %%nf
    longname: Noise detected flag
    direction: r

  %%ore
    longname: Overrun error
    direction: r

  %%idle
    longname: Idle line detected
    direction: r

  %%rxne
    longname: Rend register not empty

  %%tc
    longname: Transmission complete
    default: 1

  %%txe
    longname: Transmit data register empty
    default: 1
    direction: r

  %%lbd
    longname: LIN break detection flag

  %%cts
    longname: CTS flag

%dr: 32
  longname: USART data register

  %%data: 9
    longname: Data

%brr: 32
  longname: USART baudrate register

  %%frac: 4
    longname: Divider fraction

  %%mant: 12
    longname: Divider mantissa

%cr1: 32
  longname: USART control register 1

  %%sbk
    longname: Send break

  %%rwu
    longname: Receiver wakeup

  %%re
    longname: Receiver enable

  %%te
    longname: Transmitter enable

  %%idleie
    longname: IDLE interrupt enable

  %%rxneie
    longname: RxNE interrupt enable

  %%tcie
    longname: TC interrupt enable

  %%txeie
    longname: TxE interrupt enable

  %%peie
    longname: PE interrupt enable

  %%ps: 1
    longname: Parity selection

    0b0: Even
    0b1: Odd

  %%pce: 1
    longname: Parity control enable

    0b0: None
    0b1: Enable

  %%wake: 1
    longname: Wakeup method

    0b0: Idle
    0b1: Addr

  %%m: 1
    longname: Word length

    0b0: 8 bits
    0b1: 9 bits

  %%ue
    longname: USART enable

  %%__padding__

  %%over8: 1
    longname: Oversampling mode

    0b0: 16
    0b1: 8

%cr2: 32
  longname: USART control register 2

  %%add: 4
    longname: Address of the USART node

  %%__padding__

  %%lbdl
    longname: LIN break detection length

  %%lbdie
    longname: LIN break detection interrupt enable

  %%__padding__

  %%lbcl
    longname: Last bit clock pulse

  %%cpha
    longname: Clock phase

  %%cpol
    longname: Clock polarity

  %%clken
    longname: Clock enable

  %%stop: 2
    longname: STOP bits

    0b00: 1 bit
    0b01: 0.5 bit
    0b10: 2 bits
    0b11: 1.5 bit

  %%linen
    longname: LIN mode enable

%cr3: 32
  longname: USART control register 3

  %%eie
    longname: Error interrupt enable

  %%iren
    longname: IrDA mode enable

  %%irlp
    longname: IrDA low-power

  %%hdsel
    longname: Half-duplex selection

  %%nack
    longname: Smartcard NACK enable

  %%scen
    longname: Smartcard mode enable

  %%dmar
    longname: DMA enable receiver

  %%dmat
    longname: DMA enable transmitter

  %%rtse
    longname: RTS enable

  %%ctse
    longname: CTS enable

  %%onebit
    longname: One sample bit method enable

%gtpr: 32
  longname: USART guard and prescaler register

  %%psc: 8
    longname: Prescaler value

  %%gt: 8
    longname: Guard time value

