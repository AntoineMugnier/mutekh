name: i2c
longname: STM32F4xx I2C controller

bound: 32
#align: 0x80

# little endian access (what ?)
swap: 0b111000

%cr1: 32
  longname: I2C controler register 1
  default: 0x0

  %%pe
    longname: Peripheral enable

  %%smbus
    longname: SMBus mode

  %%__padding__

  %%smbtype
    longname: SMBus type

  %%enarp
    longname: ARP enable

  %%enpec
    longname: PEC enable

  %%engc
    longname: General call enable

  %%nostretch
    longname: Clock stretching disable

  %%start
    longname: Start condition generation

  %%stop
    longname: Stop condition generation

  %%ack
    longname: Acknowledge enable

  %%pos
    longname: Acknowledge/PEC position

  %%pec
    longname: Packet error checking

  %%alert
    longname: SMBus alert

  %%__padding__

  %%swrst
    longname: Software reset

%cr2: 32
  longname: I2C control register 2
  default: 0x0

  %%freq: 5-0
    longname: Peripheral clock frequency

  %%__padding__: 2

  %%iterren
    longname: Error interrupt enable

  %%itevten
    longname: Event interrupt enable

  %%itbufen
    longname: Buffer interrupt enable

  %%dmaen
    longname: DMA requests enable

  %%last
    longname: DMA last transfer

%oar1: 32
  longname: I2C Own address register 1
  default: 0x0

  %%add: 9-0
    longname: Interface address

  %%__padding__: 5

  %%addmode
    longname: Addressing mode (slave mode)

%oar2: 32
  longname: I2C Own address register 2
  default: 0x0

  %%endual
    longname: Dual addressing mode enable

  %%add2: 7-1
    longname: Second interface address

%dr: 32
  longname: I2C data register
  default: 0x0

  %%data: 7-0
    longname: 8-bit data register

%sr1: 32
  longname: I2C status register 1
  default: 0

  %%sb
    longname: Start bit generated
    direction: r

  %%address
    longname: Address sent (master mode)/matched (slave mode)
    direction: r

  %%btf
    longname: Byte transfer finished
    direction: r

  %%add10
    longname: 10-bit header sent (master mode)
    direction: r

  %%stop
    longname: Stop detection (slave mode)
    direction: r

  %%__padding__

  %%rxne
    longname: Data register not empty (receivers)
    direction: r

  %%txe
    longname: Data register empty (transmitters)
    direction: r

  %%berr
    longname: Bus error

  %%arlo
    longname: Arbitration lost (master mode)

  %%af
    longname: Acknowledge failure

  %%ovr
    longname: Overrun/Underrun

  %%pecerr
    longname: PEC error in reception

  %%__padding__

  %%timeout
    longname: Timeout

  %%smbalert
    longname: SMBus alert

%sr2: 32
  longname: I2C status register 2
  default: 0x0

  %%msl
    longname: Master/slave mode
    direction: r

  %%busy
    longname: Bus busy
    direction: r

  %%tra
    longname: Transmitter/receiver mode
    direction: r

  %%__padding__

  %%gencall
    longname: General call address (slave mode)
    direction: r

  %%smbdefault
    longname: SMBus device default address (slave mode)
    direction: r

  %%smbhost
    longname: SMBus host header (slave mode)
    direction: r

  %%dualf
    longname: Dual flag (slave mode)
    direction: r

  %%pec: 15-8
    longname: Packet error checking register

%ccr: 32
  longname: I2C clock control register
  default: 0x0

  %%ccr: 11-0
    longname: Clock control register in Fm/Sm mode (master mode)

  %%__padding__: 2

  %%duty
    longname: Fm mode duty cycle

  %%fs
    longname: I2C master mode selection (Fm/Sm)

%trise: 32
  longname: I2C TRISE register
  default: 0x2

%fltr: 32
  longname: I2C FLTR register
  default: 0x0

  %%dnf: 3-0
    longname: Digital noise filter

  %%anoff
    longname: Analog noise filter off

