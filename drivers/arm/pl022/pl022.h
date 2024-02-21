#ifndef PL022_REGS_H_
#define PL022_REGS_H_

// Peripheral BFgen decl for peripheral
// Register width: 32 bits
// BFgen-originated description version 1 This definition was converted from
// a System View Description file.

// Register 'cr0': Control register 0, SSPCR0 on page 3-4
// Control register 0, SSPCR0 on page 3-4
// Offset: 0x0
// Access mode: rw
#define PL022__CR0__OFFSET 0x0
// Serial clock rate. The value SCR is used to generate the transmit and
// receive bit rate of the PrimeCell SSP. The bit rate is: F SSPCLK CPSDVSR
// x (1+SCR) where CPSDVSR is an even value from 2-254, programmed through
// the SSPCPSR register and SCR is a value from 0-255
#define PL022__CR0__SCR__MASK 0xff00
#define PL022__CR0__SCR__POS 8
#define PL022__CR0__SCR__WIDTH 8

// SSPCLKOUT phase, applicable to Motorola SPI frame format only. See
// Motorola SPI frame format on page 2-10
#define PL022__CR0__SPH__MASK 0x80
#define PL022__CR0__SPH__POS 7
#define PL022__CR0__SPH__WIDTH 1

// SSPCLKOUT polarity, applicable to Motorola SPI frame format only. See
// Motorola SPI frame format on page 2-10
#define PL022__CR0__SPO__MASK 0x40
#define PL022__CR0__SPO__POS 6
#define PL022__CR0__SPO__WIDTH 1

// Frame format
#define PL022__CR0__FRF__MASK 0x30
#define PL022__CR0__FRF__POS 4
#define PL022__CR0__FRF__WIDTH 2
#define PL022__CR0__FRF__ENUM__MOTOROLA 0
#define PL022__CR0__FRF__ENUM__TI 1
#define PL022__CR0__FRF__ENUM__MICROWIRE 2
#define PL022__CR0__FRF__ENUM__RESERVED 3

// Data Size Select, number of bits - 1, 4 <= bits <= 16
#define PL022__CR0__DSS__MASK 0xf
#define PL022__CR0__DSS__POS 0
#define PL022__CR0__DSS__WIDTH 4


// Register 'cr1': Control register 1, SSPCR1 on page 3-5
// Control register 1, SSPCR1 on page 3-5
// Offset: 0x4
// Access mode: rw
#define PL022__CR1__OFFSET 0x4
// Slave-mode output disable. This bit is relevant only in the slave mode,
// MS=1. In multiple-slave systems, it is possible for an PrimeCell SSP
// master to broadcast a message to all slaves in the system while ensuring
// that only one slave drives data onto its serial output line. In such
// systems the RXD lines from multiple slaves could be tied together. To
// operate in such systems, the SOD bit can be set if the PrimeCell SSP
// slave is not supposed to drive the SSPTXD line: 0 SSP can drive the
// SSPTXD output in slave mode. 1 SSP must not drive the SSPTXD output in
// slave mode
#define PL022__CR1__SOD__MASK 0x8
#define PL022__CR1__SOD__POS 3
#define PL022__CR1__SOD__WIDTH 1

// Master or slave mode select. This bit can be modified only when the
// PrimeCell SSP is disabled, SSE=0
#define PL022__CR1__MS__MASK 0x4
#define PL022__CR1__MS__POS 2
#define PL022__CR1__MS__WIDTH 1
#define PL022__CR1__MS__ENUM__MASTER 0
#define PL022__CR1__MS__ENUM__SLAVE 1

// Synchronous serial port enable: 0 SSP operation disabled. 1 SSP operation
// enabled
#define PL022__CR1__SSE__MASK 0x2
#define PL022__CR1__SSE__POS 1
#define PL022__CR1__SSE__WIDTH 1

// Loop back mode
#define PL022__CR1__LBM__MASK 0x1
#define PL022__CR1__LBM__POS 0
#define PL022__CR1__LBM__WIDTH 1
#define PL022__CR1__LBM__ENUM__NORMAL 0
#define PL022__CR1__LBM__ENUM__LOOPBACK 1


// Register 'dr': Data register, SSPDR on page 3-6
// Data register, SSPDR on page 3-6
// Offset: 0x8
// Access mode: rw
#define PL022__DR__OFFSET 0x8
// Transmit/Receive FIFO: Read Receive FIFO. Write Transmit FIFO. You must
// right-justify data when the PrimeCell SSP is programmed for a data size
// that is less than 16 bits. Unused bits at the top are ignored by transmit
// logic. The receive logic automatically right-justifies
#define PL022__DR__DATA__MASK 0xffff
#define PL022__DR__DATA__POS 0
#define PL022__DR__DATA__WIDTH 16


// Register 'sr': Status register, SSPSR on page 3-7
// Status register, SSPSR on page 3-7
// Offset: 0xc
// Access mode: rw
#define PL022__SR__OFFSET 0xc
// PrimeCell SSP busy flag, RO: 0 SSP is idle. 1 SSP is currently
// transmitting and/or receiving a frame or the transmit FIFO is not empty
#define PL022__SR__BSY__MASK 0x10
#define PL022__SR__BSY__POS 4
#define PL022__SR__BSY__WIDTH 1

// Receive FIFO full, RO: 0 Receive FIFO is not full. 1 Receive FIFO is full
#define PL022__SR__RFF__MASK 0x8
#define PL022__SR__RFF__POS 3
#define PL022__SR__RFF__WIDTH 1

// Receive FIFO not empty, RO: 0 Receive FIFO is empty. 1 Receive FIFO is
// not empty
#define PL022__SR__RNE__MASK 0x4
#define PL022__SR__RNE__POS 2
#define PL022__SR__RNE__WIDTH 1

// Transmit FIFO not full, RO: 0 Transmit FIFO is full. 1 Transmit FIFO is
// not full
#define PL022__SR__TNF__MASK 0x2
#define PL022__SR__TNF__POS 1
#define PL022__SR__TNF__WIDTH 1

// Transmit FIFO empty, RO: 0 Transmit FIFO is not empty. 1 Transmit FIFO is
// empty
#define PL022__SR__TFE__MASK 0x1
#define PL022__SR__TFE__POS 0
#define PL022__SR__TFE__WIDTH 1


// Register 'cpsr': Clock prescale register, SSPCPSR on page 3-8
// Clock prescale register, SSPCPSR on page 3-8
// Offset: 0x10
// Access mode: rw
#define PL022__CPSR__OFFSET 0x10
// Clock prescale divisor. Must be an even number from 2-254, depending on
// the frequency of SSPCLK. The least significant bit always returns zero on
// reads
#define PL022__CPSR__CPSDVSR__MASK 0xff
#define PL022__CPSR__CPSDVSR__POS 0
#define PL022__CPSR__CPSDVSR__WIDTH 8


// Register 'imsc': Interrupt mask set or clear register, SSPIMSC on page 3-9
// Interrupt mask set or clear register, SSPIMSC on page 3-9
// Offset: 0x14
// Access mode: rw
#define PL022__IMSC__OFFSET 0x14
// Transmit FIFO interrupt mask: 0 Transmit FIFO half empty or less
// condition interrupt is masked. 1 Transmit FIFO half empty or less
// condition interrupt is not masked
#define PL022__IMSC__TXIM__MASK 0x8
#define PL022__IMSC__TXIM__POS 3
#define PL022__IMSC__TXIM__WIDTH 1

// Receive FIFO interrupt mask: 0 Receive FIFO half full or less condition
// interrupt is masked. 1 Receive FIFO half full or less condition interrupt
// is not masked
#define PL022__IMSC__RXIM__MASK 0x4
#define PL022__IMSC__RXIM__POS 2
#define PL022__IMSC__RXIM__WIDTH 1

// Receive timeout interrupt mask: 0 Receive FIFO not empty and no read
// prior to timeout period interrupt is masked. 1 Receive FIFO not empty and
// no read prior to timeout period interrupt is not masked
#define PL022__IMSC__RTIM__MASK 0x2
#define PL022__IMSC__RTIM__POS 1
#define PL022__IMSC__RTIM__WIDTH 1

// Receive overrun interrupt mask: 0 Receive FIFO written to while full
// condition interrupt is masked. 1 Receive FIFO written to while full
// condition interrupt is not masked
#define PL022__IMSC__RORIM__MASK 0x1
#define PL022__IMSC__RORIM__POS 0
#define PL022__IMSC__RORIM__WIDTH 1


// Register 'ris': Raw interrupt status register, SSPRIS on page 3-10
// Raw interrupt status register, SSPRIS on page 3-10
// Offset: 0x18
// Access mode: rw
#define PL022__RIS__OFFSET 0x18
// Gives the raw interrupt state, prior to masking, of the SSPTXINTR
// interrupt
#define PL022__RIS__TXRIS__MASK 0x8
#define PL022__RIS__TXRIS__POS 3
#define PL022__RIS__TXRIS__WIDTH 1

// Gives the raw interrupt state, prior to masking, of the SSPRXINTR
// interrupt
#define PL022__RIS__RXRIS__MASK 0x4
#define PL022__RIS__RXRIS__POS 2
#define PL022__RIS__RXRIS__WIDTH 1

// Gives the raw interrupt state, prior to masking, of the SSPRTINTR
// interrupt
#define PL022__RIS__RTRIS__MASK 0x2
#define PL022__RIS__RTRIS__POS 1
#define PL022__RIS__RTRIS__WIDTH 1

// Gives the raw interrupt state, prior to masking, of the SSPRORINTR
// interrupt
#define PL022__RIS__RORRIS__MASK 0x1
#define PL022__RIS__RORRIS__POS 0
#define PL022__RIS__RORRIS__WIDTH 1


// Register 'mis': Masked interrupt status register, SSPMIS on page 3-11
// Masked interrupt status register, SSPMIS on page 3-11
// Offset: 0x1c
// Access mode: rw
#define PL022__MIS__OFFSET 0x1c
// Gives the transmit FIFO masked interrupt state, after masking, of the
// SSPTXINTR interrupt
#define PL022__MIS__TXMIS__MASK 0x8
#define PL022__MIS__TXMIS__POS 3
#define PL022__MIS__TXMIS__WIDTH 1

// Gives the receive FIFO masked interrupt state, after masking, of the
// SSPRXINTR interrupt
#define PL022__MIS__RXMIS__MASK 0x4
#define PL022__MIS__RXMIS__POS 2
#define PL022__MIS__RXMIS__WIDTH 1

// Gives the receive timeout masked interrupt state, after masking, of the
// SSPRTINTR interrupt
#define PL022__MIS__RTMIS__MASK 0x2
#define PL022__MIS__RTMIS__POS 1
#define PL022__MIS__RTMIS__WIDTH 1

// Gives the receive over run masked interrupt status, after masking, of the
// SSPRORINTR interrupt
#define PL022__MIS__RORMIS__MASK 0x1
#define PL022__MIS__RORMIS__POS 0
#define PL022__MIS__RORMIS__WIDTH 1


// Register 'icr': Interrupt clear register, SSPICR on page 3-11
// Interrupt clear register, SSPICR on page 3-11
// Offset: 0x20
// Access mode: rw
#define PL022__ICR__OFFSET 0x20
// Clears the SSPRTINTR interrupt This field is cleared on writing 1.
#define PL022__ICR__RTIC__MASK 0x2
#define PL022__ICR__RTIC__POS 1
#define PL022__ICR__RTIC__WIDTH 1

// Clears the SSPRORINTR interrupt This field is cleared on writing 1.
#define PL022__ICR__RORIC__MASK 0x1
#define PL022__ICR__RORIC__POS 0
#define PL022__ICR__RORIC__WIDTH 1


// Register 'dmacr': DMA control register, SSPDMACR on page 3-12
// DMA control register, SSPDMACR on page 3-12
// Offset: 0x24
// Access mode: rw
#define PL022__DMACR__OFFSET 0x24
// Transmit DMA Enable. If this bit is set to 1, DMA for the transmit FIFO
// is enabled
#define PL022__DMACR__TXDMAE__MASK 0x2
#define PL022__DMACR__TXDMAE__POS 1
#define PL022__DMACR__TXDMAE__WIDTH 1

// Receive DMA Enable. If this bit is set to 1, DMA for the receive FIFO is
// enabled
#define PL022__DMACR__RXDMAE__MASK 0x1
#define PL022__DMACR__RXDMAE__POS 0
#define PL022__DMACR__RXDMAE__WIDTH 1


// Register 'periphid0': Peripheral identification registers, SSPPeriphID0-3 on page 3-13
// Peripheral identification registers, SSPPeriphID0-3 on page 3-13
// Offset: 0xfe0
// Access mode: rw
#define PL022__PERIPHID0__OFFSET 0xfe0
// These bits read back as 0x22
#define PL022__PERIPHID0__PARTNUMBER0__MASK 0xff
#define PL022__PERIPHID0__PARTNUMBER0__POS 0
#define PL022__PERIPHID0__PARTNUMBER0__WIDTH 8


// Register 'periphid1': Peripheral identification registers, SSPPeriphID0-3 on page 3-13
// Peripheral identification registers, SSPPeriphID0-3 on page 3-13
// Offset: 0xfe4
// Access mode: rw
#define PL022__PERIPHID1__OFFSET 0xfe4
// These bits read back as 0x1
#define PL022__PERIPHID1__DESIGNER0__MASK 0xf0
#define PL022__PERIPHID1__DESIGNER0__POS 4
#define PL022__PERIPHID1__DESIGNER0__WIDTH 4

// These bits read back as 0x0
#define PL022__PERIPHID1__PARTNUMBER1__MASK 0xf
#define PL022__PERIPHID1__PARTNUMBER1__POS 0
#define PL022__PERIPHID1__PARTNUMBER1__WIDTH 4


// Register 'periphid2': Peripheral identification registers, SSPPeriphID0-3 on page 3-13
// Peripheral identification registers, SSPPeriphID0-3 on page 3-13
// Offset: 0xfe8
// Access mode: rw
#define PL022__PERIPHID2__OFFSET 0xfe8
// These bits return the peripheral revision
#define PL022__PERIPHID2__REVISION__MASK 0xf0
#define PL022__PERIPHID2__REVISION__POS 4
#define PL022__PERIPHID2__REVISION__WIDTH 4

// These bits read back as 0x4
#define PL022__PERIPHID2__DESIGNER1__MASK 0xf
#define PL022__PERIPHID2__DESIGNER1__POS 0
#define PL022__PERIPHID2__DESIGNER1__WIDTH 4


// Register 'periphid3': Peripheral identification registers, SSPPeriphID0-3 on page 3-13
// Peripheral identification registers, SSPPeriphID0-3 on page 3-13
// Offset: 0xfec
// Access mode: rw
#define PL022__PERIPHID3__OFFSET 0xfec
// These bits read back as 0x00
#define PL022__PERIPHID3__CONFIGURATION__MASK 0xff
#define PL022__PERIPHID3__CONFIGURATION__POS 0
#define PL022__PERIPHID3__CONFIGURATION__WIDTH 8


// Register 'pcellid0': PrimeCell identification registers, SSPPCellID0-3 on page 3-16
// PrimeCell identification registers, SSPPCellID0-3 on page 3-16
// Offset: 0xff0
// Access mode: rw
#define PL022__PCELLID0__OFFSET 0xff0
// These bits read back as 0x0D
#define PL022__PCELLID0__PCELLID0__MASK 0xff
#define PL022__PCELLID0__PCELLID0__POS 0
#define PL022__PCELLID0__PCELLID0__WIDTH 8


// Register 'pcellid1': PrimeCell identification registers, SSPPCellID0-3 on page 3-16
// PrimeCell identification registers, SSPPCellID0-3 on page 3-16
// Offset: 0xff4
// Access mode: rw
#define PL022__PCELLID1__OFFSET 0xff4
// These bits read back as 0xF0
#define PL022__PCELLID1__PCELLID1__MASK 0xff
#define PL022__PCELLID1__PCELLID1__POS 0
#define PL022__PCELLID1__PCELLID1__WIDTH 8


// Register 'pcellid2': PrimeCell identification registers, SSPPCellID0-3 on page 3-16
// PrimeCell identification registers, SSPPCellID0-3 on page 3-16
// Offset: 0xff8
// Access mode: rw
#define PL022__PCELLID2__OFFSET 0xff8
// These bits read back as 0x05
#define PL022__PCELLID2__PCELLID2__MASK 0xff
#define PL022__PCELLID2__PCELLID2__POS 0
#define PL022__PCELLID2__PCELLID2__WIDTH 8


// Register 'pcellid3': PrimeCell identification registers, SSPPCellID0-3 on page 3-16
// PrimeCell identification registers, SSPPCellID0-3 on page 3-16
// Offset: 0xffc
// Access mode: rw
#define PL022__PCELLID3__OFFSET 0xffc
// These bits read back as 0xB1
#define PL022__PCELLID3__PCELLID3__MASK 0xff
#define PL022__PCELLID3__PCELLID3__POS 0
#define PL022__PCELLID3__PCELLID3__WIDTH 8



#define pl022_reg_addr(inst, reg)                                    \
    ((uint32_t)(inst) + PL022__##reg##__OFFSET)

#define pl022_reg_idx_addr(inst, reg, idx)                                    \
    ((uint32_t)(inst) + PL022__##reg##__OFFSET + ((idx) * PL022__##reg##__STRIDE))


#define pl022_set32(inst, reg, value) \
    cpu_mem_write_32(pl022_reg_addr(inst, reg), (value))

#define pl022_get32(inst, reg)                                   \
    cpu_mem_read_32(pl022_reg_addr(inst, reg))

#define pl022_idx_set32(inst, reg, idx, value)                             \
    cpu_mem_write_32(pl022_reg_idx_addr(inst, reg, idx), (value))

#define pl022_idx_get32(inst, reg, idx)                                   \
    cpu_mem_read_32(pl022_reg_idx_addr(inst, reg, idx))


#define pl022_get(regval, reg, field)                                \
    (((regval) & PL022__##reg##__##field##__MASK)                         \
     >> PL022__##reg##__##field##__POS)

#define pl022_ins_val(regval, reg, field, value)                       \
    (((regval) & ~PL022__##reg##__##field##__MASK) |                        \
     (PL022__##reg##__##field##__MASK & ((value) << PL022__##reg##__##field##__POS)))

#define pl022_enum(reg, field, symbol)                           \
    (PL022__##reg##__##field##__ENUM__##symbol)

#define pl022_ins_enum(regval, reg, field, symbol)                      \
    pl022_ins_val((regval), reg, field, pl022_enum(reg, field, symbol))

#define pl022_bit(reg, field)                           \
    (PL022__##reg##__##field##__MASK)

#endif
