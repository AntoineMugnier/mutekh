#ifndef ARM_PL022_H_
#define ARM_PL022_H_

// PL022.SSPCR0 Control register 0, SSPCR0
#define PL022_SSPCR0_OFFSET 0x00000000

// PL022.SSPCR0_SCR Serial clock rate. The value SCR is used to
// generate the transmit and receive bit rate of the PrimeCell
// SSP. The bit rate is: F SSPCLK CPSDVSR x (1+SCR) where CPSDVSR is
// an even value from 2-254, programmed through the SSPCPSR register
// and SCR is a value from 0-255.
#define PL022_SSPCR0_SCR_MASK    0xff00
#define PL022_SSPCR0_SCR_WIDTH    8
#define PL022_SSPCR0_SCR_POS    8

// PL022.SSPCR0_SPH SSPCLKOUT phase, applicable to Motorola SPI frame
// format only.  See Motorola SPI frame format
#define PL022_SSPCR0_SPH_MASK    0x80
#define PL022_SSPCR0_SPH_WIDTH    1
#define PL022_SSPCR0_SPH_POS    7

// PL022.SSPCR0_SPO SSPCLKOUT polarity, applicable to Motorola SPI
// frame format only. See Motorola SPI frame format
#define PL022_SSPCR0_SPO_MASK    0x40
#define PL022_SSPCR0_SPO_WIDTH    1
#define PL022_SSPCR0_SPO_POS    6

// PL022.SSPCR0_FRF Frame format: 00 Motorola SPI frame format. 01 TI
// synchronous serial frame format. 10 National Microwire frame
// format. 11 Reserved, undefined operation.
#define PL022_SSPCR0_FRF_MASK    0x30
#define PL022_SSPCR0_FRF_WIDTH    2
#define PL022_SSPCR0_FRF_POS    4
#define PL022_SSPCR0_FRF_MOTOROLA  0
#define PL022_SSPCR0_FRF_TI        1
#define PL022_SSPCR0_FRF_MICROWIRE 2
#define PL022_SSPCR0_FRF_RESERVED  3

// PL022.SSPCR0_DSS Data Size Select, 1 less than bit count
#define PL022_SSPCR0_DSS_MASK    0x0f
#define PL022_SSPCR0_DSS_WIDTH    4
#define PL022_SSPCR0_DSS_POS    0


// PL022.SSPCR1 Control register 1, SSPCR1
#define PL022_SSPCR1_OFFSET 0x00000004

// PL022.SSPCR1_SOD Slave-mode output disable. This bit is relevant
// only in the slave mode, MS=1. In multiple-slave systems, it is
// possible for an PrimeCell SSP master to broadcast a message to all
// slaves in the system while ensuring that only one slave drives data
// onto its serial output line. In such systems the RXD lines from
// multiple slaves could be tied together. To operate in such systems,
// the SOD bit can be set if the PrimeCell SSP slave is not supposed
// to drive the SSPTXD line:
// - 0 SSP can drive the SSPTXD output in slave mode.
// - 1 SSP must not drive the SSPTXD output in slave mode.
#define PL022_SSPCR1_SOD_MASK    0x08
#define PL022_SSPCR1_SOD_WIDTH    1
#define PL022_SSPCR1_SOD_POS    3

// PL022.SSPCR1_MS Master or slave mode select. This bit can be
// modified only when the PrimeCell SSP is disabled, SSE=0.
#define PL022_SSPCR1_MS_MASK    0x04
#define PL022_SSPCR1_MS_WIDTH    1
#define PL022_SSPCR1_MS_POS    2
#define PL022_SSPCR1_MS_MASTER 0
#define PL022_SSPCR1_MS_SLAVE  1

// PL022.SSPCR1_SSE Synchronous serial port enable
#define PL022_SSPCR1_SSE_MASK    0x02
#define PL022_SSPCR1_SSE_WIDTH    1
#define PL022_SSPCR1_SSE_POS    1

// PL022.SSPCR1_LBM Loop back mode
#define PL022_SSPCR1_LBM_MASK    0x01
#define PL022_SSPCR1_LBM_WIDTH    1
#define PL022_SSPCR1_LBM_POS    0


// PL022.SSPDR Data register, SSPDR
#define PL022_SSPDR_OFFSET 0x00000008

// PL022.SSPDR_DATA Transmit/Receive FIFO: Read Receive FIFO. Write
// Transmit FIFO.  You must right-justify data when the PrimeCell SSP
// is programmed for a data size that is less than 16 bits. Unused
// bits at the top are ignored by transmit logic. The receive logic
// automatically right-justifies.
#define PL022_SSPDR_DATA_MASK    0xffff
#define PL022_SSPDR_DATA_WIDTH    16
#define PL022_SSPDR_DATA_POS    0


// PL022.SSPSR Status register, SSPSR
#define PL022_SSPSR_OFFSET 0x0000000c

// PL022.SSPSR_BSY PrimeCell SSP busy flag, RO
#define PL022_SSPSR_BSY_MASK    0x10
#define PL022_SSPSR_BSY_WIDTH    1
#define PL022_SSPSR_BSY_POS    4

// PL022.SSPSR_RFF Receive FIFO full, RO
#define PL022_SSPSR_RFF_MASK    0x08
#define PL022_SSPSR_RFF_WIDTH    1
#define PL022_SSPSR_RFF_POS    3

// PL022.SSPSR_RNE Receive FIFO not empty, RO
#define PL022_SSPSR_RNE_MASK    0x04
#define PL022_SSPSR_RNE_WIDTH    1
#define PL022_SSPSR_RNE_POS    2

// PL022.SSPSR_TNF Transmit FIFO not full, RO
#define PL022_SSPSR_TNF_MASK    0x02
#define PL022_SSPSR_TNF_WIDTH    1
#define PL022_SSPSR_TNF_POS    1

// PL022.SSPSR_TFE Transmit FIFO empty, RO
#define PL022_SSPSR_TFE_MASK    0x01
#define PL022_SSPSR_TFE_WIDTH    1
#define PL022_SSPSR_TFE_POS    0


// PL022.SSPCPSR Clock prescale register, SSPCPSR
#define PL022_SSPCPSR_OFFSET 0x00000010

// PL022.SSPCPSR_CPSDVSR Clock prescale divisor. Must be an even
// number from 2-254, depending on the frequency of SSPCLK. The least
// significant bit always returns zero on reads.
#define PL022_SSPCPSR_CPSDVSR_MASK    0xff
#define PL022_SSPCPSR_CPSDVSR_WIDTH    8
#define PL022_SSPCPSR_CPSDVSR_POS    0


// PL022.SSPIMSC Interrupt mask set or clear register, SSPIMSC
#define PL022_SSPIMSC_OFFSET 0x00000014

// PL022.SSPIMSC_TXIM Transmit FIFO interrupt mask
#define PL022_SSPIMSC_TXIM_MASK    0x08
#define PL022_SSPIMSC_TXIM_WIDTH    1
#define PL022_SSPIMSC_TXIM_POS    3

// PL022.SSPIMSC_RXIM Receive FIFO interrupt mask
#define PL022_SSPIMSC_RXIM_MASK    0x04
#define PL022_SSPIMSC_RXIM_WIDTH    1
#define PL022_SSPIMSC_RXIM_POS    2

// PL022.SSPIMSC_RTIM Receive timeout interrupt mask
#define PL022_SSPIMSC_RTIM_MASK    0x02
#define PL022_SSPIMSC_RTIM_WIDTH    1
#define PL022_SSPIMSC_RTIM_POS    1

// PL022.SSPIMSC_RORIM Receive overrun interrupt mask
#define PL022_SSPIMSC_RORIM_MASK    0x01
#define PL022_SSPIMSC_RORIM_WIDTH    1
#define PL022_SSPIMSC_RORIM_POS    0


// PL022.SSPRIS Raw interrupt status register, SSPRIS
#define PL022_SSPRIS_OFFSET 0x00000018

// PL022.SSPRIS_TXRIS Gives the raw interrupt state, prior to masking,
// of the SSPTXINTR interrupt
#define PL022_SSPRIS_TXRIS_MASK    0x08
#define PL022_SSPRIS_TXRIS_WIDTH    1
#define PL022_SSPRIS_TXRIS_POS    3

// PL022.SSPRIS_RXRIS Gives the raw interrupt state, prior to masking,
// of the SSPRXINTR interrupt
#define PL022_SSPRIS_RXRIS_MASK    0x04
#define PL022_SSPRIS_RXRIS_WIDTH    1
#define PL022_SSPRIS_RXRIS_POS    2

// PL022.SSPRIS_RTRIS Gives the raw interrupt state, prior to masking,
// of the SSPRTINTR interrupt
#define PL022_SSPRIS_RTRIS_MASK    0x02
#define PL022_SSPRIS_RTRIS_WIDTH    1
#define PL022_SSPRIS_RTRIS_POS    1

// PL022.SSPRIS_RORRIS Gives the raw interrupt state, prior to
// masking, of the SSPRORINTR interrupt
#define PL022_SSPRIS_RORRIS_MASK    0x01
#define PL022_SSPRIS_RORRIS_WIDTH    1
#define PL022_SSPRIS_RORRIS_POS    0


// PL022.SSPMIS Masked interrupt status register, SSPMIS
#define PL022_SSPMIS_OFFSET 0x0000001c

// PL022.SSPMIS_TXMIS Gives the transmit FIFO masked interrupt state,
// after masking, of the SSPTXINTR interrupt
#define PL022_SSPMIS_TXMIS_MASK    0x08
#define PL022_SSPMIS_TXMIS_WIDTH    1
#define PL022_SSPMIS_TXMIS_POS    3

// PL022.SSPMIS_RXMIS Gives the receive FIFO masked interrupt state,
// after masking, of the SSPRXINTR interrupt
#define PL022_SSPMIS_RXMIS_MASK    0x04
#define PL022_SSPMIS_RXMIS_WIDTH    1
#define PL022_SSPMIS_RXMIS_POS    2

// PL022.SSPMIS_RTMIS Gives the receive timeout masked interrupt
// state, after masking, of the SSPRTINTR interrupt
#define PL022_SSPMIS_RTMIS_MASK    0x02
#define PL022_SSPMIS_RTMIS_WIDTH    1
#define PL022_SSPMIS_RTMIS_POS    1

// PL022.SSPMIS_RORMIS Gives the receive over run masked interrupt
// status, after masking, of the SSPRORINTR interrupt
#define PL022_SSPMIS_RORMIS_MASK    0x01
#define PL022_SSPMIS_RORMIS_WIDTH    1
#define PL022_SSPMIS_RORMIS_POS    0


// PL022.SSPICR Interrupt clear register, SSPICR
#define PL022_SSPICR_OFFSET 0x00000020

// PL022.SSPICR_RTIC Clears the SSPRTINTR interrupt
#define PL022_SSPICR_RTIC_MASK    0x02
#define PL022_SSPICR_RTIC_WIDTH    1
#define PL022_SSPICR_RTIC_POS    1

// PL022.SSPICR_RORIC Clears the SSPRORINTR interrupt
#define PL022_SSPICR_RORIC_MASK    0x01
#define PL022_SSPICR_RORIC_WIDTH    1
#define PL022_SSPICR_RORIC_POS    0


// PL022.SSPDMACR DMA control register, SSPDMACR
#define PL022_SSPDMACR_OFFSET 0x00000024

// PL022.SSPDMACR_TXDMAE Transmit DMA Enable. If this bit is set to 1,
// DMA for the transmit FIFO is enabled.
#define PL022_SSPDMACR_TXDMAE_MASK    0x02
#define PL022_SSPDMACR_TXDMAE_WIDTH    1
#define PL022_SSPDMACR_TXDMAE_POS    1

// PL022.SSPDMACR_RXDMAE Receive DMA Enable. If this bit is set to 1,
// DMA for the receive FIFO is enabled.
#define PL022_SSPDMACR_RXDMAE_MASK    0x01
#define PL022_SSPDMACR_RXDMAE_WIDTH    1
#define PL022_SSPDMACR_RXDMAE_POS    0

#define pl022_reg_addr(inst, name)                             \
  ((uint32_t)(inst) + PL022_##name##_OFFSET)

#define pl022_set32(inst, name, value)                             \
    cpu_mem_write_32(pl022_reg_addr(inst, name), (value))

#define pl022_get32(inst, name)                                   \
    cpu_mem_read_32(pl022_reg_addr(inst, name))

#define pl022_set8(inst, name, value)                             \
    cpu_mem_write_8(pl022_reg_addr(inst, name), (value))

#define pl022_get8(inst, name)                                   \
    cpu_mem_read_8(pl022_reg_addr(inst, name))

#define pl022_get(v32, reg, field)                                \
    ((v32 & PL022_##reg##_##field##_MASK)                         \
     >> PL022_##reg##_##field##_POS)

#define pl022_ins_val(v32, reg, field, value)                       \
    ((v32 & ~PL022_##reg##_##field##_MASK) |                        \
     (PL022_##reg##_##field##_MASK & ((value) << PL022_##reg##_##field##_POS)))

#define pl022_ins_enum(v32, reg, field, value)                      \
    pl022_ins_val(v32, reg, field, PL022_##reg##_##field##_##value)

#define pl022_bit(reg, field)                           \
    (PL022_##reg##_##field##_MASK)

#endif
