/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs cdefs_use_field_setval=1 cdefs_use_field_set=1               \
     -O soclib_spi.h
*/

#ifndef _SOCLIB_SPI_BFGEN_DEFS_
#define _SOCLIB_SPI_BFGEN_DEFS_

#define SOCLIB_SPI_CTRL_ADDR                         0x00000000
/** Bit width of the SPI transfer word minus one. @multiple */
  #define SOCLIB_SPI_CTRL_WSIZE(v)                 ((v) << 0)
  #define SOCLIB_SPI_CTRL_WSIZE_SET(x, v)          do { (x) = (((x) & ~0x1f) | ((v) << 0)); } while(0)
  #define SOCLIB_SPI_CTRL_WSIZE_GET(x)             (((x) >> 0) & 0x1f)
/** This bit defines the SPI word transmit bit order @multiple */
  #define SOCLIB_SPI_CTRL_LSBF(v)                  ((SOCLIB_SPI_CTRL_LSBF_##v) << 8)
  #define SOCLIB_SPI_CTRL_LSBF_SET(x, v)           do { (x) = (((x) & ~0x100) | ((SOCLIB_SPI_CTRL_LSBF_##v) << 8)); } while(0)
  #define SOCLIB_SPI_CTRL_LSBF_SETVAL(x, v)        do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define SOCLIB_SPI_CTRL_LSBF_GET(x)              (((x) >> 8) & 0x1)
/** MSB first */
    #define SOCLIB_SPI_CTRL_LSBF_MSBF                0x00000000
/** LSB first */
    #define SOCLIB_SPI_CTRL_LSBF_LSBF                0x00000001
/** This bit defines the SPI clock polarity @multiple */
  #define SOCLIB_SPI_CTRL_CKPOL(v)                 ((SOCLIB_SPI_CTRL_CKPOL_##v) << 9)
  #define SOCLIB_SPI_CTRL_CKPOL_SET(x, v)          do { (x) = (((x) & ~0x200) | ((SOCLIB_SPI_CTRL_CKPOL_##v) << 9)); } while(0)
  #define SOCLIB_SPI_CTRL_CKPOL_SETVAL(x, v)       do { (x) = (((x) & ~0x200) | ((v) << 9)); } while(0)
  #define SOCLIB_SPI_CTRL_CKPOL_GET(x)             (((x) >> 9) & 0x1)
    #define SOCLIB_SPI_CTRL_CKPOL_RISING             0x00000000
    #define SOCLIB_SPI_CTRL_CKPOL_FALLING            0x00000001
/** This bit defines the SPI clock phase @multiple */
  #define SOCLIB_SPI_CTRL_CKPHA(v)                 ((SOCLIB_SPI_CTRL_CKPHA_##v) << 10)
  #define SOCLIB_SPI_CTRL_CKPHA_SET(x, v)          do { (x) = (((x) & ~0x400) | ((SOCLIB_SPI_CTRL_CKPHA_##v) << 10)); } while(0)
  #define SOCLIB_SPI_CTRL_CKPHA_SETVAL(x, v)       do { (x) = (((x) & ~0x400) | ((v) << 10)); } while(0)
  #define SOCLIB_SPI_CTRL_CKPHA_GET(x)             (((x) >> 10) & 0x1)
    #define SOCLIB_SPI_CTRL_CKPHA_NORMAL             0x00000000
    #define SOCLIB_SPI_CTRL_CKPHA_INVERSE            0x00000001
/** When set the MISO and MOSI signal are active low. @multiple */
  #define SOCLIB_SPI_CTRL_DPOL(v)                  ((SOCLIB_SPI_CTRL_DPOL_##v) << 11)
  #define SOCLIB_SPI_CTRL_DPOL_SET(x, v)           do { (x) = (((x) & ~0x800) | ((SOCLIB_SPI_CTRL_DPOL_##v) << 11)); } while(0)
  #define SOCLIB_SPI_CTRL_DPOL_SETVAL(x, v)        do { (x) = (((x) & ~0x800) | ((v) << 11)); } while(0)
  #define SOCLIB_SPI_CTRL_DPOL_GET(x)              (((x) >> 11) & 0x1)
    #define SOCLIB_SPI_CTRL_DPOL_NORMAL              0x00000000
    #define SOCLIB_SPI_CTRL_DPOL_INVERTED            0x00000001
/** When set the RX fifo remains empty. @multiple */
  #define SOCLIB_SPI_CTRL_NORX(v)                  ((SOCLIB_SPI_CTRL_NORX_##v) << 12)
  #define SOCLIB_SPI_CTRL_NORX_SET(x, v)           do { (x) = (((x) & ~0x1000) | ((SOCLIB_SPI_CTRL_NORX_##v) << 12)); } while(0)
  #define SOCLIB_SPI_CTRL_NORX_SETVAL(x, v)        do { (x) = (((x) & ~0x1000) | ((v) << 12)); } while(0)
  #define SOCLIB_SPI_CTRL_NORX_GET(x)              (((x) >> 12) & 0x1)
    #define SOCLIB_SPI_CTRL_NORX_RX                  0x00000000
    #define SOCLIB_SPI_CTRL_NORX_NO_RX               0x00000001
/** When set the SPI controller is in reset state. (fifos cleared, tlen = 0)
   @multiple */
  #define SOCLIB_SPI_CTRL_RESET(v)                 ((SOCLIB_SPI_CTRL_RESET_##v) << 13)
  #define SOCLIB_SPI_CTRL_RESET_SET(x, v)          do { (x) = (((x) & ~0x2000) | ((SOCLIB_SPI_CTRL_RESET_##v) << 13)); } while(0)
  #define SOCLIB_SPI_CTRL_RESET_SETVAL(x, v)       do { (x) = (((x) & ~0x2000) | ((v) << 13)); } while(0)
  #define SOCLIB_SPI_CTRL_RESET_GET(x)             (((x) >> 13) & 0x1)
    #define SOCLIB_SPI_CTRL_RESET_NORMAL             0x00000000
    #define SOCLIB_SPI_CTRL_RESET_RESET              0x00000001

/** This register contains the hardware block instantiation parameters @multiple
   */
#define SOCLIB_SPI_CONFIG_ADDR                       0x00000004
/** Size of the RX and TX fifos @multiple */
  #define SOCLIB_SPI_CONFIG_FSIZE(v)               ((SOCLIB_SPI_CONFIG_FSIZE_##v) << 0)
  #define SOCLIB_SPI_CONFIG_FSIZE_SET(x, v)        do { (x) = (((x) & ~0x7) | ((SOCLIB_SPI_CONFIG_FSIZE_##v) << 0)); } while(0)
  #define SOCLIB_SPI_CONFIG_FSIZE_SETVAL(x, v)     do { (x) = (((x) & ~0x7) | ((v) << 0)); } while(0)
  #define SOCLIB_SPI_CONFIG_FSIZE_GET(x)           (((x) >> 0) & 0x7)
    #define SOCLIB_SPI_CONFIG_FSIZE_2_WORDS          0x00000000
    #define SOCLIB_SPI_CONFIG_FSIZE_4_WORDS          0x00000001
    #define SOCLIB_SPI_CONFIG_FSIZE_8_WORDS          0x00000002
    #define SOCLIB_SPI_CONFIG_FSIZE_16_WORDS         0x00000003
    #define SOCLIB_SPI_CONFIG_FSIZE_32_WORDS         0x00000004
    #define SOCLIB_SPI_CONFIG_FSIZE_64_WORDS         0x00000005
    #define SOCLIB_SPI_CONFIG_FSIZE_128_WORDS        0x00000006
    #define SOCLIB_SPI_CONFIG_FSIZE_256_WORDS        0x00000007
/** Maximum bit width of the SPI transfer word minus one. @multiple */
  #define SOCLIB_SPI_CONFIG_WSIZE(v)               ((v) << 8)
  #define SOCLIB_SPI_CONFIG_WSIZE_SET(x, v)        do { (x) = (((x) & ~0x1f00) | ((v) << 8)); } while(0)
  #define SOCLIB_SPI_CONFIG_WSIZE_GET(x)           (((x) >> 8) & 0x1f)
/** Number of general purpose input lines minus one @multiple */
  #define SOCLIB_SPI_CONFIG_GPINCNT(v)             ((v) << 13)
  #define SOCLIB_SPI_CONFIG_GPINCNT_SET(x, v)      do { (x) = (((x) & ~0x3e000) | ((v) << 13)); } while(0)
  #define SOCLIB_SPI_CONFIG_GPINCNT_GET(x)         (((x) >> 13) & 0x1f)
/** Number of general purpose output lines minus one @multiple */
  #define SOCLIB_SPI_CONFIG_GPOUTCNT(v)            ((v) << 18)
  #define SOCLIB_SPI_CONFIG_GPOUTCNT_SET(x, v)     do { (x) = (((x) & ~0x7c0000) | ((v) << 18)); } while(0)
  #define SOCLIB_SPI_CONFIG_GPOUTCNT_GET(x)        (((x) >> 18) & 0x1f)

/** This indicates the SPI clock divider. spi_clk=(clk/(clkdiv+1)/2) @multiple */
#define SOCLIB_SPI_CLKDIV_ADDR                       0x00000008

/** Write access to the TX fifo and read access from the RX fifo. TX and RX fifo
   are updated at the same time when a SPI word transfer completes. A new
   transfer is started only if the RX fifo is not full and the TX fifo is not
   empty. @multiple */
#define SOCLIB_SPI_FIFO_ADDR                         0x0000000c

/** This register contains the value of up to 32 general purpose output lines.
   @multiple */
#define SOCLIB_SPI_GPOUT_ADDR                        0x00000010
  #define SOCLIB_SPI_GPOUT_PIN_COUNT               32
  #define SOCLIB_SPI_GPOUT_PIN(fidx)               (0x00000001 << ((fidx)))
  #define SOCLIB_SPI_GPOUT_PIN_SET(fidx, x, v)     do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)

/** When read this register return the logic level of up to 32 input lines. When
   written this register set the interrupt mask of input lines. @multiple */
#define SOCLIB_SPI_GPIN_ADDR                         0x00000014
  #define SOCLIB_SPI_GPIN_PIN_COUNT                32
  #define SOCLIB_SPI_GPIN_PIN(fidx)                (0x00000001 << ((fidx)))
  #define SOCLIB_SPI_GPIN_PIN_SET(fidx, x, v)      do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)

/** This bit defines the irq sense mode of general purpose input lines. @multiple
   */
#define SOCLIB_SPI_GPIRQ_MODE_ADDR                   0x00000018
  #define SOCLIB_SPI_GPIRQ_MODE_PIN_COUNT          32
  #define SOCLIB_SPI_GPIRQ_MODE_PIN(fidx, v)       ((SOCLIB_SPI_GPIRQ_MODE_PIN_##v) << ((fidx) + 0))
  #define SOCLIB_SPI_GPIRQ_MODE_PIN_SET(fidx, x, v) do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((SOCLIB_SPI_GPIRQ_MODE_PIN_##v) << ((fidx) + 0))); } while(0)
  #define SOCLIB_SPI_GPIRQ_MODE_PIN_SETVAL(fidx, x, v) do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)
  #define SOCLIB_SPI_GPIRQ_MODE_PIN_GET(fidx, x)   (((x) >> ((fidx) + 0)) & 0x1)
/** The input bit of the irq pending register changes to one when an edge is detected a non-masked input. */
  #define SOCLIB_SPI_GPIRQ_MODE_PIN_EDGE           0x00000000
/** The input bit of the irq pending register is one if at least one the non-masked input is active. */
  #define SOCLIB_SPI_GPIRQ_MODE_PIN_LEVEL          0x00000001

/** This bit defines the irq signal polarity of general purpose input lines.
   @multiple */
#define SOCLIB_SPI_GPIRQ_POL_ADDR                    0x0000001c
  #define SOCLIB_SPI_GPIRQ_POL_PIN_COUNT           32
  #define SOCLIB_SPI_GPIRQ_POL_PIN(fidx, v)        ((SOCLIB_SPI_GPIRQ_POL_PIN_##v) << ((fidx) + 0))
  #define SOCLIB_SPI_GPIRQ_POL_PIN_SET(fidx, x, v) do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((SOCLIB_SPI_GPIRQ_POL_PIN_##v) << ((fidx) + 0))); } while(0)
  #define SOCLIB_SPI_GPIRQ_POL_PIN_SETVAL(fidx, x, v) do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)
  #define SOCLIB_SPI_GPIRQ_POL_PIN_GET(fidx, x)    (((x) >> ((fidx) + 0)) & 0x1)
/** Rising edge or active high */
  #define SOCLIB_SPI_GPIRQ_POL_PIN_RISING          0x00000000
/** Falling edge or active low */
  #define SOCLIB_SPI_GPIRQ_POL_PIN_FALLING         0x00000001

/** This bit mask is xored into the gpout register when the tlen register is
   loaded and when it decrement from 1 to 0. @multiple */
#define SOCLIB_SPI_CSTGL_ADDR                        0x00000020
  #define SOCLIB_SPI_CSTGL_PIN_COUNT               32
  #define SOCLIB_SPI_CSTGL_PIN(fidx, v)            ((SOCLIB_SPI_CSTGL_PIN_##v) << ((fidx) + 0))
  #define SOCLIB_SPI_CSTGL_PIN_SET(fidx, x, v)     do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((SOCLIB_SPI_CSTGL_PIN_##v) << ((fidx) + 0))); } while(0)
  #define SOCLIB_SPI_CSTGL_PIN_SETVAL(fidx, x, v)  do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)
  #define SOCLIB_SPI_CSTGL_PIN_GET(fidx, x)        (((x) >> ((fidx) + 0)) & 0x1)
/** Don't change the output line */
  #define SOCLIB_SPI_CSTGL_PIN_NONE                0x00000000
/** Toggle the output line */
  #define SOCLIB_SPI_CSTGL_PIN_TOGGLE              0x00000001

/** This register is decreased when a SPI word has been transfered. @multiple */
#define SOCLIB_SPI_TLEN_ADDR                         0x00000024

#define SOCLIB_SPI_STATUS_ADDR                       0x00000028
/** This bit is set when the TX fifo is full @multiple */
  #define SOCLIB_SPI_STATUS_TXFULL                 0x00000001
  #define SOCLIB_SPI_STATUS_TXFULL_SET(x, v)       do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
/** This bit is set when the TX fifo is empty @multiple */
  #define SOCLIB_SPI_STATUS_TXEMPTY                0x00000002
  #define SOCLIB_SPI_STATUS_TXEMPTY_SET(x, v)      do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
/** This bit is set when the TX fifo is half empty @multiple */
  #define SOCLIB_SPI_STATUS_TXHALF                 0x00000004
  #define SOCLIB_SPI_STATUS_TXHALF_SET(x, v)       do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
/** This bit is set when the RX fifo is full @multiple */
  #define SOCLIB_SPI_STATUS_RXFULL                 0x00000100
  #define SOCLIB_SPI_STATUS_RXFULL_SET(x, v)       do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
/** This bit is set when the RX fifo is empty @multiple */
  #define SOCLIB_SPI_STATUS_RXEMPTY                0x00000200
  #define SOCLIB_SPI_STATUS_RXEMPTY_SET(x, v)      do { (x) = (((x) & ~0x200) | ((v) << 9)); } while(0)
/** This bit is set when the RX fifo is half full @multiple */
  #define SOCLIB_SPI_STATUS_RXHALF                 0x00000400
  #define SOCLIB_SPI_STATUS_RXHALF_SET(x, v)       do { (x) = (((x) & ~0x400) | ((v) << 10)); } while(0)

/** This register is cleared when read. The irq output line of the SPI controller
   is active when this register is not zero. @multiple */
#define SOCLIB_SPI_IRQPEND_ADDR                      0x0000002c
/** This bit is set when the TX fifo state changes to full. @multiple */
  #define SOCLIB_SPI_IRQPEND_TXFULL                0x00000001
  #define SOCLIB_SPI_IRQPEND_TXFULL_SET(x, v)      do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
/** This bit is set when the TX fifo state changes to empty. @multiple */
  #define SOCLIB_SPI_IRQPEND_TXEMPTY               0x00000002
  #define SOCLIB_SPI_IRQPEND_TXEMPTY_SET(x, v)     do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
/** This bit is set when the TX fifo state changes to half empty. @multiple */
  #define SOCLIB_SPI_IRQPEND_TXHALF                0x00000004
  #define SOCLIB_SPI_IRQPEND_TXHALF_SET(x, v)      do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
/** This bit is set when the tlen register changes from 1 to 0. @multiple */
  #define SOCLIB_SPI_IRQPEND_DONE                  0x00000008
  #define SOCLIB_SPI_IRQPEND_DONE_SET(x, v)        do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
/** This bit is set when the RX fifo state changes to full. @multiple */
  #define SOCLIB_SPI_IRQPEND_RXFULL                0x00000010
  #define SOCLIB_SPI_IRQPEND_RXFULL_SET(x, v)      do { (x) = (((x) & ~0x10) | ((v) << 4)); } while(0)
/** This bit is set when the RX fifo state changes to empty. @multiple */
  #define SOCLIB_SPI_IRQPEND_RXEMPTY               0x00000020
  #define SOCLIB_SPI_IRQPEND_RXEMPTY_SET(x, v)     do { (x) = (((x) & ~0x20) | ((v) << 5)); } while(0)
/** This bit is set when the RX fifo state changes to half full. @multiple */
  #define SOCLIB_SPI_IRQPEND_RXHALF                0x00000040
  #define SOCLIB_SPI_IRQPEND_RXHALF_SET(x, v)      do { (x) = (((x) & ~0x40) | ((v) << 6)); } while(0)
/** This bit is set when the irq condition is detected on an input line. This bit
   will return to zero if the irq mode is level and no more irq condition is
   detected. @multiple */
  #define SOCLIB_SPI_IRQPEND_GPIRQ                 0x00000080
  #define SOCLIB_SPI_IRQPEND_GPIRQ_SET(x, v)       do { (x) = (((x) & ~0x80) | ((v) << 7)); } while(0)
/** When the gpirq bit is set, this field indicates the index of the lower input
   line which triggered the irq. This field is zero when no irq is pending.
   @multiple */
  #define SOCLIB_SPI_IRQPEND_GPIRQN(v)             ((v) << 24)
  #define SOCLIB_SPI_IRQPEND_GPIRQN_SET(x, v)      do { (x) = (((x) & ~0x1f000000) | ((v) << 24)); } while(0)
  #define SOCLIB_SPI_IRQPEND_GPIRQN_GET(x)         (((x) >> 24) & 0x1f)

/** This register specify which irq lines are enabled @multiple */
#define SOCLIB_SPI_IRQMASK_ADDR                      0x00000030
  #define SOCLIB_SPI_IRQMASK_TXFULL                0x00000001
  #define SOCLIB_SPI_IRQMASK_TXFULL_SET(x, v)      do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define SOCLIB_SPI_IRQMASK_TXEMPTY               0x00000002
  #define SOCLIB_SPI_IRQMASK_TXEMPTY_SET(x, v)     do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define SOCLIB_SPI_IRQMASK_TXHALF                0x00000004
  #define SOCLIB_SPI_IRQMASK_TXHALF_SET(x, v)      do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
  #define SOCLIB_SPI_IRQMASK_DONE                  0x00000008
  #define SOCLIB_SPI_IRQMASK_DONE_SET(x, v)        do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
  #define SOCLIB_SPI_IRQMASK_RXFULL                0x00000010
  #define SOCLIB_SPI_IRQMASK_RXFULL_SET(x, v)      do { (x) = (((x) & ~0x10) | ((v) << 4)); } while(0)
  #define SOCLIB_SPI_IRQMASK_RXEMPTY               0x00000020
  #define SOCLIB_SPI_IRQMASK_RXEMPTY_SET(x, v)     do { (x) = (((x) & ~0x20) | ((v) << 5)); } while(0)
  #define SOCLIB_SPI_IRQMASK_RXHALF                0x00000040
  #define SOCLIB_SPI_IRQMASK_RXHALF_SET(x, v)      do { (x) = (((x) & ~0x40) | ((v) << 6)); } while(0)
  #define SOCLIB_SPI_IRQMASK_GPIRQ                 0x00000080
  #define SOCLIB_SPI_IRQMASK_GPIRQ_SET(x, v)       do { (x) = (((x) & ~0x80) | ((v) << 7)); } while(0)

#endif

