/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs -O include/arch/efm32/leuart.h cdefs_use_reg_mask=1          \
     cdefs_use_field_setval=1
*/

#ifndef _EFM32_LEUART_BFGEN_DEFS_
#define _EFM32_LEUART_BFGEN_DEFS_

#define EFM32_LEUART_CTRL_ADDR                       0x00000000
#define EFM32_LEUART_CTRL_MASK                       0x0000ffff
/** When set, LEUn_TX is tristated whenever the transmitter is inactive.
   @multiple */
  #define EFM32_LEUART_CTRL_AUTOTRI(v)               ((EFM32_LEUART_CTRL_AUTOTRI_##v) << 0)
  #define EFM32_LEUART_CTRL_AUTOTRI_SET(x, v)        do { (x) = (((x) & ~0x1) | ((EFM32_LEUART_CTRL_AUTOTRI_##v) << 0)); } while(0)
  #define EFM32_LEUART_CTRL_AUTOTRI_SETVAL(x, v)     do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define EFM32_LEUART_CTRL_AUTOTRI_GET(x)           (((x) >> 0) & 0x1)
    #define EFM32_LEUART_CTRL_AUTOTRI_HIGH             0x00000000
    #define EFM32_LEUART_CTRL_AUTOTRI_TRISTATED        0x00000001
/** This register sets the number of data bits. @multiple */
  #define EFM32_LEUART_CTRL_DATABITS(v)              ((EFM32_LEUART_CTRL_DATABITS_##v) << 1)
  #define EFM32_LEUART_CTRL_DATABITS_SET(x, v)       do { (x) = (((x) & ~0x2) | ((EFM32_LEUART_CTRL_DATABITS_##v) << 1)); } while(0)
  #define EFM32_LEUART_CTRL_DATABITS_SETVAL(x, v)    do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define EFM32_LEUART_CTRL_DATABITS_GET(x)          (((x) >> 1) & 0x1)
    #define EFM32_LEUART_CTRL_DATABITS_EIGHT           0x00000000
    #define EFM32_LEUART_CTRL_DATABITS_NINE            0x00000001
/** Determines whether parity bits are enabled, and whether even or odd parity
   should be used. @multiple */
  #define EFM32_LEUART_CTRL_PARITY(v)                ((EFM32_LEUART_CTRL_PARITY_##v) << 2)
  #define EFM32_LEUART_CTRL_PARITY_SET(x, v)         do { (x) = (((x) & ~0xc) | ((EFM32_LEUART_CTRL_PARITY_##v) << 2)); } while(0)
  #define EFM32_LEUART_CTRL_PARITY_SETVAL(x, v)      do { (x) = (((x) & ~0xc) | ((v) << 2)); } while(0)
  #define EFM32_LEUART_CTRL_PARITY_GET(x)            (((x) >> 2) & 0x3)
    #define EFM32_LEUART_CTRL_PARITY_NONE              0x00000000
    #define EFM32_LEUART_CTRL_PARITY_EVEN              0x00000002
    #define EFM32_LEUART_CTRL_PARITY_ODD               0x00000003
/** Determines the number of stop-bits used. Only used when transmitting data.
   The receiver only verifies that one stop bit is present. @multiple */
  #define EFM32_LEUART_CTRL_STOPBITS(v)              ((EFM32_LEUART_CTRL_STOPBITS_##v) << 4)
  #define EFM32_LEUART_CTRL_STOPBITS_SET(x, v)       do { (x) = (((x) & ~0x10) | ((EFM32_LEUART_CTRL_STOPBITS_##v) << 4)); } while(0)
  #define EFM32_LEUART_CTRL_STOPBITS_SETVAL(x, v)    do { (x) = (((x) & ~0x10) | ((v) << 4)); } while(0)
  #define EFM32_LEUART_CTRL_STOPBITS_GET(x)          (((x) >> 4) & 0x1)
    #define EFM32_LEUART_CTRL_STOPBITS_ONE             0x00000000
    #define EFM32_LEUART_CTRL_STOPBITS_TWO             0x00000001
/** Set to invert the output on LEUn_TX and input on LEUn_RX. @multiple */
  #define EFM32_LEUART_CTRL_INV(v)                   ((EFM32_LEUART_CTRL_INV_##v) << 5)
  #define EFM32_LEUART_CTRL_INV_SET(x, v)            do { (x) = (((x) & ~0x20) | ((EFM32_LEUART_CTRL_INV_##v) << 5)); } while(0)
  #define EFM32_LEUART_CTRL_INV_SETVAL(x, v)         do { (x) = (((x) & ~0x20) | ((v) << 5)); } while(0)
  #define EFM32_LEUART_CTRL_INV_GET(x)               (((x) >> 5) & 0x1)
    #define EFM32_LEUART_CTRL_INV_H1L0                 0x00000000
    #define EFM32_LEUART_CTRL_INV_H0L1                 0x00000001
/** When set, RX DMA requests will be cleared on framing and parity errors.
   @multiple */
  #define EFM32_LEUART_CTRL_ERRSDMA                  0x00000040
/** Set to connect receiver to LEUn_TX instead of LEUn_RX. @multiple */
  #define EFM32_LEUART_CTRL_LOOPBK                   0x00000080
/** Clears RXBLOCK when the start-frame is found in the incoming data. The
   start-frame is loaded into the receive buffer. @multiple */
  #define EFM32_LEUART_CTRL_SFUBRX(v)                ((EFM32_LEUART_CTRL_SFUBRX_##v) << 8)
  #define EFM32_LEUART_CTRL_SFUBRX_SET(x, v)         do { (x) = (((x) & ~0x100) | ((EFM32_LEUART_CTRL_SFUBRX_##v) << 8)); } while(0)
  #define EFM32_LEUART_CTRL_SFUBRX_SETVAL(x, v)      do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define EFM32_LEUART_CTRL_SFUBRX_GET(x)            (((x) >> 8) & 0x1)
    #define EFM32_LEUART_CTRL_SFUBRX_NONE              0x00000000
    #define EFM32_LEUART_CTRL_SFUBRX_CLEAR             0x00000001
/** Set to enable multi-processor mode. @multiple */
  #define EFM32_LEUART_CTRL_MPM(v)                   ((EFM32_LEUART_CTRL_MPM_##v) << 9)
  #define EFM32_LEUART_CTRL_MPM_SET(x, v)            do { (x) = (((x) & ~0x200) | ((EFM32_LEUART_CTRL_MPM_##v) << 9)); } while(0)
  #define EFM32_LEUART_CTRL_MPM_SETVAL(x, v)         do { (x) = (((x) & ~0x200) | ((v) << 9)); } while(0)
  #define EFM32_LEUART_CTRL_MPM_GET(x)               (((x) >> 9) & 0x1)
    #define EFM32_LEUART_CTRL_MPM_NONE                 0x00000000
    #define EFM32_LEUART_CTRL_MPM_INTERRUPT            0x00000001
/** Defines the value of the multi-processor address bit. An incoming frame with
   its 9th bit equal to the value of this bit marks the frame as a
   multi-processor address frame. @multiple */
  #define EFM32_LEUART_CTRL_MPAB                     0x00000400
/** When 9-bit frames are transmitted, the default value of the 9th bit is given
   by BIT8DV. If TXDATA is used to write a frame, then the value of BIT8DV is
   assigned to the 9th bit of the outgoing frame. If a frame is written with
   TXDATAX however, the default value is overridden by the written value.
   @multiple */
  #define EFM32_LEUART_CTRL_BIT8DV                   0x00000800
/** Set to wake the DMA controller up when in EM2 and data is available in the
   receive buffer. @multiple */
  #define EFM32_LEUART_CTRL_RXDMAWU(v)               ((EFM32_LEUART_CTRL_RXDMAWU_##v) << 12)
  #define EFM32_LEUART_CTRL_RXDMAWU_SET(x, v)        do { (x) = (((x) & ~0x1000) | ((EFM32_LEUART_CTRL_RXDMAWU_##v) << 12)); } while(0)
  #define EFM32_LEUART_CTRL_RXDMAWU_SETVAL(x, v)     do { (x) = (((x) & ~0x1000) | ((v) << 12)); } while(0)
  #define EFM32_LEUART_CTRL_RXDMAWU_GET(x)           (((x) >> 12) & 0x1)
    #define EFM32_LEUART_CTRL_RXDMAWU_NONE             0x00000000
    #define EFM32_LEUART_CTRL_RXDMAWU_WAKE             0x00000001
/** Set to wake the DMA controller up when in EM2 and space is available in the
   transmit buffer. @multiple */
  #define EFM32_LEUART_CTRL_TXDMAWU(v)               ((EFM32_LEUART_CTRL_TXDMAWU_##v) << 13)
  #define EFM32_LEUART_CTRL_TXDMAWU_SET(x, v)        do { (x) = (((x) & ~0x2000) | ((EFM32_LEUART_CTRL_TXDMAWU_##v) << 13)); } while(0)
  #define EFM32_LEUART_CTRL_TXDMAWU_SETVAL(x, v)     do { (x) = (((x) & ~0x2000) | ((v) << 13)); } while(0)
  #define EFM32_LEUART_CTRL_TXDMAWU_GET(x)           (((x) >> 13) & 0x1)
    #define EFM32_LEUART_CTRL_TXDMAWU_NONE             0x00000000
    #define EFM32_LEUART_CTRL_TXDMAWU_WAKE             0x00000001
/** Configurable delay before new transfers. Frames sent back-to-back are not
   delayed. @multiple */
  #define EFM32_LEUART_CTRL_TXDELAY(v)               ((EFM32_LEUART_CTRL_TXDELAY_##v) << 14)
  #define EFM32_LEUART_CTRL_TXDELAY_SET(x, v)        do { (x) = (((x) & ~0xc000) | ((EFM32_LEUART_CTRL_TXDELAY_##v) << 14)); } while(0)
  #define EFM32_LEUART_CTRL_TXDELAY_SETVAL(x, v)     do { (x) = (((x) & ~0xc000) | ((v) << 14)); } while(0)
  #define EFM32_LEUART_CTRL_TXDELAY_GET(x)           (((x) >> 14) & 0x3)
    #define EFM32_LEUART_CTRL_TXDELAY_NONE             0x00000000
    #define EFM32_LEUART_CTRL_TXDELAY_SINGLE           0x00000001
    #define EFM32_LEUART_CTRL_TXDELAY_DOUBLE           0x00000002

#define EFM32_LEUART_CMD_ADDR                        0x00000004
#define EFM32_LEUART_CMD_MASK                        0x000000ff
/** Set to activate data reception on LEUn_RX @multiple */
  #define EFM32_LEUART_CMD_RXEN                      0x00000001
/** Set to disable data reception. If a frame is under reception when the
   receiver is disabled, the incoming frame is discarded. @multiple */
  #define EFM32_LEUART_CMD_RXDIS                     0x00000002
/** Set to enable data transmission. @multiple */
  #define EFM32_LEUART_CMD_TXEN                      0x00000004
/** Set to disable transmission. @multiple */
  #define EFM32_LEUART_CMD_TXDIS                     0x00000008
/** Set to set RXBLOCK, resulting in all incoming frames being discarded.
   @multiple */
  #define EFM32_LEUART_CMD_RXBLOCKEN                 0x00000010
/** Set to clear RXBLOCK, resulting in all incoming frames being loaded into the
   receive buffer. @multiple */
  #define EFM32_LEUART_CMD_RXBLOCKDIS                0x00000020
/** Set to clear receive buffer and the RX shift register. @multiple */
  #define EFM32_LEUART_CMD_CLEARTX                   0x00000040
/** Set to clear receive buffer and the RX shift register. @multiple */
  #define EFM32_LEUART_CMD_CLEARRX                   0x00000080

#define EFM32_LEUART_STATUS_ADDR                     0x00000008
#define EFM32_LEUART_STATUS_MASK                     0x0000003f
/** Set when the receiver is enabled. The receiver must be enabled for start
   frames, signal frames, and multi-processor address bit detection. @multiple */
  #define EFM32_LEUART_STATUS_RXENS                  0x00000001
/** Set when the transmitter is enabled. @multiple */
  #define EFM32_LEUART_STATUS_TXENS                  0x00000002
/** When set, the receiver discards incoming frames. An incoming frame will not
   be loaded into the receive buffer if this bit is set at the instant the frame
   has been completely received. @multiple */
  #define EFM32_LEUART_STATUS_RXBLOCK                0x00000004
/** Set when a transmission has completed and no more data is available in the
   transmit buffer. Cleared when a new transmission starts. @multiple */
  #define EFM32_LEUART_STATUS_TXC                    0x00000008
/** Indicates the level of the transmit buffer. Set when the transmit buffer is
   empty, and cleared when it is full. @multiple */
  #define EFM32_LEUART_STATUS_TXBL                   0x00000010
/** Set when data is available in the receive buffer. Cleared when the receive
   buffer is empty. @multiple */
  #define EFM32_LEUART_STATUS_RXDATAV                0x00000020

#define EFM32_LEUART_CLKDIV_ADDR                     0x0000000c
#define EFM32_LEUART_CLKDIV_MASK                     0x00007ff8
/** Specifies the fractional clock divider for the LEUART @multiple */
  #define EFM32_LEUART_CLKDIV_DIV(v)                 ((v) << 3)
  #define EFM32_LEUART_CLKDIV_DIV_SET(x, v)          do { (x) = (((x) & ~0x7ff8) | ((v) << 3)); } while(0)
  #define EFM32_LEUART_CLKDIV_DIV_GET(x)             (((x) >> 3) & 0xfff)

#define EFM32_LEUART_STARTFRAME_ADDR                 0x00000010
#define EFM32_LEUART_STARTFRAME_MASK                 0x000001ff
/** When a frame matching STARTFRAME is detected by the receiver, STARTF
   interrupt flag is set, and if SFUBRX is set, RXBLOCK is cleared. The
   start-frame is be loaded into the RX buffer. @multiple */
  #define EFM32_LEUART_STARTFRAME_STARTFRAME(v)      ((v) << 0)
  #define EFM32_LEUART_STARTFRAME_STARTFRAME_SET(x, v) do { (x) = (((x) & ~0x1ff) | ((v) << 0)); } while(0)
  #define EFM32_LEUART_STARTFRAME_STARTFRAME_GET(x)  (((x) >> 0) & 0x1ff)

#define EFM32_LEUART_SIGFRAME_ADDR                   0x00000014
#define EFM32_LEUART_SIGFRAME_MASK                   0x000001ff
/** When a frame matching SIGFRAME is detected by the receiver, SIGF interrupt
   flag is set. @multiple */
  #define EFM32_LEUART_SIGFRAME_SIGFRAME(v)          ((v) << 0)
  #define EFM32_LEUART_SIGFRAME_SIGFRAME_SET(x, v)   do { (x) = (((x) & ~0x1ff) | ((v) << 0)); } while(0)
  #define EFM32_LEUART_SIGFRAME_SIGFRAME_GET(x)      (((x) >> 0) & 0x1ff)

#define EFM32_LEUART_RXDATAX_ADDR                    0x00000018
#define EFM32_LEUART_RXDATAX_MASK                    0x0000c1ff
/** Use this register to access data read from the LEUART. Buffer is cleared on
   read access. @multiple */
  #define EFM32_LEUART_RXDATAX_RXDATA(v)             ((v) << 0)
  #define EFM32_LEUART_RXDATAX_RXDATA_SET(x, v)      do { (x) = (((x) & ~0x1ff) | ((v) << 0)); } while(0)
  #define EFM32_LEUART_RXDATAX_RXDATA_GET(x)         (((x) >> 0) & 0x1ff)
/** Set if data in buffer has a parity error. @multiple */
  #define EFM32_LEUART_RXDATAX_PERR                  0x00004000
/** Set if data in buffer has a framing error. Can be the result of a break
   condition. @multiple */
  #define EFM32_LEUART_RXDATAX_FERR                  0x00008000

#define EFM32_LEUART_RXDATA_ADDR                     0x0000001c
#define EFM32_LEUART_RXDATA_MASK                     0x000000ff
/** Use this register to access data read from LEUART. Buffer is cleared on read
   access. Only the 8 LSB can be read using this register. @multiple */
  #define EFM32_LEUART_RXDATA_RXDATA(v)              ((v) << 0)
  #define EFM32_LEUART_RXDATA_RXDATA_SET(x, v)       do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define EFM32_LEUART_RXDATA_RXDATA_GET(x)          (((x) >> 0) & 0xff)

#define EFM32_LEUART_RXDATAXP_ADDR                   0x00000020
#define EFM32_LEUART_RXDATAXP_MASK                   0x0000c1ff
/** Use this register to access data read from the LEUART. @multiple */
  #define EFM32_LEUART_RXDATAXP_RXDATAP(v)           ((v) << 0)
  #define EFM32_LEUART_RXDATAXP_RXDATAP_SET(x, v)    do { (x) = (((x) & ~0x1ff) | ((v) << 0)); } while(0)
  #define EFM32_LEUART_RXDATAXP_RXDATAP_GET(x)       (((x) >> 0) & 0x1ff)
/** Set if data in buffer has a parity error. @multiple */
  #define EFM32_LEUART_RXDATAXP_PERR                 0x00004000
/** Set if data in buffer has a framing error. Can be the result of a break
   condition. @multiple */
  #define EFM32_LEUART_RXDATAXP_FERR                 0x00008000

#define EFM32_LEUART_TXDATAX_ADDR                    0x00000024
#define EFM32_LEUART_TXDATAX_MASK                    0x0000e1ff
/** Use this register to write data to the LEUART. If the transmitter is enabled,
   a transfer will be initiated at the first opportunity. @multiple */
  #define EFM32_LEUART_TXDATAX_TXDATA(v)             ((v) << 0)
  #define EFM32_LEUART_TXDATAX_TXDATA_SET(x, v)      do { (x) = (((x) & ~0x1ff) | ((v) << 0)); } while(0)
  #define EFM32_LEUART_TXDATAX_TXDATA_GET(x)         (((x) >> 0) & 0x1ff)
/** Set to send data as a break. Recipient will see a framing error or a break
   condition depending on its configuration and the value of TXDATA. @multiple */
  #define EFM32_LEUART_TXDATAX_TXBREAK(v)            ((EFM32_LEUART_TXDATAX_TXBREAK_##v) << 13)
  #define EFM32_LEUART_TXDATAX_TXBREAK_SET(x, v)     do { (x) = (((x) & ~0x2000) | ((EFM32_LEUART_TXDATAX_TXBREAK_##v) << 13)); } while(0)
  #define EFM32_LEUART_TXDATAX_TXBREAK_SETVAL(x, v)  do { (x) = (((x) & ~0x2000) | ((v) << 13)); } while(0)
  #define EFM32_LEUART_TXDATAX_TXBREAK_GET(x)        (((x) >> 13) & 0x1)
    #define EFM32_LEUART_TXDATAX_TXBREAK_NONE          0x00000000
    #define EFM32_LEUART_TXDATAX_TXBREAK_BREAK         0x00000001
/** Set to disable transmitter directly after transmission has competed.
   @multiple */
  #define EFM32_LEUART_TXDATAX_TXDIASAT(v)           ((EFM32_LEUART_TXDATAX_TXDIASAT_##v) << 14)
  #define EFM32_LEUART_TXDATAX_TXDIASAT_SET(x, v)    do { (x) = (((x) & ~0x4000) | ((EFM32_LEUART_TXDATAX_TXDIASAT_##v) << 14)); } while(0)
  #define EFM32_LEUART_TXDATAX_TXDIASAT_SETVAL(x, v) do { (x) = (((x) & ~0x4000) | ((v) << 14)); } while(0)
  #define EFM32_LEUART_TXDATAX_TXDIASAT_GET(x)       (((x) >> 14) & 0x1)
    #define EFM32_LEUART_TXDATAX_TXDIASAT_NONE         0x00000000
    #define EFM32_LEUART_TXDATAX_TXDIASAT_DISABLED     0x00000001
/** Set to enable reception after transmission. @multiple */
  #define EFM32_LEUART_TXDATAX_RXENAT                0x00008000

#define EFM32_LEUART_TXDATA_ADDR                     0x00000028
#define EFM32_LEUART_TXDATA_MASK                     0x000000ff
/** This frame will be added to TX buffer. Only 8 LSB can be written using this
   register. 9th bit and control bits will be cleared. @multiple */
  #define EFM32_LEUART_TXDATA_TXDATA(v)              ((v) << 0)
  #define EFM32_LEUART_TXDATA_TXDATA_SET(x, v)       do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define EFM32_LEUART_TXDATA_TXDATA_GET(x)          (((x) >> 0) & 0xff)

#define EFM32_LEUART_IF_ADDR                         0x0000002c
#define EFM32_LEUART_IF_MASK                         0x000007ff
/** Set after a transmission when both the TX buffer and shift register are
   empty. @multiple */
  #define EFM32_LEUART_IF_TXC                        0x00000001
/** Set when space becomes available in the transmit buffer for a new frame.
   @multiple */
  #define EFM32_LEUART_IF_TXBL                       0x00000002
/** Set when data becomes available in the receive buffer. @multiple */
  #define EFM32_LEUART_IF_RXDATAV                    0x00000004
/** Set when data is incoming while the receive shift register is full. The data
   previously in shift register is overwritten by the new data. @multiple */
  #define EFM32_LEUART_IF_RXOF                       0x00000008
/** Set when trying to read from the receive buffer when it is empty. @multiple
   */
  #define EFM32_LEUART_IF_RXUF                       0x00000010
/** Set when a write is done to the transmit buffer while it is full. The data
   already in the transmit buffer is preserved. @multiple */
  #define EFM32_LEUART_IF_TXOF                       0x00000020
/** Set when a frame with a parity error is received while RXBLOCK is cleared.
   @multiple */
  #define EFM32_LEUART_IF_PERR                       0x00000040
/** Set when a frame with a framing error is received while RXBLOCK is cleared.
   @multiple */
  #define EFM32_LEUART_IF_FERR                       0x00000080
/** Set when a multi-processor address frame is detected. @multiple */
  #define EFM32_LEUART_IF_MPAF                       0x00000100
/** Set when a start frame is detected. @multiple */
  #define EFM32_LEUART_IF_STARTF                     0x00000200
/** Set when a signal frame is detected. @multiple */
  #define EFM32_LEUART_IF_SIGF                       0x00000400

#define EFM32_LEUART_IFS_ADDR                        0x00000030
#define EFM32_LEUART_IFS_MASK                        0x000007f9
/** Write to 1 to set the TXC interrupt flag. @multiple */
  #define EFM32_LEUART_IFS_TXC                       0x00000001
/** Write to 1 to set the RXOF interrupt flag. @multiple */
  #define EFM32_LEUART_IFS_RXOF                      0x00000008
/** Write to 1 to set the RXUF interrupt flag. @multiple */
  #define EFM32_LEUART_IFS_RXUF                      0x00000010
/** Write to 1 to set the TXOF interrupt flag. @multiple */
  #define EFM32_LEUART_IFS_TXOF                      0x00000020
/** Write to 1 to set the PERR interrupt flag. @multiple */
  #define EFM32_LEUART_IFS_PERR                      0x00000040
/** Write to 1 to set the FERR interrupt flag. @multiple */
  #define EFM32_LEUART_IFS_FERR                      0x00000080
/** Write to 1 to set the MPAF interrupt flag. @multiple */
  #define EFM32_LEUART_IFS_MPAF                      0x00000100
/** Write to 1 to set the STARTF interrupt flag. @multiple */
  #define EFM32_LEUART_IFS_STARTF                    0x00000200
/** Write to 1 to set the SIGF interrupt flag. @multiple */
  #define EFM32_LEUART_IFS_SIGF                      0x00000400

#define EFM32_LEUART_IFC_ADDR                        0x00000034
#define EFM32_LEUART_IFC_MASK                        0x000007f9
/** Write to 1 to clear the TXC interrupt flag. @multiple */
  #define EFM32_LEUART_IFC_TXC                       0x00000001
/** Write to 1 to clear the RXOF interrupt flag. @multiple */
  #define EFM32_LEUART_IFC_RXOF                      0x00000008
/** Write to 1 to clear the RXUF interrupt flag. @multiple */
  #define EFM32_LEUART_IFC_RXUF                      0x00000010
/** Write to 1 to clear the TXOF interrupt flag. @multiple */
  #define EFM32_LEUART_IFC_TXOF                      0x00000020
/** Write to 1 to clear the PERR interrupt flag. @multiple */
  #define EFM32_LEUART_IFC_PERR                      0x00000040
/** Write to 1 to clear the FERR interrupt flag. @multiple */
  #define EFM32_LEUART_IFC_FERR                      0x00000080
/** Write to 1 to clear the MPAF interrupt flag. @multiple */
  #define EFM32_LEUART_IFC_MPAF                      0x00000100
/** Write to 1 to clear the STARTF interrupt flag. @multiple */
  #define EFM32_LEUART_IFC_STARTF                    0x00000200
/** Write to 1 to clear the SIGF interrupt flag. @multiple */
  #define EFM32_LEUART_IFC_SIGF                      0x00000400

#define EFM32_LEUART_IEN_ADDR                        0x00000038
#define EFM32_LEUART_IEN_MASK                        0x000007ff
/** Enable interrupt on TX complete. @multiple */
  #define EFM32_LEUART_IEN_TXC                       0x00000001
/** Enable interrupt on TX buffer level. @multiple */
  #define EFM32_LEUART_IEN_TXBL                      0x00000002
/** Enable interrupt on RX data. @multiple */
  #define EFM32_LEUART_IEN_RXDATAV                   0x00000004
/** Enable interrupt on RX overflow. @multiple */
  #define EFM32_LEUART_IEN_RXOF                      0x00000008
/** Enable interrupt on RX underflow. @multiple */
  #define EFM32_LEUART_IEN_RXUF                      0x00000010
/** Enable interrupt on TX overflow. @multiple */
  #define EFM32_LEUART_IEN_TXOF                      0x00000020
/** Enable interrupt on parity error. @multiple */
  #define EFM32_LEUART_IEN_PERR                      0x00000040
/** Enable interrupt on framing error. @multiple */
  #define EFM32_LEUART_IEN_FERR                      0x00000080
/** Enable interrupt on multi-processor address frame. @multiple */
  #define EFM32_LEUART_IEN_MPAF                      0x00000100
/** Enable interrupt on start frame. @multiple */
  #define EFM32_LEUART_IEN_STARTF                    0x00000200
/** Enable interrupt on signal frame. @multiple */
  #define EFM32_LEUART_IEN_SIGF                      0x00000400

#define EFM32_LEUART_PULSECTRL_ADDR                  0x0000003c
#define EFM32_LEUART_PULSECTRL_MASK                  0x0000003f
/** Configure the pulse width of the pulse generator as a number of 32.768 kHz
   clock cycles. @multiple */
  #define EFM32_LEUART_PULSECTRL_PULSEW(v)           ((v) << 0)
  #define EFM32_LEUART_PULSECTRL_PULSEW_SET(x, v)    do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define EFM32_LEUART_PULSECTRL_PULSEW_GET(x)       (((x) >> 0) & 0xf)
/** Filter LEUART output through pulse generator and the LEUART input through the
   pulse extender. @multiple */
  #define EFM32_LEUART_PULSECTRL_PULSEEN             0x00000010
/** Enable a one-cycle pulse filter for pulse extender @multiple */
  #define EFM32_LEUART_PULSECTRL_PULSEFILT           0x00000020

#define EFM32_LEUART_FREEZE_ADDR                     0x00000040
#define EFM32_LEUART_FREEZE_MASK                     0x00000001
/** When set, the update of the LEUART is postponed until this bit is cleared.
   Use this bit to update several registers simultaneously. @multiple */
  #define EFM32_LEUART_FREEZE_REGFREEZE(v)           ((EFM32_LEUART_FREEZE_REGFREEZE_##v) << 0)
  #define EFM32_LEUART_FREEZE_REGFREEZE_SET(x, v)    do { (x) = (((x) & ~0x1) | ((EFM32_LEUART_FREEZE_REGFREEZE_##v) << 0)); } while(0)
  #define EFM32_LEUART_FREEZE_REGFREEZE_SETVAL(x, v) do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define EFM32_LEUART_FREEZE_REGFREEZE_GET(x)       (((x) >> 0) & 0x1)
    #define EFM32_LEUART_FREEZE_REGFREEZE_UPDATE       0x00000000
    #define EFM32_LEUART_FREEZE_REGFREEZE_FREEZE       0x00000001

#define EFM32_LEUART_SYNCBUSY_ADDR                   0x00000044
#define EFM32_LEUART_SYNCBUSY_MASK                   0x000000ff
/** Set when the value written to CTRL is being synchronized. @multiple */
  #define EFM32_LEUART_SYNCBUSY_CTRL                 0x00000001
/** Set when the value written to CMD is being synchronized. @multiple */
  #define EFM32_LEUART_SYNCBUSY_CMD                  0x00000002
/** Set when the value written to CLKDIV is being synchronized. @multiple */
  #define EFM32_LEUART_SYNCBUSY_CLKDIV               0x00000004
/** Set when the value written to STARTFRAME is being synchronized. @multiple */
  #define EFM32_LEUART_SYNCBUSY_STARTFRAME           0x00000008
/** Set when the value written to SIGFRAME is being synchronized. @multiple */
  #define EFM32_LEUART_SYNCBUSY_SIGFRAME             0x00000010
/** Set when the value written to TXDATAX is being synchronized. @multiple */
  #define EFM32_LEUART_SYNCBUSY_TXDATAX              0x00000020
/** Set when the value written to TXDATA is being synchronized. @multiple */
  #define EFM32_LEUART_SYNCBUSY_TXDATA               0x00000040
/** Set when the value written to PULSECTRL is being synchronized. @multiple */
  #define EFM32_LEUART_SYNCBUSY_PULSECTRL            0x00000080

#define EFM32_LEUART_ROUTE_ADDR                      0x00000054
#define EFM32_LEUART_ROUTE_MASK                      0x00000703
/** When set, the RX pin of the LEUART is enabled. @multiple */
  #define EFM32_LEUART_ROUTE_RXPEN                   0x00000001
/** When set, the TX pin of the LEUART is enabled. @multiple */
  #define EFM32_LEUART_ROUTE_TXPEN                   0x00000002
/** Decides the location of the LEUART I/O pins. @multiple */
  #define EFM32_LEUART_ROUTE_LOCATION(v)             ((EFM32_LEUART_ROUTE_LOCATION_##v) << 8)
  #define EFM32_LEUART_ROUTE_LOCATION_SET(x, v)      do { (x) = (((x) & ~0x700) | ((EFM32_LEUART_ROUTE_LOCATION_##v) << 8)); } while(0)
  #define EFM32_LEUART_ROUTE_LOCATION_SETVAL(x, v)   do { (x) = (((x) & ~0x700) | ((v) << 8)); } while(0)
  #define EFM32_LEUART_ROUTE_LOCATION_GET(x)         (((x) >> 8) & 0x7)
    #define EFM32_LEUART_ROUTE_LOCATION_LOC0           0x00000000
    #define EFM32_LEUART_ROUTE_LOCATION_LOC1           0x00000001
    #define EFM32_LEUART_ROUTE_LOCATION_LOC2           0x00000002
    #define EFM32_LEUART_ROUTE_LOCATION_LOC3           0x00000003
    #define EFM32_LEUART_ROUTE_LOCATION_LOC4           0x00000004

#define EFM32_LEUART_INPUT_ADDR                      0x000000ac
#define EFM32_LEUART_INPUT_MASK                      0x00000013
/** Select PRS channel as input to RX. @multiple */
  #define EFM32_LEUART_INPUT_RXPRSSEL(v)             ((EFM32_LEUART_INPUT_RXPRSSEL_##v) << 0)
  #define EFM32_LEUART_INPUT_RXPRSSEL_SET(x, v)      do { (x) = (((x) & ~0x3) | ((EFM32_LEUART_INPUT_RXPRSSEL_##v) << 0)); } while(0)
  #define EFM32_LEUART_INPUT_RXPRSSEL_SETVAL(x, v)   do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define EFM32_LEUART_INPUT_RXPRSSEL_GET(x)         (((x) >> 0) & 0x3)
    #define EFM32_LEUART_INPUT_RXPRSSEL_PRSCH0         0x00000000
    #define EFM32_LEUART_INPUT_RXPRSSEL_PRSCH1         0x00000001
    #define EFM32_LEUART_INPUT_RXPRSSEL_PRSCH2         0x00000002
    #define EFM32_LEUART_INPUT_RXPRSSEL_PRSCH3         0x00000003
/** When set, the PRS channel selected as input to RX. @multiple */
  #define EFM32_LEUART_INPUT_RXPRS                   0x00000010

#endif

