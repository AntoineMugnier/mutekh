#ifndef NXP_FLEXCOMM_H_
#define NXP_FLEXCOMM_H_

//
// Register and bit definitions for peripheral 'Flexcomm'
//

// FLEXCOMM.CFG Spi configuration register
#define FLEXCOMM_CFG_OFFSET 0x00000400
#define FLEXCOMM_CFG_SPOL1_MASK 0x200
#define FLEXCOMM_CFG_SPOL1_POS 9
#define FLEXCOMM_CFG_SPOL1_WIDTH 1
#define FLEXCOMM_CFG_SPOL0_MASK 0x100
#define FLEXCOMM_CFG_SPOL0_POS 8
#define FLEXCOMM_CFG_SPOL0_WIDTH 1
#define FLEXCOMM_CFG_LOOP_MASK 0x80
#define FLEXCOMM_CFG_LOOP_POS 7
#define FLEXCOMM_CFG_LOOP_WIDTH 1
#define FLEXCOMM_CFG_CPOL_MASK 0x20
#define FLEXCOMM_CFG_CPOL_POS 5
#define FLEXCOMM_CFG_CPOL_WIDTH 1
#define FLEXCOMM_CFG_CPHA_MASK 0x10
#define FLEXCOMM_CFG_CPHA_POS 4
#define FLEXCOMM_CFG_CPHA_WIDTH 1
#define FLEXCOMM_CFG_LSBF_MASK 0x8
#define FLEXCOMM_CFG_LSBF_POS 3
#define FLEXCOMM_CFG_LSBF_WIDTH 1
#define FLEXCOMM_CFG_MASTER_MASK 0x4
#define FLEXCOMM_CFG_MASTER_POS 2
#define FLEXCOMM_CFG_MASTER_WIDTH 1
#define FLEXCOMM_CFG_ENABLE_MASK 0x1
#define FLEXCOMM_CFG_ENABLE_POS 0
#define FLEXCOMM_CFG_ENABLE_WIDTH 1

// FLEXCOMM.DLY SPI delay register
#define FLEXCOMM_DLY_OFFSET 0x00000404
#define FLEXCOMM_DLY_TRANSFER_DELAY_MASK 0xf000
#define FLEXCOMM_DLY_TRANSFER_DELAY_POS 12
#define FLEXCOMM_DLY_TRANSFER_DELAY_WIDTH 4
#define FLEXCOMM_DLY_FRAME_DELAY_MASK 0xf00
#define FLEXCOMM_DLY_FRAME_DELAY_POS 8
#define FLEXCOMM_DLY_FRAME_DELAY_WIDTH 4
#define FLEXCOMM_DLY_POST_DELAY_MASK 0xf0
#define FLEXCOMM_DLY_POST_DELAY_POS 4
#define FLEXCOMM_DLY_POST_DELAY_WIDTH 4
#define FLEXCOMM_DLY_PRE_DELAY_MASK 0xf
#define FLEXCOMM_DLY_PRE_DELAY_POS 0
#define FLEXCOMM_DLY_PRE_DELAY_WIDTH 4

// FLEXCOMM.STAT SPI status register
#define FLEXCOMM_STAT_OFFSET 0x00000408
#define FLEXCOMM_STAT_MSTIDLE_MASK 0x100
#define FLEXCOMM_STAT_MSTIDLE_POS 8
#define FLEXCOMM_STAT_MSTIDLE_WIDTH 1
#define FLEXCOMM_STAT_END_TRANSFER_MASK 0x80
#define FLEXCOMM_STAT_END_TRANSFER_POS 7
#define FLEXCOMM_STAT_END_TRANSFER_WIDTH 1
#define FLEXCOMM_STAT_STALLED_MASK 0x40
#define FLEXCOMM_STAT_STALLED_POS 6
#define FLEXCOMM_STAT_STALLED_WIDTH 1
#define FLEXCOMM_STAT_SSD_MASK 0x20
#define FLEXCOMM_STAT_SSD_POS 5
#define FLEXCOMM_STAT_SSD_WIDTH 1
#define FLEXCOMM_STAT_SSA_MASK 0x10
#define FLEXCOMM_STAT_SSA_POS 4
#define FLEXCOMM_STAT_SSA_WIDTH 1
#define FLEXCOMM_STAT_TXUR_MASK 0x8
#define FLEXCOMM_STAT_TXUR_POS 3
#define FLEXCOMM_STAT_TXUR_WIDTH 1
#define FLEXCOMM_STAT_RXOV_MASK 0x4
#define FLEXCOMM_STAT_RXOV_POS 2
#define FLEXCOMM_STAT_RXOV_WIDTH 1

// FLEXCOMM.INTENSET SPI interrupt enable read and set
#define FLEXCOMM_INTENSET_OFFSET 0x0000040c
#define FLEXCOMM_INTENSET_MSTIDLEEN_MASK 0x100
#define FLEXCOMM_INTENSET_MSTIDLEEN_POS 8
#define FLEXCOMM_INTENSET_MSTIDLEEN_WIDTH 1
#define FLEXCOMM_INTENSET_SSDEN_MASK 0x20
#define FLEXCOMM_INTENSET_SSDEN_POS 5
#define FLEXCOMM_INTENSET_SSDEN_WIDTH 1
#define FLEXCOMM_INTENSET_SSAEN_MASK 0x10
#define FLEXCOMM_INTENSET_SSAEN_POS 4
#define FLEXCOMM_INTENSET_SSAEN_WIDTH 1
#define FLEXCOMM_INTENSET_TXUREN_MASK 0x8
#define FLEXCOMM_INTENSET_TXUREN_POS 3
#define FLEXCOMM_INTENSET_TXUREN_WIDTH 1
#define FLEXCOMM_INTENSET_RXOVEN_MASK 0x4
#define FLEXCOMM_INTENSET_RXOVEN_POS 2
#define FLEXCOMM_INTENSET_RXOVEN_WIDTH 1

// FLEXCOMM.INTENCLR SPI interrupt enable clear
#define FLEXCOMM_INTENCLR_OFFSET 0x00000410
#define FLEXCOMM_INTENCLR_MSTIDLE_MASK 0x100
#define FLEXCOMM_INTENCLR_MSTIDLE_POS 8
#define FLEXCOMM_INTENCLR_MSTIDLE_WIDTH 1
#define FLEXCOMM_INTENCLR_SSDEN_MASK 0x20
#define FLEXCOMM_INTENCLR_SSDEN_POS 5
#define FLEXCOMM_INTENCLR_SSDEN_WIDTH 1
#define FLEXCOMM_INTENCLR_SSAEN_MASK 0x10
#define FLEXCOMM_INTENCLR_SSAEN_POS 4
#define FLEXCOMM_INTENCLR_SSAEN_WIDTH 1
#define FLEXCOMM_INTENCLR_TXUREN_MASK 0x8
#define FLEXCOMM_INTENCLR_TXUREN_POS 3
#define FLEXCOMM_INTENCLR_TXUREN_WIDTH 1
#define FLEXCOMM_INTENCLR_RXOVEN_MASK 0x4
#define FLEXCOMM_INTENCLR_RXOVEN_POS 2
#define FLEXCOMM_INTENCLR_RXOVEN_WIDTH 1

// FLEXCOMM.RXDAT SPI received data
#define FLEXCOMM_RXDAT_OFFSET 0x00000414
#define FLEXCOMM_RXDAT_SOT_MASK 0x100000
#define FLEXCOMM_RXDAT_SOT_POS 20
#define FLEXCOMM_RXDAT_SOT_WIDTH 1
#define FLEXCOMM_RXDAT_RXSSELN_MASK 0x30000
#define FLEXCOMM_RXDAT_RXSSELN_POS 16
#define FLEXCOMM_RXDAT_RXSSELN_WIDTH 2
#define FLEXCOMM_RXDAT_RXDAT_MASK 0xffff
#define FLEXCOMM_RXDAT_RXDAT_POS 0
#define FLEXCOMM_RXDAT_RXDAT_WIDTH 16

// FLEXCOMM.TXDATCTL SPI transmit data with control
#define FLEXCOMM_TXDATCTL_OFFSET 0x00000418
#define FLEXCOMM_TXDATCTL_FLEN_MASK 0xf000000
#define FLEXCOMM_TXDATCTL_FLEN_POS 24
#define FLEXCOMM_TXDATCTL_FLEN_WIDTH 4
#define FLEXCOMM_TXDATCTL_RXIGNORE_MASK 0x400000
#define FLEXCOMM_TXDATCTL_RXIGNORE_POS 22
#define FLEXCOMM_TXDATCTL_RXIGNORE_WIDTH 1
#define FLEXCOMM_TXDATCTL_EOF_MASK 0x200000
#define FLEXCOMM_TXDATCTL_EOF_POS 21
#define FLEXCOMM_TXDATCTL_EOF_WIDTH 1
#define FLEXCOMM_TXDATCTL_EOT_MASK 0x100000
#define FLEXCOMM_TXDATCTL_EOT_POS 20
#define FLEXCOMM_TXDATCTL_EOT_WIDTH 1
#define FLEXCOMM_TXDATCTL_TXSSELN1_MASK 0x20000
#define FLEXCOMM_TXDATCTL_TXSSELN1_POS 17
#define FLEXCOMM_TXDATCTL_TXSSELN1_WIDTH 1
#define FLEXCOMM_TXDATCTL_TXSSELN0_MASK 0x10000
#define FLEXCOMM_TXDATCTL_TXSSELN0_POS 16
#define FLEXCOMM_TXDATCTL_TXSSELN0_WIDTH 1
#define FLEXCOMM_TXDATCTL_TXDATA_MASK 0xffff
#define FLEXCOMM_TXDATCTL_TXDATA_POS 0
#define FLEXCOMM_TXDATCTL_TXDATA_WIDTH 16

// FLEXCOMM.TXDAT SPI transmit data
#define FLEXCOMM_TXDAT_OFFSET 0x0000041c
#define FLEXCOMM_TXDAT_TXDAT_MASK 0xffff
#define FLEXCOMM_TXDAT_TXDAT_POS 0
#define FLEXCOMM_TXDAT_TXDAT_WIDTH 16

// FLEXCOMM.TXCTL SPI transmit control
#define FLEXCOMM_TXCTL_OFFSET 0x00000420
#define FLEXCOMM_TXCTL_LEN_MASK 0xf000000
#define FLEXCOMM_TXCTL_LEN_POS 24
#define FLEXCOMM_TXCTL_LEN_WIDTH 4
#define FLEXCOMM_TXCTL_RXIGNORE_MASK 0x400000
#define FLEXCOMM_TXCTL_RXIGNORE_POS 22
#define FLEXCOMM_TXCTL_RXIGNORE_WIDTH 1
#define FLEXCOMM_TXCTL_EOF_MASK 0x200000
#define FLEXCOMM_TXCTL_EOF_POS 21
#define FLEXCOMM_TXCTL_EOF_WIDTH 1
#define FLEXCOMM_TXCTL_EOT_MASK 0x100000
#define FLEXCOMM_TXCTL_EOT_POS 20
#define FLEXCOMM_TXCTL_EOT_WIDTH 1
#define FLEXCOMM_TXCTL_TXSSEL1_N_MASK 0x20000
#define FLEXCOMM_TXCTL_TXSSEL1_N_POS 17
#define FLEXCOMM_TXCTL_TXSSEL1_N_WIDTH 1
#define FLEXCOMM_TXCTL_TXSSEL0_N_MASK 0x10000
#define FLEXCOMM_TXCTL_TXSSEL0_N_POS 16
#define FLEXCOMM_TXCTL_TXSSEL0_N_WIDTH 1

// FLEXCOMM.DIV SPI clock divider
#define FLEXCOMM_DIV_OFFSET 0x00000424
#define FLEXCOMM_DIV_DIVVAL_MASK 0xffff
#define FLEXCOMM_DIV_DIVVAL_POS 0
#define FLEXCOMM_DIV_DIVVAL_WIDTH 16

// FLEXCOMM.INTSTAT SPI interrupt status enable
#define FLEXCOMM_INTSTAT_OFFSET 0x00000428
#define FLEXCOMM_INTSTAT_MSTIDLE_MASK 0x100
#define FLEXCOMM_INTSTAT_MSTIDLE_POS 8
#define FLEXCOMM_INTSTAT_MSTIDLE_WIDTH 1
#define FLEXCOMM_INTSTAT_SSD_MASK 0x20
#define FLEXCOMM_INTSTAT_SSD_POS 5
#define FLEXCOMM_INTSTAT_SSD_WIDTH 1
#define FLEXCOMM_INTSTAT_SSA_MASK 0x10
#define FLEXCOMM_INTSTAT_SSA_POS 4
#define FLEXCOMM_INTSTAT_SSA_WIDTH 1
#define FLEXCOMM_INTSTAT_TXUR_MASK 0x8
#define FLEXCOMM_INTSTAT_TXUR_POS 3
#define FLEXCOMM_INTSTAT_TXUR_WIDTH 1
#define FLEXCOMM_INTSTAT_RXOV_MASK 0x4
#define FLEXCOMM_INTSTAT_RXOV_POS 2
#define FLEXCOMM_INTSTAT_RXOV_WIDTH 1

// FLEXCOMM.FIFOCFG FIFO configuration and enable register
#define FLEXCOMM_FIFOCFG_OFFSET 0x00000e00
#define FLEXCOMM_FIFOCFG_POPDBG_MASK 0x40000
#define FLEXCOMM_FIFOCFG_POPDBG_POS 18
#define FLEXCOMM_FIFOCFG_POPDBG_WIDTH 1
#define FLEXCOMM_FIFOCFG_EMPTYRX_MASK 0x20000
#define FLEXCOMM_FIFOCFG_EMPTYRX_POS 17
#define FLEXCOMM_FIFOCFG_EMPTYRX_WIDTH 1
#define FLEXCOMM_FIFOCFG_EMPTYTX_MASK 0x10000
#define FLEXCOMM_FIFOCFG_EMPTYTX_POS 16
#define FLEXCOMM_FIFOCFG_EMPTYTX_WIDTH 1
#define FLEXCOMM_FIFOCFG_WAKERX_MASK 0x8000
#define FLEXCOMM_FIFOCFG_WAKERX_POS 15
#define FLEXCOMM_FIFOCFG_WAKERX_WIDTH 1
#define FLEXCOMM_FIFOCFG_WAKETX_MASK 0x4000
#define FLEXCOMM_FIFOCFG_WAKETX_POS 14
#define FLEXCOMM_FIFOCFG_WAKETX_WIDTH 1
#define FLEXCOMM_FIFOCFG_DMARX_MASK 0x2000
#define FLEXCOMM_FIFOCFG_DMARX_POS 13
#define FLEXCOMM_FIFOCFG_DMARX_WIDTH 1
#define FLEXCOMM_FIFOCFG_DMATX_MASK 0x1000
#define FLEXCOMM_FIFOCFG_DMATX_POS 12
#define FLEXCOMM_FIFOCFG_DMATX_WIDTH 1
#define FLEXCOMM_FIFOCFG_SIZE_MASK 0x30
#define FLEXCOMM_FIFOCFG_SIZE_POS 4
#define FLEXCOMM_FIFOCFG_SIZE_WIDTH 2
#define FLEXCOMM_FIFOCFG_ENABLERX_MASK 0x2
#define FLEXCOMM_FIFOCFG_ENABLERX_POS 1
#define FLEXCOMM_FIFOCFG_ENABLERX_WIDTH 1
#define FLEXCOMM_FIFOCFG_ENABLETX_MASK 0x1
#define FLEXCOMM_FIFOCFG_ENABLETX_POS 0
#define FLEXCOMM_FIFOCFG_ENABLETX_WIDTH 1

// FLEXCOMM.FIFOSTAT FIFO configuration and enable register
#define FLEXCOMM_FIFOSTAT_OFFSET 0x00000e04
#define FLEXCOMM_FIFOSTAT_RXLVL_MASK 0x1f0000
#define FLEXCOMM_FIFOSTAT_RXLVL_POS 16
#define FLEXCOMM_FIFOSTAT_RXLVL_WIDTH 5
#define FLEXCOMM_FIFOSTAT_TXLVL_MASK 0x1f00
#define FLEXCOMM_FIFOSTAT_TXLVL_POS 8
#define FLEXCOMM_FIFOSTAT_TXLVL_WIDTH 5
#define FLEXCOMM_FIFOSTAT_RXFULL_MASK 0x80
#define FLEXCOMM_FIFOSTAT_RXFULL_POS 7
#define FLEXCOMM_FIFOSTAT_RXFULL_WIDTH 1
#define FLEXCOMM_FIFOSTAT_RXNOTEMPTY_MASK 0x40
#define FLEXCOMM_FIFOSTAT_RXNOTEMPTY_POS 6
#define FLEXCOMM_FIFOSTAT_RXNOTEMPTY_WIDTH 1
#define FLEXCOMM_FIFOSTAT_TXNOTFULL_MASK 0x20
#define FLEXCOMM_FIFOSTAT_TXNOTFULL_POS 5
#define FLEXCOMM_FIFOSTAT_TXNOTFULL_WIDTH 1
#define FLEXCOMM_FIFOSTAT_TXEMPTY_MASK 0x10
#define FLEXCOMM_FIFOSTAT_TXEMPTY_POS 4
#define FLEXCOMM_FIFOSTAT_TXEMPTY_WIDTH 1
#define FLEXCOMM_FIFOSTAT_PERINT_MASK 0x8
#define FLEXCOMM_FIFOSTAT_PERINT_POS 3
#define FLEXCOMM_FIFOSTAT_PERINT_WIDTH 1
#define FLEXCOMM_FIFOSTAT_RXERR_MASK 0x2
#define FLEXCOMM_FIFOSTAT_RXERR_POS 1
#define FLEXCOMM_FIFOSTAT_RXERR_WIDTH 1
#define FLEXCOMM_FIFOSTAT_TXERR_MASK 0x1
#define FLEXCOMM_FIFOSTAT_TXERR_POS 0
#define FLEXCOMM_FIFOSTAT_TXERR_WIDTH 1

// FLEXCOMM.FIFOTRIG FIFO trigger settings for interrupt and DMA settings
#define FLEXCOMM_FIFOTRIG_OFFSET 0x00000e08
#define FLEXCOMM_FIFOTRIG_RXLVL_MASK 0xf0000
#define FLEXCOMM_FIFOTRIG_RXLVL_POS 16
#define FLEXCOMM_FIFOTRIG_RXLVL_WIDTH 4
#define FLEXCOMM_FIFOTRIG_TXLVL_MASK 0xf00
#define FLEXCOMM_FIFOTRIG_TXLVL_POS 8
#define FLEXCOMM_FIFOTRIG_TXLVL_WIDTH 4
#define FLEXCOMM_FIFOTRIG_RXLVLENA_MASK 0x2
#define FLEXCOMM_FIFOTRIG_RXLVLENA_POS 1
#define FLEXCOMM_FIFOTRIG_RXLVLENA_WIDTH 1
#define FLEXCOMM_FIFOTRIG_TXLVLENA_MASK 0x1
#define FLEXCOMM_FIFOTRIG_TXLVLENA_POS 0
#define FLEXCOMM_FIFOTRIG_TXLVLENA_WIDTH 1

// FLEXCOMM.FIFOINTENSET FIFO interrupt enable set and read register
#define FLEXCOMM_FIFOINTENSET_OFFSET 0x00000e10
#define FLEXCOMM_FIFOINTENSET_RXLVL_MASK 0x8
#define FLEXCOMM_FIFOINTENSET_RXLVL_POS 3
#define FLEXCOMM_FIFOINTENSET_RXLVL_WIDTH 1
#define FLEXCOMM_FIFOINTENSET_TXLVL_MASK 0x4
#define FLEXCOMM_FIFOINTENSET_TXLVL_POS 2
#define FLEXCOMM_FIFOINTENSET_TXLVL_WIDTH 1
#define FLEXCOMM_FIFOINTENSET_RXERR_MASK 0x2
#define FLEXCOMM_FIFOINTENSET_RXERR_POS 1
#define FLEXCOMM_FIFOINTENSET_RXERR_WIDTH 1
#define FLEXCOMM_FIFOINTENSET_TXERR_MASK 0x1
#define FLEXCOMM_FIFOINTENSET_TXERR_POS 0
#define FLEXCOMM_FIFOINTENSET_TXERR_WIDTH 1

// FLEXCOMM.FIFOINTENCLR FIFO interrupt enable clear and read register
#define FLEXCOMM_FIFOINTENCLR_OFFSET 0x00000e14
#define FLEXCOMM_FIFOINTENCLR_RXLVL_MASK 0x8
#define FLEXCOMM_FIFOINTENCLR_RXLVL_POS 3
#define FLEXCOMM_FIFOINTENCLR_RXLVL_WIDTH 1
#define FLEXCOMM_FIFOINTENCLR_TXLVL_MASK 0x4
#define FLEXCOMM_FIFOINTENCLR_TXLVL_POS 2
#define FLEXCOMM_FIFOINTENCLR_TXLVL_WIDTH 1
#define FLEXCOMM_FIFOINTENCLR_RXERR_MASK 0x2
#define FLEXCOMM_FIFOINTENCLR_RXERR_POS 1
#define FLEXCOMM_FIFOINTENCLR_RXERR_WIDTH 1
#define FLEXCOMM_FIFOINTENCLR_TXERR_MASK 0x1
#define FLEXCOMM_FIFOINTENCLR_TXERR_POS 0
#define FLEXCOMM_FIFOINTENCLR_TXERR_WIDTH 1

// FLEXCOMM.FIFOINTSTAT FIFO interrupt status register
#define FLEXCOMM_FIFOINTSTAT_OFFSET 0x00000e18
#define FLEXCOMM_FIFOINTSTAT_PERINT_MASK 0x10
#define FLEXCOMM_FIFOINTSTAT_PERINT_POS 4
#define FLEXCOMM_FIFOINTSTAT_PERINT_WIDTH 1
#define FLEXCOMM_FIFOINTSTAT_RXLVL_MASK 0x8
#define FLEXCOMM_FIFOINTSTAT_RXLVL_POS 3
#define FLEXCOMM_FIFOINTSTAT_RXLVL_WIDTH 1
#define FLEXCOMM_FIFOINTSTAT_TXLVL_MASK 0x4
#define FLEXCOMM_FIFOINTSTAT_TXLVL_POS 2
#define FLEXCOMM_FIFOINTSTAT_TXLVL_WIDTH 1
#define FLEXCOMM_FIFOINTSTAT_RXERR_MASK 0x2
#define FLEXCOMM_FIFOINTSTAT_RXERR_POS 1
#define FLEXCOMM_FIFOINTSTAT_RXERR_WIDTH 1
#define FLEXCOMM_FIFOINTSTAT_TXERR_MASK 0x1
#define FLEXCOMM_FIFOINTSTAT_TXERR_POS 0
#define FLEXCOMM_FIFOINTSTAT_TXERR_WIDTH 1

// FLEXCOMM.FIFOWR FIFO write data
#define FLEXCOMM_FIFOWR_OFFSET 0x00000e20
#define FLEXCOMM_FIFOWR_LSPI_3WIRE_MASK 0x10000000
#define FLEXCOMM_FIFOWR_LSPI_3WIRE_POS 28
#define FLEXCOMM_FIFOWR_LSPI_3WIRE_WIDTH 1
#define FLEXCOMM_FIFOWR_LEN_MASK 0xf000000
#define FLEXCOMM_FIFOWR_LEN_POS 24
#define FLEXCOMM_FIFOWR_LEN_WIDTH 4
#define FLEXCOMM_FIFOWR_TXIGNORE_MASK 0x800000
#define FLEXCOMM_FIFOWR_TXIGNORE_POS 23
#define FLEXCOMM_FIFOWR_TXIGNORE_WIDTH 1
#define FLEXCOMM_FIFOWR_RXIGNORE_MASK 0x400000
#define FLEXCOMM_FIFOWR_RXIGNORE_POS 22
#define FLEXCOMM_FIFOWR_RXIGNORE_WIDTH 1
#define FLEXCOMM_FIFOWR_EOF_MASK 0x200000
#define FLEXCOMM_FIFOWR_EOF_POS 21
#define FLEXCOMM_FIFOWR_EOF_WIDTH 1
#define FLEXCOMM_FIFOWR_EOT_MASK 0x100000
#define FLEXCOMM_FIFOWR_EOT_POS 20
#define FLEXCOMM_FIFOWR_EOT_WIDTH 1
#define FLEXCOMM_FIFOWR_TXSSEL1_N_MASK 0x20000
#define FLEXCOMM_FIFOWR_TXSSEL1_N_POS 17
#define FLEXCOMM_FIFOWR_TXSSEL1_N_WIDTH 1
#define FLEXCOMM_FIFOWR_TXSSEL0_N_MASK 0x10000
#define FLEXCOMM_FIFOWR_TXSSEL0_N_POS 16
#define FLEXCOMM_FIFOWR_TXSSEL0_N_WIDTH 1
#define FLEXCOMM_FIFOWR_TXDATA_MASK 0xffff
#define FLEXCOMM_FIFOWR_TXDATA_POS 0
#define FLEXCOMM_FIFOWR_TXDATA_WIDTH 16

// FLEXCOMM.FIFORD FIFO read data
#define FLEXCOMM_FIFORD_OFFSET 0x00000e30
#define FLEXCOMM_FIFORD_SOT_MASK 0x100000
#define FLEXCOMM_FIFORD_SOT_POS 20
#define FLEXCOMM_FIFORD_SOT_WIDTH 1
#define FLEXCOMM_FIFORD_RXSSEL1_N_MASK 0x20000
#define FLEXCOMM_FIFORD_RXSSEL1_N_POS 17
#define FLEXCOMM_FIFORD_RXSSEL1_N_WIDTH 1
#define FLEXCOMM_FIFORD_RXSSEL0_N_MASK 0x10000
#define FLEXCOMM_FIFORD_RXSSEL0_N_POS 16
#define FLEXCOMM_FIFORD_RXSSEL0_N_WIDTH 1
#define FLEXCOMM_FIFORD_RXDATA_MASK 0xffff
#define FLEXCOMM_FIFORD_RXDATA_POS 0
#define FLEXCOMM_FIFORD_RXDATA_WIDTH 16

// FLEXCOMM.FIFORDNOPOP FIFO read data without popping FIFO
#define FLEXCOMM_FIFORDNOPOP_OFFSET 0x00000e40
#define FLEXCOMM_FIFORDNOPOP_SOT_MASK 0x100000
#define FLEXCOMM_FIFORDNOPOP_SOT_POS 20
#define FLEXCOMM_FIFORDNOPOP_SOT_WIDTH 1
#define FLEXCOMM_FIFORDNOPOP_RXSSEL1_N_MASK 0x20000
#define FLEXCOMM_FIFORDNOPOP_RXSSEL1_N_POS 17
#define FLEXCOMM_FIFORDNOPOP_RXSSEL1_N_WIDTH 1
#define FLEXCOMM_FIFORDNOPOP_RXSSEL0_N_MASK 0x10000
#define FLEXCOMM_FIFORDNOPOP_RXSSEL0_N_POS 16
#define FLEXCOMM_FIFORDNOPOP_RXSSEL0_N_WIDTH 1
#define FLEXCOMM_FIFORDNOPOP_RXDATA_MASK 0xffff
#define FLEXCOMM_FIFORDNOPOP_RXDATA_POS 0
#define FLEXCOMM_FIFORDNOPOP_RXDATA_WIDTH 16

#define FLEXCOMM_FIFOSIZE_OFFSET 0x00000e48
#define FLEXCOMM_FIFOSIZE_SIZE_MASK 0x1f
#define FLEXCOMM_FIFOSIZE_SIZE_POS 0
#define FLEXCOMM_FIFOSIZE_SIZE_WIDTH 5

// FLEXCOMM.PSELID Peripheral Select and Flexcomm ID register.
#define FLEXCOMM_PSELID_OFFSET 0x00000ff8
#define FLEXCOMM_PSELID_ID_MASK 0xfffff000
#define FLEXCOMM_PSELID_ID_POS 12
#define FLEXCOMM_PSELID_ID_WIDTH 20
#define FLEXCOMM_PSELID_I2SPRESENT_MASK 0x80
#define FLEXCOMM_PSELID_I2SPRESENT_POS 7
#define FLEXCOMM_PSELID_I2SPRESENT_WIDTH 1
#define FLEXCOMM_PSELID_I2CPRESENT_MASK 0x40
#define FLEXCOMM_PSELID_I2CPRESENT_POS 6
#define FLEXCOMM_PSELID_I2CPRESENT_WIDTH 1
#define FLEXCOMM_PSELID_SPIPRESENT_MASK 0x20
#define FLEXCOMM_PSELID_SPIPRESENT_POS 5
#define FLEXCOMM_PSELID_SPIPRESENT_WIDTH 1
#define FLEXCOMM_PSELID_USARTPRESENT_MASK 0x10
#define FLEXCOMM_PSELID_USARTPRESENT_POS 4
#define FLEXCOMM_PSELID_USARTPRESENT_WIDTH 1
#define FLEXCOMM_PSELID_LOCK_MASK 0x8
#define FLEXCOMM_PSELID_LOCK_POS 3
#define FLEXCOMM_PSELID_LOCK_WIDTH 1
#define FLEXCOMM_PSELID_PERSEL_MASK 0x7
#define FLEXCOMM_PSELID_PERSEL_POS 0
#define FLEXCOMM_PSELID_PERSEL_WIDTH 3

#define FLEXCOMM_PSELID_PERSEL_SPI 2

// FLEXCOMM.ID FIFO data read with no pop
#define FLEXCOMM_ID_OFFSET 0x00000ffc
#define FLEXCOMM_ID_ID_MASK 0xffff0000
#define FLEXCOMM_ID_ID_POS 16
#define FLEXCOMM_ID_ID_WIDTH 16
#define FLEXCOMM_ID_MAJOR_REV_MASK 0xf000
#define FLEXCOMM_ID_MAJOR_REV_POS 12
#define FLEXCOMM_ID_MAJOR_REV_WIDTH 4
#define FLEXCOMM_ID_MINOR_REV_MASK 0xf00
#define FLEXCOMM_ID_MINOR_REV_POS 8
#define FLEXCOMM_ID_MINOR_REV_WIDTH 4
#define FLEXCOMM_ID_APERTURE_MASK 0xff
#define FLEXCOMM_ID_APERTURE_POS 0
#define FLEXCOMM_ID_APERTURE_WIDTH 8

#define flexcomm_set32(inst, name, value)                             \
    cpu_mem_write_32((uint32_t)(inst) + FLEXCOMM_##name##_OFFSET, (value))

#define flexcomm_get32(inst, name)                                   \
    cpu_mem_read_32((uint32_t)(inst) + FLEXCOMM_##name##_OFFSET)

#define flexcomm_set8(inst, name, value)                             \
    cpu_mem_write_8((uint32_t)(inst) + FLEXCOMM_##name##_OFFSET, (value))

#define flexcomm_get8(inst, name)                                   \
    cpu_mem_read_8((uint32_t)(inst) + FLEXCOMM_##name##_OFFSET)

#define flexcomm_get(v32, reg, field)                                \
    ((v32 & FLEXCOMM_##reg##_##field##_MASK)                         \
     >> FLEXCOMM_##reg##_##field##_POS)

#define flexcomm_ins_val(v32, reg, field, value)                       \
    ((v32 & ~FLEXCOMM_##reg##_##field##_MASK) |                        \
     (FLEXCOMM_##reg##_##field##_MASK & ((value) << FLEXCOMM_##reg##_##field##_POS)))

#define flexcomm_ins_enum(v32, reg, field, value)                      \
    flexcomm_ins_val(v32, reg, field, FLEXCOMM_##reg##_##field##_##value)

#define flexcomm_bit(reg, field)                           \
    (FLEXCOMM_##reg##_##field##_MASK)

#endif