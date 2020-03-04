/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.
*/

#ifndef S2LP_REGS_H_
#define S2LP_REGS_H_

// SPI header byte
#define S2LP_WRITE_REG_BYTE  ((uint8_t)0x00)
#define S2LP_READ_REG_BYTE   ((uint8_t)0x01)
#define S2LP_CMD_HEADER_BYTE ((uint8_t)0x80)

// GPIO config registers
#define S2LP_GPIO0_CONF_ADDR     ((uint8_t)0x00)
#define S2LP_GPIO1_CONF_ADDR     ((uint8_t)0x01)
#define S2LP_GPIO2_CONF_ADDR     ((uint8_t)0x02)
#define S2LP_GPIO3_CONF_ADDR     ((uint8_t)0x03)

// GPIO registers masks
#define S2LP_GPIO_SELECT_REGMASK ((uint8_t)0xF8)
#define S2LP_GPIO_MODE_REGMASK   ((uint8_t)0x03)

// IRQ registers
#define S2LP_IRQ_MASK3_ADDR      ((uint8_t)0x50)
#define S2LP_IRQ_MASK2_ADDR      ((uint8_t)0x51)
#define S2LP_IRQ_MASK1_ADDR      ((uint8_t)0x52)
#define S2LP_IRQ_MASK0_ADDR      ((uint8_t)0x53)
#define S2LP_IRQ_STATUS3_ADDR    ((uint8_t)0xFA)
#define S2LP_IRQ_STATUS2_ADDR    ((uint8_t)0xFB)
#define S2LP_IRQ_STATUS1_ADDR    ((uint8_t)0xFC)
#define S2LP_IRQ_STATUS0_ADDR    ((uint8_t)0xFD)

// Packet registers
#define S2LP_PCKTCTRL6_ADDR      ((uint8_t)0x2B)
#define S2LP_PCKTCTRL5_ADDR      ((uint8_t)0x2C)
#define S2LP_PCKTCTRL4_ADDR      ((uint8_t)0x2D)
#define S2LP_PCKTCTRL3_ADDR      ((uint8_t)0x2E)
#define S2LP_PCKTCTRL2_ADDR      ((uint8_t)0x2F)
#define S2LP_PCKTCTRL1_ADDR      ((uint8_t)0x30)
#define S2LP_PCKTLEN1_ADDR       ((uint8_t)0x31)
#define S2LP_PCKTLEN0_ADDR       ((uint8_t)0x32)
#define S2LP_SYNC3_ADDR          ((uint8_t)0x33)
#define S2LP_SYNC2_ADDR          ((uint8_t)0x34)
#define S2LP_SYNC1_ADDR          ((uint8_t)0x35)
#define S2LP_SYNC0_ADDR          ((uint8_t)0x36)
#define S2LP_PROTOCOL2_ADDR      ((uint8_t)0x39)
#define S2LP_PROTOCOL1_ADDR      ((uint8_t)0x3A)
#define S2LP_PROTOCOL0_ADDR      ((uint8_t)0x3B)

// Packet registers masks/offsets
#define S2LP_PCKTCTRL6_SYNC_LEN_REGMASK      ((uint8_t)0xFC)
#define S2LP_PCKTCTRL6_PREAMBLE_LEN_REGMASK  ((uint8_t)0x03)
#define S2LP_PCKTCTRL6_SYNC_LEN_OFFSET(x)    (uint8_t)((x) << 2)

#define S2LP_PCKTCTRL4_LEN_WID_REGMASK       ((uint8_t)0x80)
#define S2LP_PCKTCTRL4_ADDRESS_LEN_REGMASK   ((uint8_t)0x08)

#define S2LP_PCKTCTRL3_PCKT_FRMT_REGMASK     ((uint8_t)0xC0)
#define S2LP_PCKTCTRL3_RX_MODE_REGMASK       ((uint8_t)0x30)
#define S2LP_PCKTCTRL3_FSK4_SYM_SWAP_REGMASK ((uint8_t)0x08)
#define S2LP_PCKTCTRL3_BYTE_SWAP_REGMASK     ((uint8_t)0x04)
#define S2LP_PCKTCTRL3_PREAMBLE_SEL_REGMASK  ((uint8_t)0x03)

#define S2LP_PCKTCTRL2_FCS_TYPE_4G_REGMASK   ((uint8_t)0x20)
#define S2LP_PCKTCTRL2_FEC_TYPE_4G_REGMASK   ((uint8_t)0x10)
#define S2LP_PCKTCTRL2_INT_EN_4G_REGMASK     ((uint8_t)0x08)
#define S2LP_PCKTCTRL2_MBUS_3OF6_EN_REGMASK  ((uint8_t)0x04)
#define S2LP_PCKTCTRL2_MANCHESTER_EN_REGMASK ((uint8_t)0x02)
#define S2LP_PCKTCTRL2_FIX_VAR_LEN_REGMASK   ((uint8_t)0x01)

#define S2LP_PCKTCTRL1_CRC_MODE_REGMASK        ((uint8_t)0xE0)
#define S2LP_PCKTCTRL1_WHIT_EN_REGMASK         ((uint8_t)0x10)
#define S2LP_PCKTCTRL1_TXSOURCE_REGMASK        ((uint8_t)0x0C)
#define S2LP_PCKTCTRL1_SECOND_SYNC_SEL_REGMASK ((uint8_t)0x02)
#define S2LP_PCKTCTRL1_FEC_EN_REGMASK          ((uint8_t)0x01)

#define S2LP_PROTOCOL2_CS_TIMEOUT_MASK_REGMASK       ((uint8_t)0x80)
#define S2LP_PROTOCOL2_SQI_TIMEOUT_MASK_REGMASK      ((uint8_t)0x40)
#define S2LP_PROTOCOL2_PQI_TIMEOUT_MASK_REGMASK      ((uint8_t)0x20)
#define S2LP_PROTOCOL2_TX_SEQ_NUM_RELOAD_REGMASK     ((uint8_t)0x18)
#define S2LP_PROTOCOL2_FIFO_GPIO_OUT_MUX_SEL_REGMASK ((uint8_t)0x04)
#define S2LP_PROTOCOL2_LDC_TIMER_MULT_REGMASK        ((uint8_t)0x03)

#define S2LP_PROTOCOL1_LDC_MODE_REGMASK              ((uint8_t)0x80)
#define S2LP_PROTOCOL1_LDC_RELOAD_ON_SYNC_REGMASK    ((uint8_t)0x40)
#define S2LP_PROTOCOL1_PIGGYBACKING_REGMASK          ((uint8_t)0x20)
#define S2LP_PROTOCOL1_FAST_CS_TERM_EN_REGMASK       ((uint8_t)0x10)
#define S2LP_PROTOCOL1_SEED_RELOAD_REGMASK           ((uint8_t)0x08)
#define S2LP_PROTOCOL1_CSMA_ON_REGMASK               ((uint8_t)0x04)
#define S2LP_PROTOCOL1_CSMA_PERS_ON_REGMASK          ((uint8_t)0x02)
#define S2LP_PROTOCOL1_AUTO_PCKT_FLT_REGMASK         ((uint8_t)0x01)

#define S2LP_PROTOCOL0_NMAX_RETX_REGMASK             ((uint8_t)0xF0)
#define S2LP_PROTOCOL0_NACK_TX_REGMASK               ((uint8_t)0x08)
#define S2LP_PROTOCOL0_AUTO_ACK_REGMASK              ((uint8_t)0x04)
#define S2LP_PROTOCOL0_PERS_RX_REGMASK               ((uint8_t)0x02)

// FIFO registers
#define S2LP_FIFO_CONFIG3_ADDR     ((uint8_t)0x3C)
#define S2LP_FIFO_CONFIG2_ADDR     ((uint8_t)0x3D)
#define S2LP_FIFO_CONFIG1_ADDR     ((uint8_t)0x3E)
#define S2LP_FIFO_CONFIG0_ADDR     ((uint8_t)0x3F)

// Rf registers
#define S2LP_SYNT3_ADDR         ((uint8_t)0x05)
#define S2LP_SYNT2_ADDR         ((uint8_t)0x06)
#define S2LP_SYNT1_ADDR         ((uint8_t)0x07)
#define S2LP_SYNT0_ADDR         ((uint8_t)0x08)
#define S2LP_IF_OFFSET_ANA_ADDR ((uint8_t)0x09)
#define S2LP_IF_OFFSET_DIG_ADDR ((uint8_t)0x0A)
#define S2LP_CH_SPACE_ADDR      ((uint8_t)0x0C)
#define S2LP_CHNUM_ADDR         ((uint8_t)0x0D)
#define S2LP_MOD4_ADDR          ((uint8_t)0x0E)
#define S2LP_MOD3_ADDR          ((uint8_t)0x0F)
#define S2LP_MOD2_ADDR          ((uint8_t)0x10)
#define S2LP_MOD1_ADDR          ((uint8_t)0x11)
#define S2LP_MOD0_ADDR          ((uint8_t)0x12)
#define S2LP_CHFLT_ADDR         ((uint8_t)0x13)
#define S2LP_RSSI_TH_ADDR       ((uint8_t)0x18)
#define S2LP_SYNTH_CONFIG2_ADDR ((uint8_t)0x65)

// Rf registers masks
#define S2LP_MOD2_MOD_TYPE_REGMASK            ((uint8_t)0xF0)
#define S2LP_MOD2_DATARATE_E_REGMASK          ((uint8_t)0x0F)

#define S2LP_MOD1_PA_INTERP_EN_REGMASK        ((uint8_t)0x80)
#define S2LP_MOD1_MOD_INTERP_EN_REGMASK       ((uint8_t)0x40)
#define S2LP_MOD1_G4FSK_CONST_MAP_REGMASK     ((uint8_t)0x30)
#define S2LP_MOD1_FDEV_E_REGMASK              ((uint8_t)0x0F)

#define S2LP_PLL_CP_ISEL_REGMASK   ((uint8_t)0xE0)
#define S2LP_BS_REGMASK            ((uint8_t)0x10)
#define S2LP_SYNT_27_24_REGMASK    ((uint8_t)0x0F)

#define S2LP_PLL_PFD_SPLIT_EN_REGMASK  ((uint8_t)0x04)

// Csma registers
#define S2LP_CSMA_CONF3_ADDR       ((uint8_t)0x4C)
#define S2LP_CSMA_CONF2_ADDR       ((uint8_t)0x4D)
#define S2LP_CSMA_CONF1_ADDR       ((uint8_t)0x4E)
#define S2LP_CSMA_CONF0_ADDR       ((uint8_t)0x4F)

// Csma registers masks
#define S2LP_CSMA1_BU_PRSC_REGMASK      ((uint8_t)0xFC)
#define S2LP_CSMA1_CCA_PERIOD_REGMASK   ((uint8_t)0x03)

#define S2LP_CSMA0_CCA_LEN_REGMASK      ((uint8_t)0xF0)
#define S2LP_CSMA0_CCA_LEN_OFFSET(x)    ((uint8_t)((x) << 4))
#define S2LP_CSMA0_NBACKOFF_MAX_REGMASK ((uint8_t)0x07)

// Power registers
#define S2LP_PA_POWER8_ADDR    ((uint8_t)0x5A)
#define S2LP_PA_POWER7_ADDR    ((uint8_t)0x5B)
#define S2LP_PA_POWER6_ADDR    ((uint8_t)0x5C)
#define S2LP_PA_POWER5_ADDR    ((uint8_t)0x5D)
#define S2LP_PA_POWER4_ADDR    ((uint8_t)0x5E)
#define S2LP_PA_POWER3_ADDR    ((uint8_t)0x5F)
#define S2LP_PA_POWER2_ADDR    ((uint8_t)0x60)
#define S2LP_PA_POWER1_ADDR    ((uint8_t)0x61)
#define S2LP_PA_POWER0_ADDR    ((uint8_t)0x62)
#define S2LP_PA_CONFIG1_ADDR   ((uint8_t)0x63)
#define S2LP_PA_CONFIG0_ADDR   ((uint8_t)0x64)

// Misc registers
#define S2LP_ANT_SELECT_CONF_ADDR ((uint8_t)0x1F)
#define S2LP_CLOCKREC1_ADDR       ((uint8_t)0x20)
#define S2LP_CLOCKREC0_ADDR       ((uint8_t)0x21)
#define S2LP_XO_RCO_CONF1_ADDR    ((uint8_t)0x6C)
#define S2LP_XO_RCO_CONF0_ADDR    ((uint8_t)0x6D)

// Misc registers masks
#define S2LP_ANT_EQU_CTRL_REGMASK        ((uint8_t)0x60)
#define S2LP_ANT_CS_BLANKING_REGMASK     ((uint8_t)0x10)
#define S2LP_ANT_AS_ENABLE_REGMASK       ((uint8_t)0x08)
#define S2LP_ANT_AS_MEAS_TIME_REGMASK    ((uint8_t)0x07)

#define S2LP_CLKREC1_P_GAIN_SLOW_REGMASK ((uint8_t)0xE0)
#define S2LP_CLKREC1_ALGO_SEL_REGMASK    ((uint8_t)0x10)
#define S2LP_CLKREC1_I_GAIN_SLOW_REGMASK ((uint8_t)0x0F)

#define S2LP_CLKREC0_P_GAIN_FAST_REGMASK ((uint8_t)0xE0)
#define S2LP_CLKREC0_PSTFLT_LEN_REGMASK  ((uint8_t)0x10)
#define S2LP_CLKREC0_I_GAIN_FAST_REGMASK ((uint8_t)0x0F)

#define S2LP_XO1_PD_CLKDIV_REGMASK       ((uint8_t)0x10)

#define S2LP_XO0_EXT_REF_REGMASK         ((uint8_t)0x80)
#define S2LP_XO0_GM_CONF_REGMASK         ((uint8_t)0x70)
#define S2LP_XO0_REFDIV_REGMASK          ((uint8_t)0x08)
#define S2LP_XO0_EXT_RCO_OSC_REGMASK     ((uint8_t)0x02)
#define S2LP_XO0_RCO_CALIBRATION_REGMASK ((uint8_t)0x01)

// Power registers masks
#define S2LP_DIG_SMOOTH_EN_REGMASK      ((uint8_t)0x80)
#define S2LP_PA_MAXDBM_REGMASK          ((uint8_t)0x40)
#define S2LP_PA_RAMP_EN_REGMASK         ((uint8_t)0x20)
#define S2LP_PA_RAMP_STEP_LEN_REGMASK   ((uint8_t)0x18)
#define S2LP_PA_LEVEL_MAX_IDX_REGMASK   ((uint8_t)0x07)

#define S2LP_LIN_NLOG_REGMASK       ((uint8_t)0x10)
#define S2LP_FIR_CFG_REGMASK        ((uint8_t)0xC0)
#define S2LP_FIR_EN_REGMASK         ((uint8_t)0x02)

#define S2LP_PA_DEGEN_TRIM_REGMASK  ((uint8_t)0xF0)
#define S2LP_PA_DEGEN_ON_REGMASK    ((uint8_t)0x08)
#define S2LP_SAFE_ASK_CAL_REGMASK   ((uint8_t)0x04)
#define S2LP_PA_FC_REGMASK          ((uint8_t)0x03)

// Rx registers
#define S2LP_AFC_CORR_ADDR               ((uint8_t)0x9E)
#define S2LP_LINK_QUALIF2_ADDR           ((uint8_t)0x9F)
#define S2LP_LINK_QUALIF1_ADDR           ((uint8_t)0xA0)
#define S2LP_RSSI_LEVEL_ADDR             ((uint8_t)0xA2)
#define S2LP_RX_PCKT_LEN1_ADDR           ((uint8_t)0xA4)
#define S2LP_RX_PCKT_LEN0_ADDR           ((uint8_t)0xA5)
#define S2LP_RSSI_LEVEL_RUN_ADDR         ((uint8_t)0xEF)

// Rx registers masks
#define S2LP_LINK_QUALIF1_CS_REGMASK     ((uint8_t)0x80)
#define S2LP_LINK_QUALIF1_SQI_REGMASK    ((uint8_t)0x7F)

// Id registers
#define S2LP_DEVICE_INFO1_ADDR           ((uint8_t)0xF0)
#define S2LP_DEVICE_INFO0_ADDR           ((uint8_t)0xF1)

// State registers
#define S2LP_MC_STATE1_ADDR              ((uint8_t)0x8D)
#define S2LP_MC_STATE0_ADDR              ((uint8_t)0x8E)
#define S2LP_TX_FIFO_STATUS_ADDR         ((uint8_t)0x8F)
#define S2LP_RX_FIFO_STATUS_ADDR         ((uint8_t)0x90)

// State registers masks
#define S2LP_STATE1_RCO_CAL_OK_REGMASK      ((uint8_t)0x10)
#define S2LP_STATE1_ANT_SEL_REGMASK         ((uint8_t)0x08)
#define S2LP_STATE1_TX_FIFO_FULL_REGMASK    ((uint8_t)0x04)
#define S2LP_STATE1_RX_FIFO_EMPTY_REGMASK   ((uint8_t)0x02)
#define S2LP_STATE1_ERROR_LOCK_REGMASK      ((uint8_t)0x01)

#define S2LP_STATE0_STATE_OFFSET(x)         (uint8_t)((x) << 1)
#define S2LP_STATE0_STATE_REGMASK           ((uint8_t)0xFE)
#define S2LP_STATE0_XO_ON_REGMASK           ((uint8_t)0x01)

// Fifo address (Write op for tx fifo, read op for rx fifo)
#define S2LP_FIFO_ADDRESS            ((uint8_t)0xFF)





// Command list
#define S2LP_CMD_TX              ((uint8_t)(0x60)),   // Start to transmit; valid only from READY
#define S2LP_CMD_RX              ((uint8_t)(0x61)),   // Start to receive; valid only from READY
#define S2LP_CMD_READY           ((uint8_t)(0x62)),   // Go to READY; valid only from STANDBY or SLEEP or LOCK
#define S2LP_CMD_STANDBY         ((uint8_t)(0x63)),   // Go to STANDBY; valid only from READY
#define S2LP_CMD_SLEEP           ((uint8_t)(0x64)),   // Go to SLEEP; valid only from READY
#define S2LP_CMD_LOCKRX          ((uint8_t)(0x65)),   // Go to LOCK state by using the RX configuration of the synth; valid only from READY
#define S2LP_CMD_LOCKTX          ((uint8_t)(0x66)),   // Go to LOCK state by using the TX configuration of the synth; valid only from READY
#define S2LP_CMD_SABORT          ((uint8_t)(0x67)),   // Force exit form TX or RX states and go to READY state; valid only from TX or RX
#define S2LP_CMD_LDC_RELOAD      ((uint8_t)(0x68)),   // LDC Mode: Reload the LDC timer with the value stored in the  LDC_PRESCALER / COUNTER  registers; valid from all states
#define S2LP_CMD_RCO_CALIB       ((uint8_t)(0x69)),   // Start (or re-start) the RCO calibration
#define S2LP_CMD_SRES            ((uint8_t)(0x70)),   // Reset of all digital part, except SPI registers
#define S2LP_CMD_FLUSHRXFIFO     ((uint8_t)(0x71)),   // Clean the RX FIFO; valid from all states
#define S2LP_CMD_FLUSHTXFIFO     ((uint8_t)(0x72)),   // Clean the TX FIFO; valid from all states
#define S2LP_CMD_SEQUENCE_UPDATE ((uint8_t)(0x73)),   // Autoretransmission: Reload the Packet sequence counter with the value stored in the PROTOCOL[2] register valid from all states

// GPIO registers values
#define S2LP_GPIO_MODE_DIGITAL_INPUT     ((uint8_t)0x01) // Digital Input on GPIO
#define S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP ((uint8_t)0x02) // Digital Output on GPIO (low current)
#define S2LP_GPIO_MODE_DIGITAL_OUTPUT_HP ((uint8_t)0x03) // Digital Output on GPIO (high current)

#define S2LP_GPIO_DIG_OUT_IRQ                       ((uint8_t)0x00) // nIRQ (Interrupt Request, active low) , default configuration after POR
#define S2LP_GPIO_DIG_OUT_POR_INV                   ((uint8_t)0x08) // POR inverted (active low)
#define S2LP_GPIO_DIG_OUT_WUT_EXP                   ((uint8_t)0x10) // Wake-Up Timer expiration: "1" when WUT has expired
#define S2LP_GPIO_DIG_OUT_LBD                       ((uint8_t)0x18) // Low battery detection: "1" when battery is below threshold setting
#define S2LP_GPIO_DIG_OUT_TX_DATA                   ((uint8_t)0x20) // TX data internal clock output (TX data are sampled on the rising edge of it)
#define S2LP_GPIO_DIG_OUT_TX_STATE                  ((uint8_t)0x28) // TX state indication: "1" when S2LP1 is passing in the TX state
#define S2LP_GPIO_DIG_OUT_TXRX_FIFO_ALMOST_EMPTY    ((uint8_t)0x30) // TX/RX FIFO Almost Empty Flag
#define S2LP_GPIO_DIG_OUT_TXRX_FIFO_ALMOST_FULL     ((uint8_t)0x38) // TX/RX FIFO Almost Full Flag
#define S2LP_GPIO_DIG_OUT_RX_DATA                   ((uint8_t)0x40) // RX data output
#define S2LP_GPIO_DIG_OUT_RX_CLOCK                  ((uint8_t)0x48) // RX clock output (recovered from received data)
#define S2LP_GPIO_DIG_OUT_RX_STATE                  ((uint8_t)0x50) // RX state indication: "1" when demodulator is ON
#define S2LP_GPIO_DIG_OUT_NOT_STANDBY_SLEEP         ((uint8_t)0x58) // VDD when the device is not in SLEEP or STANDBY
#define S2LP_GPIO_DIG_OUT_STANDBY                   ((uint8_t)0x60) // VDD when device is in STANDBY
#define S2LP_GPIO_DIG_OUT_ANTENNA_SWITCH            ((uint8_t)0x68) // Antenna switch used for antenna diversity
#define S2LP_GPIO_DIG_OUT_VALID_PREAMBLE            ((uint8_t)0x70) // Valid Preamble Detected Flag
#define S2LP_GPIO_DIG_OUT_SYNC_DETECTED             ((uint8_t)0x78) // Sync WordSync Word Detected Flag
#define S2LP_GPIO_DIG_OUT_RSSI_THRESHOLD            ((uint8_t)0x80) // RSSI above threshold
#define S2LP_GPIO_DIG_OUT_MCU_CLOCK                 ((uint8_t)0x88) // MCU Clock
#define S2LP_GPIO_DIG_OUT_TX_RX_MODE                ((uint8_t)0x90) // TX or RX mode indicator (to enable an external range extender)
#define S2LP_GPIO_DIG_OUT_VDD                       ((uint8_t)0x98) // VDD (to emulate an additional GPIO of the MCU, programmable by SPI)
#define S2LP_GPIO_DIG_OUT_GND                       ((uint8_t)0xA0) // GND (to emulate an additional GPIO of the MCU, programmable by SPI)
#define S2LP_GPIO_DIG_OUT_SMPS_EXT                  ((uint8_t)0xA8) // External SMPS enable signal (active high)
#define S2LP_GPIO_DIG_OUT_SLEEP                     ((uint8_t)0xB0) // Device in SLEEP (active high)
#define S2LP_GPIO_DIG_OUT_READY                     ((uint8_t)0xB8) // Device in READY (active high)
#define S2LP_GPIO_DIG_OUT_LOCK                      ((uint8_t)0xC0) // Device in LOCK (active high)
#define S2LP_GPIO_DIG_OUT_WAIT_FOR_LOCK_SIG         ((uint8_t)0xC8) // Device waiting for LOCK (active high)
#define S2LP_GPIO_DIG_OUT_TX_DATA_OOK_SIGNAL        ((uint8_t)0xD0)
#define S2LP_GPIO_DIG_OUT_WAIT_FOR_READY2_SIG       ((uint8_t)0xD8)
#define S2LP_GPIO_DIG_OUT_WAIT_FOR_TIMER_FOR_PM_SET ((uint8_t)0xE0)
#define S2LP_GPIO_DIG_OUT_WAIT_VCO_CALIBRATION      ((uint8_t)0xE8)
#define S2LP_GPIO_DIG_OUT_ENABLE_SYNTH_FULL_CIRCUIT ((uint8_t)0xF0)
#define S2LP_GPIO_DIG_IN_TX_COMMAND                 ((uint8_t)0x00)
#define S2LP_GPIO_DIG_IN_RX_COMMAND                 ((uint8_t)0x08)
#define S2LP_GPIO_DIG_IN_TX_DATA_INPUT_FOR_DIRECTRF ((uint8_t)0x10)
#define S2LP_GPIO_DIG_IN_DATA_WAKEUP                ((uint8_t)0x18)
#define S2LP_GPIO_DIG_IN_EXT_CLOCK_AT_34_7KHZ       ((uint8_t)0x20)

// IRQ 0 registers values
#define S2LP_IRQ_RX_DATA_READY       (1 << 0)      // RX data ready
#define S2LP_IRQ_RX_DATA_DISC        (1 << 1)      // RX data discarded (upon filtering)
#define S2LP_IRQ_TX_DATA_SENT        (1 << 2)      // TX data sent
#define S2LP_IRQ_MAX_RE_TX_REACH     (1 << 3)      // Max re-TX reached
#define S2LP_IRQ_CRC_ERROR           (1 << 4)      // CRC error
#define S2LP_IRQ_TX_FIFO_ERROR       (1 << 5)      // TX FIFO underflow/overflow error
#define S2LP_IRQ_RX_FIFO_ERROR       (1 << 6)      // RX FIFO underflow/overflow error
#define S2LP_IRQ_TX_FIFO_ALMOST_FULL (1 << 7)      // TX FIFO almost full
// IRQ 1
#define S2LP_IRQ_TX_FIFO_ALMOST_EMPTY (1 << 0)     // TX FIFO almost empty
#define S2LP_IRQ_RX_FIFO_ALMOST_FULL  (1 << 1)     // RX FIFO almost full
#define S2LP_IRQ_RX_FIFO_ALMOST_EMPTY (1 << 2)     // RX FIFO almost empty
#define S2LP_IRQ_MAX_BO_CCA_REACHED   (1 << 3)     // Max number of back-off during CCA
#define S2LP_IRQ_VALID_PREAMBLE       (1 << 4)     // Valid preamble detected
#define S2LP_IRQ_VALID_SYNC           (1 << 5)     // Sync word detected
#define S2LP_IRQ_RSSI_ABOVE_TH        (1 << 6)     // RSSI above threshold
#define S2LP_IRQ_WKUP_TOUT_LDC        (1 << 7)     // Wake-up timeout in LDC mode
// IRQ 2
#define S2LP_IRQ_READY               (1 << 0)      // READY state
#define S2LP_IRQ_STANDBY_DELAYED     (1 << 1)      // STANDBY state after MCU_CK_CONF_CLOCK_TAIL_X clock cycles
#define S2LP_IRQ_LOW_BATT_LVL        (1 << 2)      // Battery level below threshold
#define S2LP_IRQ_POR                 (1 << 3)      // Power On Reset
#define S2LP_IRQ_BOR                 (1 << 4)      // Brown out event (both accurate and inaccurate)
#define S2LP_IRQ_LOCK                (1 << 5)      // LOCK state
#define S2LP_IRQ_VCO_CALIBRATION_END (1 << 6)      // End of VCO calibration procedure
#define S2LP_IRQ_PA_CALIBRATION_END  (1 << 7)      // End of PA calibration procedure
// IRQ 3
#define S2LP_IRQ_PM_COUNT_EXPIRED    (1 << 0)      // only for debug; Power Management startup timer expiration (see reg PM_START_COUNTER, 0xB5)
#define S2LP_IRQ_XO_COUNT_EXPIRED    (1 << 1)      // only for debug; Crystal oscillator settling time counter expired
#define S2LP_IRQ_TX_START_TIME       (1 << 2)      // only for debug; TX circuitry startup time; see TX_START_COUNTER
#define S2LP_IRQ_RX_START_TIME       (1 << 3)      // only for debug; RX circuitry startup time; see TX_START_COUNTER
#define S2LP_IRQ_RX_TIMEOUT          (1 << 4)      // RX operation timeout
#define S2LP_IRQ_RX_SNIFF_TIMEOUT    (1 << 5)      // RX sniff opeartion timeout

// Packet register values
#define S2LP_PKT_PREAMBLE_MODE_1 ((uint8_t)0x00) // 0101 (2FSK) / 0111 (4FSK)
#define S2LP_PKT_PREAMBLE_MODE_2 ((uint8_t)0x01) // 1010 (2FSK) / 0010 (4FSK)
#define S2LP_PKT_PREAMBLE_MODE_3 ((uint8_t)0x02) // 1100 (2FSK) / 1101 (4FSK)
#define S2LP_PKT_PREAMBLE_MODE_4 ((uint8_t)0x03) // 0011 (2FSK) / 1000 (4FSK)

#define S2LP_PKT_NO_CRC               ((uint8_t)0x00) // No CRC
#define S2LP_PKT_CRC_MODE_8BITS       ((uint8_t)0x20) // CRC length 8 bits  - poly: 0x07
#define S2LP_PKT_CRC_MODE_16BITS_1    ((uint8_t)0x40) // CRC length 16 bits - poly: 0x8005
#define S2LP_PKT_CRC_MODE_16BITS_2    ((uint8_t)0x60) // CRC length 16 bits - poly: 0x1021
#define S2LP_PKT_CRC_MODE_24BITS      ((uint8_t)0x80) // CRC length 24 bits - poly: 0x864CFB
#define S2LP_PKT_CRC_MODE_32BITS      ((uint8_t)0xA0) // CRC length 32 bits - poly: 0x04C011BB7

// Rf register values
#define S2LP_MOD_NO_MOD       0x70 // CW modulation selected
#define S2LP_MOD_2FSK         0x00 // 2-FSK modulation selected
#define S2LP_MOD_4FSK         0x10 // 4-FSK modulation selected
#define S2LP_MOD_2GFSK_BT05   0xA0 // G2FSK modulation selected with BT = 0.5
#define S2LP_MOD_2GFSK_BT1    0x20 // G2FSK modulation selected with BT = 1
#define S2LP_MOD_4GFSK_BT05   0xB0 // G4FSK modulation selected with BT = 0.5
#define S2LP_MOD_4GFSK_BT1    0x30 // G4FSK modulation selected with BT = 1
#define S2LP_MOD_ASK_OOK      0x50 // OOK modulation selected.
#define S2LP_MOD_POLAR        0x60 // OOK modulation selected.

// CSMA registers values
#define S2LP_CSMA_CONF1_DEF_VAL  0x4
#define S2LP_CSMA_PERIOD_64TBIT  0   // CSMA/CA: Sets CCA period to 64*TBIT 
#define S2LP_CSMA_PERIOD_128TBIT 1   // CSMA/CA: Sets CCA period to 128*TBIT 
#define S2LP_CSMA_PERIOD_256TBIT 2   // CSMA/CA: Sets CCA period to 256*TBIT 
#define S2LP_CSMA_PERIOD_512TBIT 3   // CSMA/CA: Sets CCA period to 512*TBIT 

// Id registers value
#define S2LP_PART_NUMBER        0x03
#define S2LP_VERSION_NUMBER     0xC1

// State registers value
#define S2LP_STATE_READY        0x00 // READY
#define S2LP_STATE_SLEEP_NOFIFO 0x01 // SLEEP NO FIFO RETENTION
#define S2LP_STATE_STANDBY      0x02 // STANDBY
#define S2LP_STATE_SLEEP        0x03 // SLEEP
#define S2LP_STATE_LOCKON       0x0C // LOCKON
#define S2LP_STATE_RX           0x30 // RX
#define S2LP_STATE_LOCK_ST      0x14 // LOCK_ST
#define S2LP_STATE_TX           0x5C // TX
#define S2LP_STATE_SYNTH_SETUP  0x50 // SYNTH_SETUP

#endif