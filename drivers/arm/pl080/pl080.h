#ifndef PL080_H_
#define PL080_H_

//
// Register and bit definitions for peripheral 'PL080'
//

// PL080.DMACIntStatus Interrupt error status
#define PL080_INTSTATUS_OFFSET 0x00000000
#define PL080_INTSTATUS_INTSTATUS_MASK 0xff
#define PL080_INTSTATUS_INTSTATUS_POS 0
#define PL080_INTSTATUS_INTSTATUS_WIDTH 8

// PL080.DMACIntTCStatus Terminal count interrupt status
#define PL080_INTTCSTATUS_OFFSET 0x00000004
#define PL080_INTTCSTATUS_INTTCSTATUS_MASK 0xff
#define PL080_INTTCSTATUS_INTTCSTATUS_POS 0
#define PL080_INTTCSTATUS_INTTCSTATUS_WIDTH 8

// PL080.DMACIntTCClear Terminal count interrupt clear
#define PL080_INTTCCLEAR_OFFSET 0x00000008
#define PL080_INTTCCLEAR_INTTCCLEAR_MASK 0xff
#define PL080_INTTCCLEAR_INTTCCLEAR_POS 0
#define PL080_INTTCCLEAR_INTTCCLEAR_WIDTH 8

// PL080.DMACIntErrorStatus Interrupt error status
#define PL080_INTERRORSTATUS_OFFSET 0x0000000c
#define PL080_INTERRORSTATUS_INTERRORSTATUS_MASK 0xff
#define PL080_INTERRORSTATUS_INTERRORSTATUS_POS 0
#define PL080_INTERRORSTATUS_INTERRORSTATUS_WIDTH 8

// PL080.DMACIntErrClr Clear the error interrupt request.
#define PL080_INTERRCLR_OFFSET 0x00000010
#define PL080_INTERRCLR_INTERRCLR_MASK 0xff
#define PL080_INTERRCLR_INTERRCLR_POS 0
#define PL080_INTERRCLR_INTERRCLR_WIDTH 8

// PL080.DMACRawIntTCStatus Terminal count interrupt status before masking.
#define PL080_RAWINTTCSTATUS_OFFSET 0x00000014
#define PL080_RAWINTTCSTATUS_RAWINTTCSTATUS_MASK 0xff
#define PL080_RAWINTTCSTATUS_RAWINTTCSTATUS_POS 0
#define PL080_RAWINTTCSTATUS_RAWINTTCSTATUS_WIDTH 8

// PL080.DMACRawIntErrorStatus Raw interrupt error status of DMA channels
#define PL080_RAWINTERRORSTATUS_OFFSET 0x00000018
#define PL080_RAWINTERRORSTATUS_RAWINTERRORSTATUS_MASK 0xff
#define PL080_RAWINTERRORSTATUS_RAWINTERRORSTATUS_POS 0
#define PL080_RAWINTERRORSTATUS_RAWINTERRORSTATUS_WIDTH 8

// PL080.DMACEnbldChns Enabled channels
#define PL080_ENBLDCHNS_OFFSET 0x0000001c
#define PL080_ENBLDCHNS_ENABLEDCHANNELS_MASK 0xff
#define PL080_ENBLDCHNS_ENABLEDCHANNELS_POS 0
#define PL080_ENBLDCHNS_ENABLEDCHANNELS_WIDTH 8

// PL080.DMACSoftBReq Enables DMA burst request to be generated by software
#define PL080_SOFTBREQ_OFFSET 0x00000020
#define PL080_SOFTBREQ_SOFTBREQ_MASK 0xffff
#define PL080_SOFTBREQ_SOFTBREQ_POS 0
#define PL080_SOFTBREQ_SOFTBREQ_WIDTH 16

// PL080.DMACSoftSReq Enables DMA single requests to be generated by software
#define PL080_SOFTSREQ_OFFSET 0x00000024
#define PL080_SOFTSREQ_SOFTSREQ_MASK 0xffff
#define PL080_SOFTSREQ_SOFTSREQ_POS 0
#define PL080_SOFTSREQ_SOFTSREQ_WIDTH 16

// PL080.DMACSoftLBReq Enables software to generate DMA last burst requests
#define PL080_SOFTLBREQ_OFFSET 0x00000028
#define PL080_SOFTLBREQ_SOFTLBREQ_MASK 0xffff
#define PL080_SOFTLBREQ_SOFTLBREQ_POS 0
#define PL080_SOFTLBREQ_SOFTLBREQ_WIDTH 16

// PL080.DMACSoftLSReq Enables software to generate DMA last single requests.
#define PL080_SOFTLSREQ_OFFSET 0x0000002c
#define PL080_SOFTLSREQ_SOFTLSREQ_MASK 0xffff
#define PL080_SOFTLSREQ_SOFTLSREQ_POS 0
#define PL080_SOFTLSREQ_SOFTLSREQ_WIDTH 16

// PL080.DMACConfiguration Configures the operation of the DMAC
#define PL080_CONFIGURATION_OFFSET 0x00000030
#define PL080_CONFIGURATION_M2_MASK 0x4
#define PL080_CONFIGURATION_M2_POS 2
#define PL080_CONFIGURATION_M2_WIDTH 1
#define PL080_CONFIGURATION_M1_MASK 0x2
#define PL080_CONFIGURATION_M1_POS 1
#define PL080_CONFIGURATION_M1_WIDTH 1
#define PL080_CONFIGURATION_E_MASK 0x1
#define PL080_CONFIGURATION_E_POS 0
#define PL080_CONFIGURATION_E_WIDTH 1

// PL080.DMACSync Enables or disables synchronization logic for the
// DMA request signals Note: You must use synchronization logic when
// the peripheral generating the DMA request runs on a different clock
// to the DMAC. For peripherals running on the same clock as the DMAC,
// disabling the synchronization logic improves the DMA request
// response time. If necessary, synchronize the DMA response signals,
// DMACCLR and DMACTC, in the peripheral
#define PL080_SYNC_OFFSET 0x00000034
#define PL080_SYNC_DMACSYNC_MASK 0xffff
#define PL080_SYNC_DMACSYNC_POS 0
#define PL080_SYNC_DMACSYNC_WIDTH 16

#define PL080_CHAN_OFFSET 0x00000100
#define PL080_CHAN_STRIDE 0x00000020
#define PL080_CHAN_COUNT 8

// PL080.DMACC0SrcAddr Contain the current source address,
// byte-aligned, of the data to be transferred. Software programs each
// register directly before the appropriate channel is enabled When
// the DMA channel is enabled, this register is updated: as the source
// address is incremented or by following the linked list when a
// complete packet of data has been transferred. Reading the register
// when the channel is active does not provide useful
// information. This is because by the time the software has processed
// the value read, the channel might have progressed. It is intended
// to be read-only when the channel has stopped, and in such case, it
// shows the source address of the last item read Note: You must align
// source and destination addresses to the source and destination
// widths
#define PL080_CHAN_SRCADDR_OFFSET 0x00000000
#define PL080_SRCADDR_SRCADDR_MASK 0xffffffff
#define PL080_SRCADDR_SRCADDR_POS 0
#define PL080_SRCADDR_SRCADDR_WIDTH 32

// PL080.DMACC0DestAddr Contain the current destination address,
// byte-aligned, of the data to be transferred.Software programs each
// register directly before the channel is enabled. When the DMA
// channel is enabled, the register is updated as the destination
// address is incremented and by following the linked list when a
// complete packet of data has been transferred. Reading the register
// when the channel is active does not provide useful
// information. This is because by the time the software has processed
// the value read, the channel might have progressed. It is intended
// to be read-only when a channel has stopped. In this case, it shows
// the destination address of the last item read
#define PL080_CHAN_DESTADDR_OFFSET 0x00000004
#define PL080_DESTADDR_DESTADDR_MASK 0xffffffff
#define PL080_DESTADDR_DESTADDR_POS 0
#define PL080_DESTADDR_DESTADDR_WIDTH 32

// PL080.DMACC0LLI Contain a word-aligned address of the next LLI. If
// the LLI is 0, then the current LLI is the last in the chain, and
// the DMA channel is disabled after all DMA transfers associated with
// it are completed
#define PL080_CHAN_LLI_OFFSET 0x00000008
#define PL080_LLI_LLI_MASK 0xfffffffc
#define PL080_LLI_LLI_POS 2
#define PL080_LLI_LLI_WIDTH 30
#define PL080_LLI_LM_MASK 0x1
#define PL080_LLI_LM_POS 0
#define PL080_LLI_LM_WIDTH 1

// PL080.DMACC0Control Contain DMA channel control information such as
// the transfer size, burst size, and transfer width. Software
// programs each register directly before the DMA channel is
// enabled. When the channel is enabled, the register is updated by
// following the linked list when a complete packet of data has been
// transferred. Reading the register while the channel is active does
// not give useful information
#define PL080_CHAN_CONTROL_OFFSET 0x0000000c
#define PL080_CONTROL_I_MASK 0x80000000
#define PL080_CONTROL_I_POS 31
#define PL080_CONTROL_I_WIDTH 1
#define PL080_CONTROL_PROT_MASK 0x70000000
#define PL080_CONTROL_PROT_POS 28
#define PL080_CONTROL_PROT_WIDTH 3
#define PL080_CONTROL_DI_MASK 0x8000000
#define PL080_CONTROL_DI_POS 27
#define PL080_CONTROL_DI_WIDTH 1
#define PL080_CONTROL_SI_MASK 0x4000000
#define PL080_CONTROL_SI_POS 26
#define PL080_CONTROL_SI_WIDTH 1
#define PL080_CONTROL_D_MASK 0x2000000
#define PL080_CONTROL_D_POS 25
#define PL080_CONTROL_D_WIDTH 1
#define PL080_CONTROL_S_MASK 0x1000000
#define PL080_CONTROL_S_POS 24
#define PL080_CONTROL_S_WIDTH 1
#define PL080_CONTROL_DWIDTH_MASK 0xe00000
#define PL080_CONTROL_DWIDTH_POS 21
#define PL080_CONTROL_DWIDTH_WIDTH 3
#define PL080_CONTROL_SWIDTH_MASK 0x1c0000
#define PL080_CONTROL_SWIDTH_POS 18
#define PL080_CONTROL_SWIDTH_WIDTH 3
#define PL080_CONTROL_DBSIZE_MASK 0x38000
#define PL080_CONTROL_DBSIZE_POS 15
#define PL080_CONTROL_DBSIZE_WIDTH 3
#define PL080_CONTROL_SBSIZE_MASK 0x7000
#define PL080_CONTROL_SBSIZE_POS 12
#define PL080_CONTROL_SBSIZE_WIDTH 3
#define PL080_CONTROL_TRANSFERSIZE_MASK 0xfff
#define PL080_CONTROL_TRANSFERSIZE_POS 0
#define PL080_CONTROL_TRANSFERSIZE_WIDTH 12

// PL080.DMACC0Configuration Configures the DMA channel. The registers
// are not updated when a new LLI is requested.
#define PL080_CHAN_CONFIGURATION_OFFSET 0x00000010
#define PL080_CONFIGURATION_H_MASK 0x40000
#define PL080_CONFIGURATION_H_POS 18
#define PL080_CONFIGURATION_H_WIDTH 1
#define PL080_CONFIGURATION_A_MASK 0x20000
#define PL080_CONFIGURATION_A_POS 17
#define PL080_CONFIGURATION_A_WIDTH 1
#define PL080_CONFIGURATION_L_MASK 0x10000
#define PL080_CONFIGURATION_L_POS 16
#define PL080_CONFIGURATION_L_WIDTH 1
#define PL080_CONFIGURATION_ITC_MASK 0x8000
#define PL080_CONFIGURATION_ITC_POS 15
#define PL080_CONFIGURATION_ITC_WIDTH 1
#define PL080_CONFIGURATION_IE_MASK 0x4000
#define PL080_CONFIGURATION_IE_POS 14
#define PL080_CONFIGURATION_IE_WIDTH 1
#define PL080_CONFIGURATION_FLOWCNTRL_MASK 0x3800
#define PL080_CONFIGURATION_FLOWCNTRL_POS 11
#define PL080_CONFIGURATION_FLOWCNTRL_WIDTH 3
#define PL080_CONFIGURATION_DESTPERIPHERAL_MASK 0x3c0
#define PL080_CONFIGURATION_DESTPERIPHERAL_POS 6
#define PL080_CONFIGURATION_DESTPERIPHERAL_WIDTH 4
#define PL080_CONFIGURATION_SRCPERIPHERAL_MASK 0x1e
#define PL080_CONFIGURATION_SRCPERIPHERAL_POS 1
#define PL080_CONFIGURATION_SRCPERIPHERAL_WIDTH 4
/*
  Redef
#define PL080_CONFIGURATION_E_MASK 0x1
#define PL080_CONFIGURATION_E_POS 0
#define PL080_CONFIGURATION_E_WIDTH 1
*/

// PL080.DMACITCR DMA controller test control register
#define PL080_ITCR_OFFSET 0x00000500
#define PL080_ITCR_T_MASK 0x1
#define PL080_ITCR_T_POS 0
#define PL080_ITCR_T_WIDTH 1

// PL080.DMACITOP1 DMA controller integration test output register1
#define PL080_ITOP1_OFFSET 0x00000504
#define PL080_ITOP1_DMACCLR_MASK 0xffff
#define PL080_ITOP1_DMACCLR_POS 0
#define PL080_ITOP1_DMACCLR_WIDTH 16

// PL080.DMACITOP2 DMA controller integration test output register2
#define PL080_ITOP2_OFFSET 0x00000508
#define PL080_ITOP2_DMACTC_MASK 0xffff
#define PL080_ITOP2_DMACTC_POS 0
#define PL080_ITOP2_DMACTC_WIDTH 16

// PL080.DMACITOP3 DMA controller integration test output register3
#define PL080_ITOP3_OFFSET 0x0000050c
#define PL080_ITOP3_E_MASK 0x2
#define PL080_ITOP3_E_POS 1
#define PL080_ITOP3_E_WIDTH 1
#define PL080_ITOP3_TC_MASK 0x1
#define PL080_ITOP3_TC_POS 0
#define PL080_ITOP3_TC_WIDTH 1

#define PL080_DEVID0_OFFSET 0x00000fc0
#define PL080_DEVID1_OFFSET 0x00000fc4
#define PL080_DEVID2_OFFSET 0x00000fc8
#define PL080_DEVID3_OFFSET 0x00000fcc
#define PL080_PID10_OFFSET 0x00000fd0
#define PL080_PID11_OFFSET 0x00000fd4
#define PL080_PID12_OFFSET 0x00000fd8
#define PL080_PID13_OFFSET 0x00000fdc
#define PL080_PID00_OFFSET 0x00000fe0
#define PL080_PID01_OFFSET 0x00000fe4
#define PL080_PID02_OFFSET 0x00000fe8
#define PL080_PID03_OFFSET 0x00000fec
#define PL080_CID0_OFFSET 0x00000ff0
#define PL080_CID1_OFFSET 0x00000ff4
#define PL080_CID2_OFFSET 0x00000ff8
#define PL080_CID3_OFFSET 0x00000ffc

#define pl080_set32(inst, name, value)                             \
    cpu_mem_write_32((uint32_t)(inst) + PL080_##name##_OFFSET, (value))

#define pl080_get32(inst, name)                                   \
    cpu_mem_read_32((uint32_t)(inst) + PL080_##name##_OFFSET)

#define pl080_set8(inst, name, value)                             \
    cpu_mem_write_8((uint32_t)(inst) + PL080_##name##_OFFSET, (value))

#define pl080_get8(inst, name)                                   \
    cpu_mem_read_8((uint32_t)(inst) + PL080_##name##_OFFSET)

#define pl080_get(v32, reg, field)                                \
    ((v32 & PL080_##reg##_##field##_MASK)                         \
     >> PL080_##reg##_##field##_POS)

#define pl080_ins_val(v32, reg, field, value)                       \
    ((v32 & ~PL080_##reg##_##field##_MASK) |                        \
     (PL080_##reg##_##field##_MASK & ((value) << PL080_##reg##_##field##_POS)))

#define pl080_ins_enum(v32, reg, field, value)                      \
    pl080_ins_val(v32, reg, field, PL080_##reg##_##field##_##value)

#define pl080_bit(reg, field)                           \
    (PL080_##reg##_##field##_MASK)


#define pl080_chan_reg_addr(inst, chan, name)                                    \
    ((uint32_t)(inst) + PL080_CHAN_OFFSET + ((chan) * PL080_CHAN_STRIDE) + PL080_CHAN_##name##_OFFSET)

#define pl080_chan_set32(inst, chan, name, value)                             \
    cpu_mem_write_32(pl080_chan_reg_addr(inst, chan, name), (value))

#define pl080_chan_get32(inst, chan, name)                                   \
    cpu_mem_read_32(pl080_chan_reg_addr(inst, chan, name))

#define pl080_chan_set8(inst, chan, name, value)                             \
    cpu_mem_write_8(pl080_chan_reg_addr(inst, chan, name), (value))

#define pl080_chan_get8(inst, chan, name)                                   \
    cpu_mem_read_8(pl080_chan_reg_addr(inst, chan, name))

#endif