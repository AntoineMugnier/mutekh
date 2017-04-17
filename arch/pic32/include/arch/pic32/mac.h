/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs cdefs_use_reg_mask=1 cdefs_use_field_setval=1
*/

#ifndef _PIC32_MAC_BFGEN_DEFS_
#define _PIC32_MAC_BFGEN_DEFS_

#define PIC32_MAC_CON1_ADDR                          0x00000000
#define PIC32_MAC_CON1_MASK                          0xffffa391
  #define PIC32_MAC_CON1_BUFCDEC                   0x00000001
  #define PIC32_MAC_CON1_MANFC                     0x00000010
  #define PIC32_MAC_CON1_AUTOFC                    0x00000080
  #define PIC32_MAC_CON1_RXEN                      0x00000100
  #define PIC32_MAC_CON1_TXRTS                     0x00000200
  #define PIC32_MAC_CON1_SIDL                      0x00002000
  #define PIC32_MAC_CON1_ON                        0x00008000
  #define PIC32_MAC_CON1_PTV(v)                    ((v) << 16)
  #define PIC32_MAC_CON1_PTV_SET(x, v)             do { (x) = (((x) & ~0xffff0000) | ((v) << 16)); } while(0)
  #define PIC32_MAC_CON1_PTV_GET(x)                (((x) >> 16) & 0xffff)

#define PIC32_MAC_CON2_ADDR                          0x00000010
#define PIC32_MAC_CON2_MASK                          0x000007ff
/** Must be aligned aligned on a boundary of 16 bytes @multiple */
  #define PIC32_MAC_CON2_RXBUFSZ(v)                ((v) << 0)
  #define PIC32_MAC_CON2_RXBUFSZ_SET(x, v)         do { (x) = (((x) & ~0x7ff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_CON2_RXBUFSZ_GET(x)            (((x) >> 0) & 0x7ff)

#define PIC32_MAC_TXST_ADDR                          0x00000020
#define PIC32_MAC_TXST_MASK                          0x00000000

#define PIC32_MAC_RXST_ADDR                          0x00000030
#define PIC32_MAC_RXST_MASK                          0x00000000

#define PIC32_MAC_HT_ADDR(ridx)                      (0x00000040 + (ridx) * 16)
#define PIC32_MAC_HT_COUNT                           2
#define PIC32_MAC_HT_MASK                            0x00000000

#define PIC32_MAC_PMM_ADDR(ridx)                     (0x00000060 + (ridx) * 16)
#define PIC32_MAC_PMM_COUNT                          2
#define PIC32_MAC_PMM_MASK                           0x00000000

#define PIC32_MAC_PMCS_ADDR                          0x00000080
#define PIC32_MAC_PMCS_MASK                          0x0000ffff
  #define PIC32_MAC_PMCS_PMCS(v)                   ((v) << 0)
  #define PIC32_MAC_PMCS_PMCS_SET(x, v)            do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_PMCS_PMCS_GET(x)               (((x) >> 0) & 0xffff)

#define PIC32_MAC_PMO_ADDR                           0x00000090
#define PIC32_MAC_PMO_MASK                           0x0000ffff
  #define PIC32_MAC_PMO_PMO(v)                     ((v) << 0)
  #define PIC32_MAC_PMO_PMO_SET(x, v)              do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_PMO_PMO_GET(x)                 (((x) >> 0) & 0xffff)

#define PIC32_MAC_RXFC_ADDR                          0x000000a0
#define PIC32_MAC_RXFC_MASK                          0x0000dfff
  #define PIC32_MAC_RXFC_BCEN                      0x00000001
  #define PIC32_MAC_RXFC_MCEN                      0x00000002
  #define PIC32_MAC_RXFC_NOTMEEN                   0x00000004
  #define PIC32_MAC_RXFC_UCEN                      0x00000008
  #define PIC32_MAC_RXFC_RUNTEN                    0x00000010
  #define PIC32_MAC_RXFC_RUNTERREN                 0x00000020
  #define PIC32_MAC_RXFC_CRCOKEN                   0x00000040
  #define PIC32_MAC_RXFC_CRCERREN                  0x00000080
  #define PIC32_MAC_RXFC_PMMODE(v)                 ((PIC32_MAC_RXFC_PMMODE_##v) << 8)
  #define PIC32_MAC_RXFC_PMMODE_SET(x, v)          do { (x) = (((x) & ~0xf00) | ((PIC32_MAC_RXFC_PMMODE_##v) << 8)); } while(0)
  #define PIC32_MAC_RXFC_PMMODE_SETVAL(x, v)       do { (x) = (((x) & ~0xf00) | ((v) << 8)); } while(0)
  #define PIC32_MAC_RXFC_PMMODE_GET(x)             (((x) >> 8) & 0xf)
    #define PIC32_MAC_RXFC_PMMODE_DISABLED           0x00000000
/** Pattern match checksum matches */
    #define PIC32_MAC_RXFC_PMMODE_MATCH              0x00000001
/** Pattern match checksum matches and is destined to STA */
    #define PIC32_MAC_RXFC_PMMODE_STA_MATCH          0x00000002
/** Pattern match checksum matches and is unicast */
    #define PIC32_MAC_RXFC_PMMODE_UNICAST_MATCH      0x00000004
/** Pattern match checksum matches and is broadcast */
    #define PIC32_MAC_RXFC_PMMODE_BROADCAST_MATCH    0x00000006
/** Pattern match checksum and hashtable match */
    #define PIC32_MAC_RXFC_PMMODE_HT_MATCH           0x00000008
/** Pattern match checksum matches and is magic packet */
    #define PIC32_MAC_RXFC_PMMODE_MAGIC_MATCH        0x00000009
  #define PIC32_MAC_RXFC_NOTPM(v)                  ((PIC32_MAC_RXFC_NOTPM_##v) << 12)
  #define PIC32_MAC_RXFC_NOTPM_SET(x, v)           do { (x) = (((x) & ~0x1000) | ((PIC32_MAC_RXFC_NOTPM_##v) << 12)); } while(0)
  #define PIC32_MAC_RXFC_NOTPM_SETVAL(x, v)        do { (x) = (((x) & ~0x1000) | ((v) << 12)); } while(0)
  #define PIC32_MAC_RXFC_NOTPM_GET(x)              (((x) >> 12) & 0x1)
/** Packet must match to be marked as pattern matched */
    #define PIC32_MAC_RXFC_NOTPM_MUST_MATCH          0x00000000
/** Packet must not patch to be marked as pattern matched */
    #define PIC32_MAC_RXFC_NOTPM_MUST_NOT_MATCH      0x00000001
  #define PIC32_MAC_RXFC_MPEN                      0x00004000
  #define PIC32_MAC_RXFC_HTEN                      0x00008000

#define PIC32_MAC_RXWM_ADDR                          0x000000b0
#define PIC32_MAC_RXWM_MASK                          0x00ff00ff
  #define PIC32_MAC_RXWM_RXEWM(v)                  ((v) << 0)
  #define PIC32_MAC_RXWM_RXEWM_SET(x, v)           do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_RXWM_RXEWM_GET(x)              (((x) >> 0) & 0xff)
  #define PIC32_MAC_RXWM_RXFWM(v)                  ((v) << 16)
  #define PIC32_MAC_RXWM_RXFWM_SET(x, v)           do { (x) = (((x) & ~0xff0000) | ((v) << 16)); } while(0)
  #define PIC32_MAC_RXWM_RXFWM_GET(x)              (((x) >> 16) & 0xff)

#define PIC32_MAC_IEN_ADDR                           0x000000c0
#define PIC32_MAC_IEN_MASK                           0x000063ef
  #define PIC32_MAC_IEN_RXOVFLW                    0x00000001
  #define PIC32_MAC_IEN_RXBUFNA                    0x00000002
  #define PIC32_MAC_IEN_TXABORT                    0x00000004
  #define PIC32_MAC_IEN_TXDONE                     0x00000008
  #define PIC32_MAC_IEN_RXACT                      0x00000020
  #define PIC32_MAC_IEN_PKTPEND                    0x00000040
  #define PIC32_MAC_IEN_RXDONE                     0x00000080
  #define PIC32_MAC_IEN_FWMARK                     0x00000100
  #define PIC32_MAC_IEN_EWMARK                     0x00000200
  #define PIC32_MAC_IEN_RXBUSE                     0x00002000
  #define PIC32_MAC_IEN_TXBUSE                     0x00004000

#define PIC32_MAC_IRQ_ADDR                           0x000000d0
#define PIC32_MAC_IRQ_MASK                           0x000063ef
  #define PIC32_MAC_IRQ_RXOVFLW                    0x00000001
  #define PIC32_MAC_IRQ_RXBUFNA                    0x00000002
  #define PIC32_MAC_IRQ_TXABORT                    0x00000004
  #define PIC32_MAC_IRQ_TXDONE                     0x00000008
  #define PIC32_MAC_IRQ_RXACT                      0x00000020
  #define PIC32_MAC_IRQ_PKTPEND                    0x00000040
  #define PIC32_MAC_IRQ_RXDONE                     0x00000080
  #define PIC32_MAC_IRQ_FWMARK                     0x00000100
  #define PIC32_MAC_IRQ_EWMARK                     0x00000200
  #define PIC32_MAC_IRQ_RXBUSE                     0x00002000
  #define PIC32_MAC_IRQ_TXBUSE                     0x00004000

#define PIC32_MAC_STAT_ADDR                          0x000000e0
#define PIC32_MAC_STAT_MASK                          0x00ff00e0
  #define PIC32_MAC_STAT_RXBUSY                    0x00000020
  #define PIC32_MAC_STAT_TXBUSY                    0x00000040
  #define PIC32_MAC_STAT_BUSY                      0x00000080
  #define PIC32_MAC_STAT_BUFCNT(v)                 ((v) << 16)
  #define PIC32_MAC_STAT_BUFCNT_SET(x, v)          do { (x) = (((x) & ~0xff0000) | ((v) << 16)); } while(0)
  #define PIC32_MAC_STAT_BUFCNT_GET(x)             (((x) >> 16) & 0xff)

#define PIC32_MAC_RXOVFLW_ADDR                       0x00000100
#define PIC32_MAC_RXOVFLW_MASK                       0x0000ffff
  #define PIC32_MAC_RXOVFLW_CNT(v)                 ((v) << 0)
  #define PIC32_MAC_RXOVFLW_CNT_SET(x, v)          do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_RXOVFLW_CNT_GET(x)             (((x) >> 0) & 0xffff)

#define PIC32_MAC_FRMTXOK_ADDR                       0x00000110
#define PIC32_MAC_FRMTXOK_MASK                       0x0000ffff
  #define PIC32_MAC_FRMTXOK_CNT(v)                 ((v) << 0)
  #define PIC32_MAC_FRMTXOK_CNT_SET(x, v)          do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_FRMTXOK_CNT_GET(x)             (((x) >> 0) & 0xffff)

#define PIC32_MAC_SCOLFRM_ADDR                       0x00000120
#define PIC32_MAC_SCOLFRM_MASK                       0x0000ffff
  #define PIC32_MAC_SCOLFRM_CNT(v)                 ((v) << 0)
  #define PIC32_MAC_SCOLFRM_CNT_SET(x, v)          do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_SCOLFRM_CNT_GET(x)             (((x) >> 0) & 0xffff)

#define PIC32_MAC_MCOLFRM_ADDR                       0x00000130
#define PIC32_MAC_MCOLFRM_MASK                       0x0000ffff
  #define PIC32_MAC_MCOLFRM_CNT(v)                 ((v) << 0)
  #define PIC32_MAC_MCOLFRM_CNT_SET(x, v)          do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_MCOLFRM_CNT_GET(x)             (((x) >> 0) & 0xffff)

#define PIC32_MAC_FRMRXOK_ADDR                       0x00000140
#define PIC32_MAC_FRMRXOK_MASK                       0x0000ffff
  #define PIC32_MAC_FRMRXOK_CNT(v)                 ((v) << 0)
  #define PIC32_MAC_FRMRXOK_CNT_SET(x, v)          do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_FRMRXOK_CNT_GET(x)             (((x) >> 0) & 0xffff)

#define PIC32_MAC_FCSERR_ADDR                        0x00000150
#define PIC32_MAC_FCSERR_MASK                        0x0000ffff
  #define PIC32_MAC_FCSERR_CNT(v)                  ((v) << 0)
  #define PIC32_MAC_FCSERR_CNT_SET(x, v)           do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_FCSERR_CNT_GET(x)              (((x) >> 0) & 0xffff)

#define PIC32_MAC_ALGNERR_ADDR                       0x00000160
#define PIC32_MAC_ALGNERR_MASK                       0x0000ffff
  #define PIC32_MAC_ALGNERR_CNT(v)                 ((v) << 0)
  #define PIC32_MAC_ALGNERR_CNT_SET(x, v)          do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_ALGNERR_CNT_GET(x)             (((x) >> 0) & 0xffff)

#define PIC32_MAC_CFG1_ADDR                          0x00000200
#define PIC32_MAC_CFG1_MASK                          0x0000cf1f
  #define PIC32_MAC_CFG1_RXENABLE                  0x00000001
  #define PIC32_MAC_CFG1_PASSALL                   0x00000002
  #define PIC32_MAC_CFG1_RXPAUSE                   0x00000004
  #define PIC32_MAC_CFG1_TXPAUSE                   0x00000008
  #define PIC32_MAC_CFG1_LOOPBACK                  0x00000010
  #define PIC32_MAC_CFG1_RESETTFUN                 0x00000100
  #define PIC32_MAC_CFG1_RESETTMCS                 0x00000200
  #define PIC32_MAC_CFG1_RESETRFUN                 0x00000400
  #define PIC32_MAC_CFG1_RESETRMCS                 0x00000800
/** This resets random generator used for TX @multiple */
  #define PIC32_MAC_CFG1_SIMRESET                  0x00004000
  #define PIC32_MAC_CFG1_SOFTRESET                 0x00008000

#define PIC32_MAC_CFG2_ADDR                          0x00000210
#define PIC32_MAC_CFG2_MASK                          0x000073ff
  #define PIC32_MAC_CFG2_FULLDPLX                  0x00000001
  #define PIC32_MAC_CFG2_LENGTHCK                  0x00000002
  #define PIC32_MAC_CFG2_HUGEFRM                   0x00000004
  #define PIC32_MAC_CFG2_DELAYCRC                  0x00000008
  #define PIC32_MAC_CFG2_CRCENABLE                 0x00000010
/** Requires crcenable = 1 @multiple */
  #define PIC32_MAC_CFG2_PADENABLE                 0x00000020
/** Requires padenable = 1 @multiple */
  #define PIC32_MAC_CFG2_VLANPAD                   0x00000040
/** Requires vlanpad = 0, padenable = 1 @multiple */
  #define PIC32_MAC_CFG2_AUTOPAD                   0x00000080
  #define PIC32_MAC_CFG2_PUREPRE                   0x00000100
  #define PIC32_MAC_CFG2_LONGPRE                   0x00000200
  #define PIC32_MAC_CFG2_NOBKOFF                   0x00001000
  #define PIC32_MAC_CFG2_BPNOBKOFF                 0x00002000
  #define PIC32_MAC_CFG2_EXCESSDFR                 0x00004000

#define PIC32_MAC_IGPT_ADDR                          0x00000220
#define PIC32_MAC_IGPT_MASK                          0x0000007f
  #define PIC32_MAC_IGPT_B2BIPKTGP(v)              ((v) << 0)
  #define PIC32_MAC_IGPT_B2BIPKTGP_SET(x, v)       do { (x) = (((x) & ~0x7f) | ((v) << 0)); } while(0)
  #define PIC32_MAC_IGPT_B2BIPKTGP_GET(x)          (((x) >> 0) & 0x7f)

#define PIC32_MAC_IGPR_ADDR                          0x00000230
#define PIC32_MAC_IGPR_MASK                          0x00007f7f
  #define PIC32_MAC_IGPR_NB2BIPKTGP2(v)            ((v) << 0)
  #define PIC32_MAC_IGPR_NB2BIPKTGP2_SET(x, v)     do { (x) = (((x) & ~0x7f) | ((v) << 0)); } while(0)
  #define PIC32_MAC_IGPR_NB2BIPKTGP2_GET(x)        (((x) >> 0) & 0x7f)
  #define PIC32_MAC_IGPR_NB2BIPKTGP1(v)            ((v) << 8)
  #define PIC32_MAC_IGPR_NB2BIPKTGP1_SET(x, v)     do { (x) = (((x) & ~0x7f00) | ((v) << 8)); } while(0)
  #define PIC32_MAC_IGPR_NB2BIPKTGP1_GET(x)        (((x) >> 8) & 0x7f)

#define PIC32_MAC_CLRT_ADDR                          0x00000240
#define PIC32_MAC_CLRT_MASK                          0x00003f0f
  #define PIC32_MAC_CLRT_RETX(v)                   ((v) << 0)
  #define PIC32_MAC_CLRT_RETX_SET(x, v)            do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define PIC32_MAC_CLRT_RETX_GET(x)               (((x) >> 0) & 0xf)
  #define PIC32_MAC_CLRT_CWINDOW(v)                ((v) << 8)
  #define PIC32_MAC_CLRT_CWINDOW_SET(x, v)         do { (x) = (((x) & ~0x3f00) | ((v) << 8)); } while(0)
  #define PIC32_MAC_CLRT_CWINDOW_GET(x)            (((x) >> 8) & 0x3f)

#define PIC32_MAC_MAXF_ADDR                          0x00000250
#define PIC32_MAC_MAXF_MASK                          0x0000ffff
  #define PIC32_MAC_MAXF_MACMAXF(v)                ((v) << 0)
  #define PIC32_MAC_MAXF_MACMAXF_SET(x, v)         do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_MAXF_MACMAXF_GET(x)            (((x) >> 0) & 0xffff)

#define PIC32_MAC_SUPP_ADDR                          0x00000260
#define PIC32_MAC_SUPP_MASK                          0x00000900
  #define PIC32_MAC_SUPP_SPEEDRMII(v)              ((PIC32_MAC_SUPP_SPEEDRMII_##v) << 8)
  #define PIC32_MAC_SUPP_SPEEDRMII_SET(x, v)       do { (x) = (((x) & ~0x100) | ((PIC32_MAC_SUPP_SPEEDRMII_##v) << 8)); } while(0)
  #define PIC32_MAC_SUPP_SPEEDRMII_SETVAL(x, v)    do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define PIC32_MAC_SUPP_SPEEDRMII_GET(x)          (((x) >> 8) & 0x1)
    #define PIC32_MAC_SUPP_SPEEDRMII_10MBPS          0x00000000
    #define PIC32_MAC_SUPP_SPEEDRMII_100MBPS         0x00000001
  #define PIC32_MAC_SUPP_RESETRMII                 0x00000800

#define PIC32_MAC_TEST_ADDR                          0x00000270
#define PIC32_MAC_TEST_MASK                          0x00000007
  #define PIC32_MAC_TEST_SHRTQNTA                  0x00000001
  #define PIC32_MAC_TEST_TESTPAUSE                 0x00000002
  #define PIC32_MAC_TEST_TESTBP                    0x00000004

#define PIC32_MAC_SMI_CFG_ADDR                       0x00000280
#define PIC32_MAC_SMI_CFG_MASK                       0x0000803f
  #define PIC32_MAC_SMI_CFG_SCANINC                0x00000001
  #define PIC32_MAC_SMI_CFG_NOPRE                  0x00000002
  #define PIC32_MAC_SMI_CFG_CLKSEL(v)              ((PIC32_MAC_SMI_CFG_CLKSEL_##v) << 2)
  #define PIC32_MAC_SMI_CFG_CLKSEL_SET(x, v)       do { (x) = (((x) & ~0x3c) | ((PIC32_MAC_SMI_CFG_CLKSEL_##v) << 2)); } while(0)
  #define PIC32_MAC_SMI_CFG_CLKSEL_SETVAL(x, v)    do { (x) = (((x) & ~0x3c) | ((v) << 2)); } while(0)
  #define PIC32_MAC_SMI_CFG_CLKSEL_GET(x)          (((x) >> 2) & 0xf)
    #define PIC32_MAC_SMI_CFG_CLKSEL_4               0x00000000
    #define PIC32_MAC_SMI_CFG_CLKSEL_6               0x00000002
    #define PIC32_MAC_SMI_CFG_CLKSEL_8               0x00000003
    #define PIC32_MAC_SMI_CFG_CLKSEL_10              0x00000004
    #define PIC32_MAC_SMI_CFG_CLKSEL_14              0x00000005
    #define PIC32_MAC_SMI_CFG_CLKSEL_20              0x00000006
    #define PIC32_MAC_SMI_CFG_CLKSEL_28              0x00000007
    #define PIC32_MAC_SMI_CFG_CLKSEL_40              0x00000008
    #define PIC32_MAC_SMI_CFG_CLKSEL_48              0x00000009
    #define PIC32_MAC_SMI_CFG_CLKSEL_50              0x0000000a
  #define PIC32_MAC_SMI_CFG_RESETMGMT              0x00008000

#define PIC32_MAC_SMI_CMD_ADDR                       0x00000290
#define PIC32_MAC_SMI_CMD_MASK                       0x00000003
  #define PIC32_MAC_SMI_CMD_READ                   0x00000001
  #define PIC32_MAC_SMI_CMD_SCAN                   0x00000002

#define PIC32_MAC_SMI_ADR_ADDR                       0x000002a0
#define PIC32_MAC_SMI_ADR_MASK                       0x001f001f
  #define PIC32_MAC_SMI_ADR_REGADDR(v)             ((v) << 0)
  #define PIC32_MAC_SMI_ADR_REGADDR_SET(x, v)      do { (x) = (((x) & ~0x1f) | ((v) << 0)); } while(0)
  #define PIC32_MAC_SMI_ADR_REGADDR_GET(x)         (((x) >> 0) & 0x1f)
  #define PIC32_MAC_SMI_ADR_PHYADDR(v)             ((v) << 16)
  #define PIC32_MAC_SMI_ADR_PHYADDR_SET(x, v)      do { (x) = (((x) & ~0x1f0000) | ((v) << 16)); } while(0)
  #define PIC32_MAC_SMI_ADR_PHYADDR_GET(x)         (((x) >> 16) & 0x1f)

#define PIC32_MAC_SMI_WTD_ADDR                       0x000002b0
#define PIC32_MAC_SMI_WTD_MASK                       0x0000ffff
  #define PIC32_MAC_SMI_WTD_MWTD(v)                ((v) << 0)
  #define PIC32_MAC_SMI_WTD_MWTD_SET(x, v)         do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_SMI_WTD_MWTD_GET(x)            (((x) >> 0) & 0xffff)

#define PIC32_MAC_SMI_RDD_ADDR                       0x000002c0
#define PIC32_MAC_SMI_RDD_MASK                       0x0000ffff
  #define PIC32_MAC_SMI_RDD_MRDD(v)                ((v) << 0)
  #define PIC32_MAC_SMI_RDD_MRDD_SET(x, v)         do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define PIC32_MAC_SMI_RDD_MRDD_GET(x)            (((x) >> 0) & 0xffff)

#define PIC32_MAC_SMI_IND_ADDR                       0x000002d0
#define PIC32_MAC_SMI_IND_MASK                       0x0000000f
  #define PIC32_MAC_SMI_IND_MIIMBUSY               0x00000001
  #define PIC32_MAC_SMI_IND_SCAN                   0x00000002
  #define PIC32_MAC_SMI_IND_NOTVALID               0x00000004
  #define PIC32_MAC_SMI_IND_LINKFAIL               0x00000008

#define PIC32_MAC_SA_ADDR(ridx)                      (0x00000300 + (ridx) * 16)
#define PIC32_MAC_SA_COUNT                           3
#define PIC32_MAC_SA_MASK                            0x0000ffff
  #define PIC32_MAC_SA_STNADDR_COUNT               2
  #define PIC32_MAC_SA_STNADDR(fidx, v)            ((v) << ((fidx) * 8 + 0))
  #define PIC32_MAC_SA_STNADDR_SET(fidx, x, v)     do { (x) = (((x) & ~(0xff << ((fidx) * 8))) | ((v) << ((fidx) * 8 + 0))); } while(0)
  #define PIC32_MAC_SA_STNADDR_GET(fidx, x)        (((x) >> ((fidx) * 8 + 0)) & 0xff)

#endif

