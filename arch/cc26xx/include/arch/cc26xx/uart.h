/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs -I defs/cc26xx/uart.bf cdefs_use_reg_comment=0               \
     cdefs_use_field_comment=0 cdefs_use_field_get=2 cdefs_use_field_set=2     \
     cdefs_use_field_shift=1 cdefs_use_field_shifted_mask=1                    \
     cdefs_use_field_shifter=0 cdefs_use_reg_comment=0                         \
     cdefs_use_value_comment=0 doc_field_doc_column=0                          \
     doc_field_longname_column=0 doc_lsb_on_left=0 doc_reg_address_column=0    \
     doc_reg_direction_column=0 doc_reg_doc_column=0 doc_reg_longname_column=0 \
     doc_split_width=0
*/

#ifndef _CC26XX_UART_BFGEN_DEFS_
#define _CC26XX_UART_BFGEN_DEFS_

#define CC26XX_UART_DR_ADDR                          0x00000000
  #define CC26XX_UART_DR_DATA                      0x000000ff
  #define CC26XX_UART_DR_DATA_SHIFT                0
  #define CC26XX_UART_DR_DATA_SET(x, v)            do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define CC26XX_UART_DR_DATA_GET(x)               (((x) >> 0) & 0xff)
  #define CC26XX_UART_DR_FE                        0x00000100
  #define CC26XX_UART_DR_FE_SHIFT                  8
  #define CC26XX_UART_DR_PE                        0x00000200
  #define CC26XX_UART_DR_PE_SHIFT                  9
  #define CC26XX_UART_DR_BE                        0x00000400
  #define CC26XX_UART_DR_BE_SHIFT                  10
  #define CC26XX_UART_DR_OE                        0x00000800
  #define CC26XX_UART_DR_OE_SHIFT                  11

#define CC26XX_UART_RSR_ADDR                         0x00000004
  #define CC26XX_UART_RSR_FE                       0x00000001
  #define CC26XX_UART_RSR_FE_SHIFT                 0
  #define CC26XX_UART_RSR_PE                       0x00000002
  #define CC26XX_UART_RSR_PE_SHIFT                 1
  #define CC26XX_UART_RSR_BE                       0x00000004
  #define CC26XX_UART_RSR_BE_SHIFT                 2
  #define CC26XX_UART_RSR_OE                       0x00000008
  #define CC26XX_UART_RSR_OE_SHIFT                 3

#define CC26XX_UART_ECR_ADDR                         0x00000004
  #define CC26XX_UART_ECR_FE                       0x00000001
  #define CC26XX_UART_ECR_FE_SHIFT                 0
  #define CC26XX_UART_ECR_PE                       0x00000002
  #define CC26XX_UART_ECR_PE_SHIFT                 1
  #define CC26XX_UART_ECR_BE                       0x00000004
  #define CC26XX_UART_ECR_BE_SHIFT                 2
  #define CC26XX_UART_ECR_OE                       0x00000008
  #define CC26XX_UART_ECR_OE_SHIFT                 3

#define CC26XX_UART_FR_ADDR                          0x00000018
  #define CC26XX_UART_FR_CTS                       0x00000001
  #define CC26XX_UART_FR_CTS_SHIFT                 0
  #define CC26XX_UART_FR_BUSY                      0x00000008
  #define CC26XX_UART_FR_BUSY_SHIFT                3
  #define CC26XX_UART_FR_RXFE                      0x00000010
  #define CC26XX_UART_FR_RXFE_SHIFT                4
  #define CC26XX_UART_FR_TXFF                      0x00000020
  #define CC26XX_UART_FR_TXFF_SHIFT                5
  #define CC26XX_UART_FR_RXFF                      0x00000040
  #define CC26XX_UART_FR_RXFF_SHIFT                6
  #define CC26XX_UART_FR_TXFE                      0x00000080
  #define CC26XX_UART_FR_TXFE_SHIFT                7

#define CC26XX_UART_IBRD_ADDR                        0x00000024
  #define CC26XX_UART_IBRD_DIVINT                  0x0000ffff
  #define CC26XX_UART_IBRD_DIVINT_SHIFT            0
  #define CC26XX_UART_IBRD_DIVINT_SET(x, v)        do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define CC26XX_UART_IBRD_DIVINT_GET(x)           (((x) >> 0) & 0xffff)

#define CC26XX_UART_FBRD_ADDR                        0x00000028
  #define CC26XX_UART_FBRD_DIVFRAC                 0x0000003f
  #define CC26XX_UART_FBRD_DIVFRAC_SHIFT           0
  #define CC26XX_UART_FBRD_DIVFRAC_SET(x, v)       do { (x) = (((x) & ~0x3f) | ((v) << 0)); } while(0)
  #define CC26XX_UART_FBRD_DIVFRAC_GET(x)          (((x) >> 0) & 0x3f)

#define CC26XX_UART_LCRH_ADDR                        0x0000002c
  #define CC26XX_UART_LCRH_BRK                     0x00000001
  #define CC26XX_UART_LCRH_BRK_SHIFT               0
  #define CC26XX_UART_LCRH_PEN                     0x00000002
  #define CC26XX_UART_LCRH_PEN_SHIFT               1
  #define CC26XX_UART_LCRH_PEN_SET(x, v)           do { (x) = (((x) & ~0x2) | ((CC26XX_UART_LCRH_PEN_##v) << 1)); } while(0)
  #define CC26XX_UART_LCRH_PEN_GET(x)              (((x) >> 1) & 0x1)
    #define CC26XX_UART_LCRH_PEN_DIS                 0x00000000
    #define CC26XX_UART_LCRH_PEN_EN                  0x00000001
  #define CC26XX_UART_LCRH_EPS                     0x00000004
  #define CC26XX_UART_LCRH_EPS_SHIFT               2
  #define CC26XX_UART_LCRH_EPS_SET(x, v)           do { (x) = (((x) & ~0x4) | ((CC26XX_UART_LCRH_EPS_##v) << 2)); } while(0)
  #define CC26XX_UART_LCRH_EPS_GET(x)              (((x) >> 2) & 0x1)
    #define CC26XX_UART_LCRH_EPS_ODD                 0x00000000
    #define CC26XX_UART_LCRH_EPS_EVEN                0x00000001
  #define CC26XX_UART_LCRH_STP2                    0x00000008
  #define CC26XX_UART_LCRH_STP2_SHIFT              3
  #define CC26XX_UART_LCRH_FEN                     0x00000010
  #define CC26XX_UART_LCRH_FEN_SHIFT               4
  #define CC26XX_UART_LCRH_FEN_SET(x, v)           do { (x) = (((x) & ~0x10) | ((CC26XX_UART_LCRH_FEN_##v) << 4)); } while(0)
  #define CC26XX_UART_LCRH_FEN_GET(x)              (((x) >> 4) & 0x1)
    #define CC26XX_UART_LCRH_FEN_DIS                 0x00000000
    #define CC26XX_UART_LCRH_FEN_EN                  0x00000001
  #define CC26XX_UART_LCRH_WLEN                    0x00000060
  #define CC26XX_UART_LCRH_WLEN_SHIFT              5
  #define CC26XX_UART_LCRH_WLEN_SET(x, v)          do { (x) = (((x) & ~0x60) | ((CC26XX_UART_LCRH_WLEN_##v) << 5)); } while(0)
  #define CC26XX_UART_LCRH_WLEN_GET(x)             (((x) >> 5) & 0x3)
    #define CC26XX_UART_LCRH_WLEN_5                  0x00000000
    #define CC26XX_UART_LCRH_WLEN_6                  0x00000001
    #define CC26XX_UART_LCRH_WLEN_7                  0x00000002
    #define CC26XX_UART_LCRH_WLEN_8                  0x00000003
  #define CC26XX_UART_LCRH_SPS                     0x00000080
  #define CC26XX_UART_LCRH_SPS_SHIFT               7

#define CC26XX_UART_CTL_ADDR                         0x00000030
  #define CC26XX_UART_CTL_UARTEN                   0x00000001
  #define CC26XX_UART_CTL_UARTEN_SHIFT             0
  #define CC26XX_UART_CTL_UARTEN_SET(x, v)         do { (x) = (((x) & ~0x1) | ((CC26XX_UART_CTL_UARTEN_##v) << 0)); } while(0)
  #define CC26XX_UART_CTL_UARTEN_GET(x)            (((x) >> 0) & 0x1)
    #define CC26XX_UART_CTL_UARTEN_DIS               0x00000000
    #define CC26XX_UART_CTL_UARTEN_EN                0x00000001
  #define CC26XX_UART_CTL_LBE                      0x00000080
  #define CC26XX_UART_CTL_LBE_SHIFT                7
  #define CC26XX_UART_CTL_LBE_SET(x, v)            do { (x) = (((x) & ~0x80) | ((CC26XX_UART_CTL_LBE_##v) << 7)); } while(0)
  #define CC26XX_UART_CTL_LBE_GET(x)               (((x) >> 7) & 0x1)
    #define CC26XX_UART_CTL_LBE_DIS                  0x00000000
    #define CC26XX_UART_CTL_LBE_EN                   0x00000001
  #define CC26XX_UART_CTL_TXE                      0x00000100
  #define CC26XX_UART_CTL_TXE_SHIFT                8
  #define CC26XX_UART_CTL_TXE_SET(x, v)            do { (x) = (((x) & ~0x100) | ((CC26XX_UART_CTL_TXE_##v) << 8)); } while(0)
  #define CC26XX_UART_CTL_TXE_GET(x)               (((x) >> 8) & 0x1)
    #define CC26XX_UART_CTL_TXE_DIS                  0x00000000
    #define CC26XX_UART_CTL_TXE_EN                   0x00000001
  #define CC26XX_UART_CTL_RXE                      0x00000200
  #define CC26XX_UART_CTL_RXE_SHIFT                9
  #define CC26XX_UART_CTL_RXE_SET(x, v)            do { (x) = (((x) & ~0x200) | ((CC26XX_UART_CTL_RXE_##v) << 9)); } while(0)
  #define CC26XX_UART_CTL_RXE_GET(x)               (((x) >> 9) & 0x1)
    #define CC26XX_UART_CTL_RXE_DIS                  0x00000000
    #define CC26XX_UART_CTL_RXE_EN                   0x00000001
  #define CC26XX_UART_CTL_RTS                      0x00000800
  #define CC26XX_UART_CTL_RTS_SHIFT                11
  #define CC26XX_UART_CTL_RTSEN                    0x00004000
  #define CC26XX_UART_CTL_RTSEN_SHIFT              14
  #define CC26XX_UART_CTL_RTSEN_SET(x, v)          do { (x) = (((x) & ~0x4000) | ((CC26XX_UART_CTL_RTSEN_##v) << 14)); } while(0)
  #define CC26XX_UART_CTL_RTSEN_GET(x)             (((x) >> 14) & 0x1)
    #define CC26XX_UART_CTL_RTSEN_DIS                0x00000000
    #define CC26XX_UART_CTL_RTSEN_EN                 0x00000001
  #define CC26XX_UART_CTL_CTSEN                    0x00008000
  #define CC26XX_UART_CTL_CTSEN_SHIFT              15
  #define CC26XX_UART_CTL_CTSEN_SET(x, v)          do { (x) = (((x) & ~0x8000) | ((CC26XX_UART_CTL_CTSEN_##v) << 15)); } while(0)
  #define CC26XX_UART_CTL_CTSEN_GET(x)             (((x) >> 15) & 0x1)
    #define CC26XX_UART_CTL_CTSEN_DIS                0x00000000
    #define CC26XX_UART_CTL_CTSEN_EN                 0x00000001

#define CC26XX_UART_IFLS_ADDR                        0x00000034
  #define CC26XX_UART_IFLS_TXSEL                   0x00000007
  #define CC26XX_UART_IFLS_TXSEL_SHIFT             0
  #define CC26XX_UART_IFLS_TXSEL_SET(x, v)         do { (x) = (((x) & ~0x7) | ((CC26XX_UART_IFLS_TXSEL_##v) << 0)); } while(0)
  #define CC26XX_UART_IFLS_TXSEL_GET(x)            (((x) >> 0) & 0x7)
    #define CC26XX_UART_IFLS_TXSEL_1_8               0x00000000
    #define CC26XX_UART_IFLS_TXSEL_2_8               0x00000001
    #define CC26XX_UART_IFLS_TXSEL_4_8               0x00000002
    #define CC26XX_UART_IFLS_TXSEL_6_8               0x00000003
    #define CC26XX_UART_IFLS_TXSEL_7_8               0x00000004
  #define CC26XX_UART_IFLS_RXSEL                   0x00000038
  #define CC26XX_UART_IFLS_RXSEL_SHIFT             3
  #define CC26XX_UART_IFLS_RXSEL_SET(x, v)         do { (x) = (((x) & ~0x38) | ((CC26XX_UART_IFLS_RXSEL_##v) << 3)); } while(0)
  #define CC26XX_UART_IFLS_RXSEL_GET(x)            (((x) >> 3) & 0x7)
    #define CC26XX_UART_IFLS_RXSEL_1_8               0x00000000
    #define CC26XX_UART_IFLS_RXSEL_2_8               0x00000001
    #define CC26XX_UART_IFLS_RXSEL_4_8               0x00000002
    #define CC26XX_UART_IFLS_RXSEL_6_8               0x00000003
    #define CC26XX_UART_IFLS_RXSEL_7_8               0x00000004

#define CC26XX_UART_IMSC_ADDR                        0x00000038
  #define CC26XX_UART_IMSC_CTSMIM                  0x00000002
  #define CC26XX_UART_IMSC_CTSMIM_SHIFT            1
  #define CC26XX_UART_IMSC_RXIM                    0x00000010
  #define CC26XX_UART_IMSC_RXIM_SHIFT              4
  #define CC26XX_UART_IMSC_TXIM                    0x00000020
  #define CC26XX_UART_IMSC_TXIM_SHIFT              5
  #define CC26XX_UART_IMSC_RTIM                    0x00000040
  #define CC26XX_UART_IMSC_RTIM_SHIFT              6
  #define CC26XX_UART_IMSC_FEIM                    0x00000080
  #define CC26XX_UART_IMSC_FEIM_SHIFT              7
  #define CC26XX_UART_IMSC_PEIM                    0x00000100
  #define CC26XX_UART_IMSC_PEIM_SHIFT              8
  #define CC26XX_UART_IMSC_BEIM                    0x00000200
  #define CC26XX_UART_IMSC_BEIM_SHIFT              9
  #define CC26XX_UART_IMSC_OEIM                    0x00000400
  #define CC26XX_UART_IMSC_OEIM_SHIFT              10

#define CC26XX_UART_RIS_ADDR                         0x0000003c
  #define CC26XX_UART_RIS_CTSRMIS                  0x00000002
  #define CC26XX_UART_RIS_CTSRMIS_SHIFT            1
  #define CC26XX_UART_RIS_RXRIS                    0x00000010
  #define CC26XX_UART_RIS_RXRIS_SHIFT              4
  #define CC26XX_UART_RIS_TXRIS                    0x00000020
  #define CC26XX_UART_RIS_TXRIS_SHIFT              5
  #define CC26XX_UART_RIS_RTRIS                    0x00000040
  #define CC26XX_UART_RIS_RTRIS_SHIFT              6
  #define CC26XX_UART_RIS_FERIS                    0x00000080
  #define CC26XX_UART_RIS_FERIS_SHIFT              7
  #define CC26XX_UART_RIS_PERIS                    0x00000100
  #define CC26XX_UART_RIS_PERIS_SHIFT              8
  #define CC26XX_UART_RIS_BERIS                    0x00000200
  #define CC26XX_UART_RIS_BERIS_SHIFT              9
  #define CC26XX_UART_RIS_OERIS                    0x00000400
  #define CC26XX_UART_RIS_OERIS_SHIFT              10

#define CC26XX_UART_MIS_ADDR                         0x00000040
  #define CC26XX_UART_MIS_CTSMMIS                  0x00000002
  #define CC26XX_UART_MIS_CTSMMIS_SHIFT            1
  #define CC26XX_UART_MIS_RXMIS                    0x00000010
  #define CC26XX_UART_MIS_RXMIS_SHIFT              4
  #define CC26XX_UART_MIS_TXMIS                    0x00000020
  #define CC26XX_UART_MIS_TXMIS_SHIFT              5
  #define CC26XX_UART_MIS_RTMIS                    0x00000040
  #define CC26XX_UART_MIS_RTMIS_SHIFT              6
  #define CC26XX_UART_MIS_FEMIS                    0x00000080
  #define CC26XX_UART_MIS_FEMIS_SHIFT              7
  #define CC26XX_UART_MIS_PEMIS                    0x00000100
  #define CC26XX_UART_MIS_PEMIS_SHIFT              8
  #define CC26XX_UART_MIS_BEMIS                    0x00000200
  #define CC26XX_UART_MIS_BEMIS_SHIFT              9
  #define CC26XX_UART_MIS_OEMIS                    0x00000400
  #define CC26XX_UART_MIS_OEMIS_SHIFT              10

#define CC26XX_UART_ICR_ADDR                         0x00000044
  #define CC26XX_UART_ICR_CTSMIC                   0x00000002
  #define CC26XX_UART_ICR_CTSMIC_SHIFT             1
  #define CC26XX_UART_ICR_RXIC                     0x00000010
  #define CC26XX_UART_ICR_RXIC_SHIFT               4
  #define CC26XX_UART_ICR_TXIC                     0x00000020
  #define CC26XX_UART_ICR_TXIC_SHIFT               5
  #define CC26XX_UART_ICR_RTIC                     0x00000040
  #define CC26XX_UART_ICR_RTIC_SHIFT               6
  #define CC26XX_UART_ICR_FEIC                     0x00000080
  #define CC26XX_UART_ICR_FEIC_SHIFT               7
  #define CC26XX_UART_ICR_PEIC                     0x00000100
  #define CC26XX_UART_ICR_PEIC_SHIFT               8
  #define CC26XX_UART_ICR_BEIC                     0x00000200
  #define CC26XX_UART_ICR_BEIC_SHIFT               9
  #define CC26XX_UART_ICR_OEIC                     0x00000400
  #define CC26XX_UART_ICR_OEIC_SHIFT               10

#define CC26XX_UART_DMACTL_ADDR                      0x00000048
  #define CC26XX_UART_DMACTL_RXDMAE                0x00000001
  #define CC26XX_UART_DMACTL_RXDMAE_SHIFT          0
  #define CC26XX_UART_DMACTL_TXDMAE                0x00000002
  #define CC26XX_UART_DMACTL_TXDMAE_SHIFT          1
  #define CC26XX_UART_DMACTL_DMAONERR              0x00000004
  #define CC26XX_UART_DMACTL_DMAONERR_SHIFT        2

#define CC26XX_UART_PERIPHID0_ADDR                   0x00000fe0
  #define CC26XX_UART_PERIPHID0_PARTNUMBER0        0x000000ff
  #define CC26XX_UART_PERIPHID0_PARTNUMBER0_SHIFT  0
  #define CC26XX_UART_PERIPHID0_PARTNUMBER0_SET(x, v) do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define CC26XX_UART_PERIPHID0_PARTNUMBER0_GET(x) (((x) >> 0) & 0xff)

#define CC26XX_UART_PERIPHID1_ADDR                   0x00000fe4
  #define CC26XX_UART_PERIPHID1_PARTNUMBER1        0x0000000f
  #define CC26XX_UART_PERIPHID1_PARTNUMBER1_SHIFT  0
  #define CC26XX_UART_PERIPHID1_PARTNUMBER1_SET(x, v) do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define CC26XX_UART_PERIPHID1_PARTNUMBER1_GET(x) (((x) >> 0) & 0xf)
  #define CC26XX_UART_PERIPHID1_DESIGNER0          0x000000f0
  #define CC26XX_UART_PERIPHID1_DESIGNER0_SHIFT    4
  #define CC26XX_UART_PERIPHID1_DESIGNER0_SET(x, v) do { (x) = (((x) & ~0xf0) | ((v) << 4)); } while(0)
  #define CC26XX_UART_PERIPHID1_DESIGNER0_GET(x)   (((x) >> 4) & 0xf)

#define CC26XX_UART_PERIPHID2_ADDR                   0x00000fe8
  #define CC26XX_UART_PERIPHID2_DESIGNER1          0x0000000f
  #define CC26XX_UART_PERIPHID2_DESIGNER1_SHIFT    0
  #define CC26XX_UART_PERIPHID2_DESIGNER1_SET(x, v) do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define CC26XX_UART_PERIPHID2_DESIGNER1_GET(x)   (((x) >> 0) & 0xf)
  #define CC26XX_UART_PERIPHID2_REVISION           0x000000f0
  #define CC26XX_UART_PERIPHID2_REVISION_SHIFT     4
  #define CC26XX_UART_PERIPHID2_REVISION_SET(x, v) do { (x) = (((x) & ~0xf0) | ((v) << 4)); } while(0)
  #define CC26XX_UART_PERIPHID2_REVISION_GET(x)    (((x) >> 4) & 0xf)

#define CC26XX_UART_PERIPHID3_ADDR                   0x00000fec
  #define CC26XX_UART_PERIPHID3_CONFIGURATION      0x000000ff
  #define CC26XX_UART_PERIPHID3_CONFIGURATION_SHIFT 0
  #define CC26XX_UART_PERIPHID3_CONFIGURATION_SET(x, v) do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define CC26XX_UART_PERIPHID3_CONFIGURATION_GET(x) (((x) >> 0) & 0xff)

#define CC26XX_UART_PCELLID0_ADDR                    0x00000ff0
  #define CC26XX_UART_PCELLID0_PCELLID0            0x000000ff
  #define CC26XX_UART_PCELLID0_PCELLID0_SHIFT      0
  #define CC26XX_UART_PCELLID0_PCELLID0_SET(x, v)  do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define CC26XX_UART_PCELLID0_PCELLID0_GET(x)     (((x) >> 0) & 0xff)

#define CC26XX_UART_PCELLID1_ADDR                    0x00000ff4
  #define CC26XX_UART_PCELLID1_PCELLID1            0x000000ff
  #define CC26XX_UART_PCELLID1_PCELLID1_SHIFT      0
  #define CC26XX_UART_PCELLID1_PCELLID1_SET(x, v)  do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define CC26XX_UART_PCELLID1_PCELLID1_GET(x)     (((x) >> 0) & 0xff)

#define CC26XX_UART_PCELLID2_ADDR                    0x00000ff8
  #define CC26XX_UART_PCELLID2_PCELLID2            0x000000ff
  #define CC26XX_UART_PCELLID2_PCELLID2_SHIFT      0
  #define CC26XX_UART_PCELLID2_PCELLID2_SET(x, v)  do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define CC26XX_UART_PCELLID2_PCELLID2_GET(x)     (((x) >> 0) & 0xff)

#define CC26XX_UART_PCELLID3_ADDR                    0x00000ffc
  #define CC26XX_UART_PCELLID3_PCELLID3            0x000000ff
  #define CC26XX_UART_PCELLID3_PCELLID3_SHIFT      0
  #define CC26XX_UART_PCELLID3_PCELLID3_SET(x, v)  do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define CC26XX_UART_PCELLID3_PCELLID3_GET(x)     (((x) >> 0) & 0xff)

#endif

