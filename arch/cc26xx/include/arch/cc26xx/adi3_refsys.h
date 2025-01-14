/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs -I defs/cc26xx/adi3_refsys.bf cdefs_use_reg_comment=0        \
     cdefs_use_field_comment=0 cdefs_use_field_get=2 cdefs_use_field_set=2     \
     cdefs_use_field_shift=1 cdefs_use_field_shifted_mask=1                    \
     cdefs_use_field_shifter=0 cdefs_use_reg_comment=0                         \
     cdefs_use_value_comment=0 doc_field_doc_column=0                          \
     doc_field_longname_column=0 doc_lsb_on_left=0 doc_reg_address_column=0    \
     doc_reg_direction_column=0 doc_reg_doc_column=0 doc_reg_longname_column=0 \
     doc_split_width=0
*/

#ifndef _CC26XX_ADI_3_REFSYS_BFGEN_DEFS_
#define _CC26XX_ADI_3_REFSYS_BFGEN_DEFS_

#define CC26XX_ADI_3_REFSYS_SPARE0_ADDR              0x00000001
  #define CC26XX_ADI_3_REFSYS_SPARE0_SPARE0        0xff
  #define CC26XX_ADI_3_REFSYS_SPARE0_SPARE0_SHIFT  0
  #define CC26XX_ADI_3_REFSYS_SPARE0_SPARE0_SET(x, v) do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define CC26XX_ADI_3_REFSYS_SPARE0_SPARE0_GET(x) (((x) >> 0) & 0xff)

#define CC26XX_ADI_3_REFSYS_REFSYSCTL0_ADDR          0x00000002
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL0_TESTCTL   0xff
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL0_TESTCTL_SHIFT 0
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL0_TESTCTL_SET(x, v) do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL0_TESTCTL_GET(x) (((x) >> 0) & 0xff)

#define CC26XX_ADI_3_REFSYS_REFSYSCTL1_ADDR          0x00000003
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL1_TESTCTL   0x03
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL1_TESTCTL_SHIFT 0
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL1_TESTCTL_SET(x, v) do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL1_TESTCTL_GET(x) (((x) >> 0) & 0x3)
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL1_BATMON_COMP_TEST_EN 0x04
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL1_BATMON_COMP_TEST_EN_SHIFT 2
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD 0xf8
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SHIFT 3
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_SET(x, v) do { (x) = (((x) & ~0xf8) | ((v) << 3)); } while(0)
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL1_TRIM_VDDS_BOD_GET(x) (((x) >> 3) & 0x1f)

#define CC26XX_ADI_3_REFSYS_REFSYSCTL2_ADDR          0x00000004
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL2_TRIM_TSENSE 0x03
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL2_TRIM_TSENSE_SHIFT 0
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL2_TRIM_TSENSE_SET(x, v) do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL2_TRIM_TSENSE_GET(x) (((x) >> 0) & 0x3)
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL2_RESERVED0 0x0c
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL2_RESERVED0_SHIFT 2
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL2_RESERVED0_SET(x, v) do { (x) = (((x) & ~0xc) | ((v) << 2)); } while(0)
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL2_RESERVED0_GET(x) (((x) >> 2) & 0x3)
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL2_TRIM_VREF 0xf0
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL2_TRIM_VREF_SHIFT 4
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL2_TRIM_VREF_SET(x, v) do { (x) = (((x) & ~0xf0) | ((v) << 4)); } while(0)
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL2_TRIM_VREF_GET(x) (((x) >> 4) & 0xf)

#define CC26XX_ADI_3_REFSYS_REFSYSCTL3_ADDR          0x00000005
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL3_TRIM_VBG  0x3f
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL3_TRIM_VBG_SHIFT 0
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL3_TRIM_VBG_SET(x, v) do { (x) = (((x) & ~0x3f) | ((v) << 0)); } while(0)
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL3_TRIM_VBG_GET(x) (((x) >> 0) & 0x3f)
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL3_VTEMP_EN  0x40
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL3_VTEMP_EN_SHIFT 6
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL3_BOD_BG_TRIM_EN 0x80
  #define CC26XX_ADI_3_REFSYS_REFSYSCTL3_BOD_BG_TRIM_EN_SHIFT 7

#define CC26XX_ADI_3_REFSYS_DCDCCTL0_ADDR            0x00000006
  #define CC26XX_ADI_3_REFSYS_DCDCCTL0_VDDR_TRIM   0x1f
  #define CC26XX_ADI_3_REFSYS_DCDCCTL0_VDDR_TRIM_SHIFT 0
  #define CC26XX_ADI_3_REFSYS_DCDCCTL0_VDDR_TRIM_SET(x, v) do { (x) = (((x) & ~0x1f) | ((v) << 0)); } while(0)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL0_VDDR_TRIM_GET(x) (((x) >> 0) & 0x1f)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL0_GLDO_ISRC   0xe0
  #define CC26XX_ADI_3_REFSYS_DCDCCTL0_GLDO_ISRC_SHIFT 5
  #define CC26XX_ADI_3_REFSYS_DCDCCTL0_GLDO_ISRC_SET(x, v) do { (x) = (((x) & ~0xe0) | ((v) << 5)); } while(0)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL0_GLDO_ISRC_GET(x) (((x) >> 5) & 0x7)

#define CC26XX_ADI_3_REFSYS_DCDCCTL1_ADDR            0x00000007
  #define CC26XX_ADI_3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP 0x1f
  #define CC26XX_ADI_3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_SHIFT 0
  #define CC26XX_ADI_3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_SET(x, v) do { (x) = (((x) & ~0x1f) | ((v) << 0)); } while(0)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL1_VDDR_TRIM_SLEEP_GET(x) (((x) >> 0) & 0x1f)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL1_VDDR_OK_HYST 0x20
  #define CC26XX_ADI_3_REFSYS_DCDCCTL1_VDDR_OK_HYST_SHIFT 5
  #define CC26XX_ADI_3_REFSYS_DCDCCTL1_IPTAT_TRIM  0xc0
  #define CC26XX_ADI_3_REFSYS_DCDCCTL1_IPTAT_TRIM_SHIFT 6
  #define CC26XX_ADI_3_REFSYS_DCDCCTL1_IPTAT_TRIM_SET(x, v) do { (x) = (((x) & ~0xc0) | ((v) << 6)); } while(0)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL1_IPTAT_TRIM_GET(x) (((x) >> 6) & 0x3)

#define CC26XX_ADI_3_REFSYS_DCDCCTL2_ADDR            0x00000008
  #define CC26XX_ADI_3_REFSYS_DCDCCTL2_TESTSEL     0x0f
  #define CC26XX_ADI_3_REFSYS_DCDCCTL2_TESTSEL_SHIFT 0
  #define CC26XX_ADI_3_REFSYS_DCDCCTL2_TESTSEL_SET(x, v) do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL2_TESTSEL_GET(x) (((x) >> 0) & 0xf)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL2_BIAS_DIS    0x10
  #define CC26XX_ADI_3_REFSYS_DCDCCTL2_BIAS_DIS_SHIFT 4
  #define CC26XX_ADI_3_REFSYS_DCDCCTL2_TEST_VDDR   0x20
  #define CC26XX_ADI_3_REFSYS_DCDCCTL2_TEST_VDDR_SHIFT 5
  #define CC26XX_ADI_3_REFSYS_DCDCCTL2_TURNON_EA_SW 0x40
  #define CC26XX_ADI_3_REFSYS_DCDCCTL2_TURNON_EA_SW_SHIFT 6
  #define CC26XX_ADI_3_REFSYS_DCDCCTL2_RESERVED7   0x80
  #define CC26XX_ADI_3_REFSYS_DCDCCTL2_RESERVED7_SHIFT 7

#define CC26XX_ADI_3_REFSYS_DCDCCTL3_ADDR            0x00000009
  #define CC26XX_ADI_3_REFSYS_DCDCCTL3_RESERVED0   0xff
  #define CC26XX_ADI_3_REFSYS_DCDCCTL3_RESERVED0_SHIFT 0
  #define CC26XX_ADI_3_REFSYS_DCDCCTL3_RESERVED0_SET(x, v) do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL3_RESERVED0_GET(x) (((x) >> 0) & 0xff)

#define CC26XX_ADI_3_REFSYS_DCDCCTL4_ADDR            0x0000000a
  #define CC26XX_ADI_3_REFSYS_DCDCCTL4_HIGH_EN_SEL 0x07
  #define CC26XX_ADI_3_REFSYS_DCDCCTL4_HIGH_EN_SEL_SHIFT 0
  #define CC26XX_ADI_3_REFSYS_DCDCCTL4_HIGH_EN_SEL_SET(x, v) do { (x) = (((x) & ~0x7) | ((v) << 0)); } while(0)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL4_HIGH_EN_SEL_GET(x) (((x) >> 0) & 0x7)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL4_LOW_EN_SEL  0x38
  #define CC26XX_ADI_3_REFSYS_DCDCCTL4_LOW_EN_SEL_SHIFT 3
  #define CC26XX_ADI_3_REFSYS_DCDCCTL4_LOW_EN_SEL_SET(x, v) do { (x) = (((x) & ~0x38) | ((v) << 3)); } while(0)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL4_LOW_EN_SEL_GET(x) (((x) >> 3) & 0x7)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL4_DEADTIME_TRIM 0xc0
  #define CC26XX_ADI_3_REFSYS_DCDCCTL4_DEADTIME_TRIM_SHIFT 6
  #define CC26XX_ADI_3_REFSYS_DCDCCTL4_DEADTIME_TRIM_SET(x, v) do { (x) = (((x) & ~0xc0) | ((v) << 6)); } while(0)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL4_DEADTIME_TRIM_GET(x) (((x) >> 6) & 0x3)

#define CC26XX_ADI_3_REFSYS_DCDCCTL5_ADDR            0x0000000b
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_IPEAK       0x07
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_IPEAK_SHIFT 0
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_IPEAK_SET(x, v) do { (x) = (((x) & ~0x7) | ((v) << 0)); } while(0)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_IPEAK_GET(x) (((x) >> 0) & 0x7)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_DITHER_EN   0x08
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_DITHER_EN_SHIFT 3
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_TESTP       0x10
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_TESTP_SHIFT 4
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_TESTN       0x20
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_TESTN_SHIFT 5
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_RESERVED6   0xc0
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_RESERVED6_SHIFT 6
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_RESERVED6_SET(x, v) do { (x) = (((x) & ~0xc0) | ((v) << 6)); } while(0)
  #define CC26XX_ADI_3_REFSYS_DCDCCTL5_RESERVED6_GET(x) (((x) >> 6) & 0x3)

#endif

