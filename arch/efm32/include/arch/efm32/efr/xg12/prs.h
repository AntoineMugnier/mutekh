/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs cdefs_use_reg_mask=1 cdefs_use_field_setval=1                \
     cdefs_use_field_set=1
*/

#ifndef _EFR32_PRS_BFGEN_DEFS_
#define _EFR32_PRS_BFGEN_DEFS_

#define EFR32_PRS_SWPULSE_ADDR                       0x00000000
#define EFR32_PRS_SWPULSE_MASK                       0x00000fff
  #define EFR32_PRS_SWPULSE_CH0PULSE               0x00000001
  #define EFR32_PRS_SWPULSE_CH0PULSE_SET(x, v)     do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define EFR32_PRS_SWPULSE_CH1PULSE               0x00000002
  #define EFR32_PRS_SWPULSE_CH1PULSE_SET(x, v)     do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define EFR32_PRS_SWPULSE_CH2PULSE               0x00000004
  #define EFR32_PRS_SWPULSE_CH2PULSE_SET(x, v)     do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
  #define EFR32_PRS_SWPULSE_CH3PULSE               0x00000008
  #define EFR32_PRS_SWPULSE_CH3PULSE_SET(x, v)     do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
  #define EFR32_PRS_SWPULSE_CH4PULSE               0x00000010
  #define EFR32_PRS_SWPULSE_CH4PULSE_SET(x, v)     do { (x) = (((x) & ~0x10) | ((v) << 4)); } while(0)
  #define EFR32_PRS_SWPULSE_CH5PULSE               0x00000020
  #define EFR32_PRS_SWPULSE_CH5PULSE_SET(x, v)     do { (x) = (((x) & ~0x20) | ((v) << 5)); } while(0)
  #define EFR32_PRS_SWPULSE_CH6PULSE               0x00000040
  #define EFR32_PRS_SWPULSE_CH6PULSE_SET(x, v)     do { (x) = (((x) & ~0x40) | ((v) << 6)); } while(0)
  #define EFR32_PRS_SWPULSE_CH7PULSE               0x00000080
  #define EFR32_PRS_SWPULSE_CH7PULSE_SET(x, v)     do { (x) = (((x) & ~0x80) | ((v) << 7)); } while(0)
  #define EFR32_PRS_SWPULSE_CH8PULSE               0x00000100
  #define EFR32_PRS_SWPULSE_CH8PULSE_SET(x, v)     do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define EFR32_PRS_SWPULSE_CH9PULSE               0x00000200
  #define EFR32_PRS_SWPULSE_CH9PULSE_SET(x, v)     do { (x) = (((x) & ~0x200) | ((v) << 9)); } while(0)
  #define EFR32_PRS_SWPULSE_CH10PULSE              0x00000400
  #define EFR32_PRS_SWPULSE_CH10PULSE_SET(x, v)    do { (x) = (((x) & ~0x400) | ((v) << 10)); } while(0)
  #define EFR32_PRS_SWPULSE_CH11PULSE              0x00000800
  #define EFR32_PRS_SWPULSE_CH11PULSE_SET(x, v)    do { (x) = (((x) & ~0x800) | ((v) << 11)); } while(0)

#define EFR32_PRS_SWLEVEL_ADDR                       0x00000004
#define EFR32_PRS_SWLEVEL_MASK                       0x00000fff
  #define EFR32_PRS_SWLEVEL_CH0LEVEL               0x00000001
  #define EFR32_PRS_SWLEVEL_CH0LEVEL_SET(x, v)     do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define EFR32_PRS_SWLEVEL_CH1LEVEL               0x00000002
  #define EFR32_PRS_SWLEVEL_CH1LEVEL_SET(x, v)     do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define EFR32_PRS_SWLEVEL_CH2LEVEL               0x00000004
  #define EFR32_PRS_SWLEVEL_CH2LEVEL_SET(x, v)     do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
  #define EFR32_PRS_SWLEVEL_CH3LEVEL               0x00000008
  #define EFR32_PRS_SWLEVEL_CH3LEVEL_SET(x, v)     do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
  #define EFR32_PRS_SWLEVEL_CH4LEVEL               0x00000010
  #define EFR32_PRS_SWLEVEL_CH4LEVEL_SET(x, v)     do { (x) = (((x) & ~0x10) | ((v) << 4)); } while(0)
  #define EFR32_PRS_SWLEVEL_CH5LEVEL               0x00000020
  #define EFR32_PRS_SWLEVEL_CH5LEVEL_SET(x, v)     do { (x) = (((x) & ~0x20) | ((v) << 5)); } while(0)
  #define EFR32_PRS_SWLEVEL_CH6LEVEL               0x00000040
  #define EFR32_PRS_SWLEVEL_CH6LEVEL_SET(x, v)     do { (x) = (((x) & ~0x40) | ((v) << 6)); } while(0)
  #define EFR32_PRS_SWLEVEL_CH7LEVEL               0x00000080
  #define EFR32_PRS_SWLEVEL_CH7LEVEL_SET(x, v)     do { (x) = (((x) & ~0x80) | ((v) << 7)); } while(0)
  #define EFR32_PRS_SWLEVEL_CH8LEVEL               0x00000100
  #define EFR32_PRS_SWLEVEL_CH8LEVEL_SET(x, v)     do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define EFR32_PRS_SWLEVEL_CH9LEVEL               0x00000200
  #define EFR32_PRS_SWLEVEL_CH9LEVEL_SET(x, v)     do { (x) = (((x) & ~0x200) | ((v) << 9)); } while(0)
  #define EFR32_PRS_SWLEVEL_CH10LEVEL              0x00000400
  #define EFR32_PRS_SWLEVEL_CH10LEVEL_SET(x, v)    do { (x) = (((x) & ~0x400) | ((v) << 10)); } while(0)
  #define EFR32_PRS_SWLEVEL_CH11LEVEL              0x00000800
  #define EFR32_PRS_SWLEVEL_CH11LEVEL_SET(x, v)    do { (x) = (((x) & ~0x800) | ((v) << 11)); } while(0)

#define EFR32_PRS_ROUTEPEN_ADDR                      0x00000008
#define EFR32_PRS_ROUTEPEN_MASK                      0x00000fff
  #define EFR32_PRS_ROUTEPEN_CHPEN_COUNT           12
  #define EFR32_PRS_ROUTEPEN_CHPEN(fidx)           (0x00000001 << ((fidx)))
  #define EFR32_PRS_ROUTEPEN_CHPEN_SET(fidx, x, v) do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)

#define EFR32_PRS_ROUTELOC0_ADDR                     0x00000010
#define EFR32_PRS_ROUTELOC0_MASK                     0x3f3f3f3f
  #define EFR32_PRS_ROUTELOC0_CH0LOC(v)            ((v) << 0)
  #define EFR32_PRS_ROUTELOC0_CH0LOC_SET(x, v)     do { (x) = (((x) & ~0x3f) | ((v) << 0)); } while(0)
  #define EFR32_PRS_ROUTELOC0_CH0LOC_GET(x)        (((x) >> 0) & 0x3f)
  #define EFR32_PRS_ROUTELOC0_CH1LOC(v)            ((v) << 8)
  #define EFR32_PRS_ROUTELOC0_CH1LOC_SET(x, v)     do { (x) = (((x) & ~0x3f00) | ((v) << 8)); } while(0)
  #define EFR32_PRS_ROUTELOC0_CH1LOC_GET(x)        (((x) >> 8) & 0x3f)
  #define EFR32_PRS_ROUTELOC0_CH2LOC(v)            ((v) << 16)
  #define EFR32_PRS_ROUTELOC0_CH2LOC_SET(x, v)     do { (x) = (((x) & ~0x3f0000) | ((v) << 16)); } while(0)
  #define EFR32_PRS_ROUTELOC0_CH2LOC_GET(x)        (((x) >> 16) & 0x3f)
  #define EFR32_PRS_ROUTELOC0_CH3LOC(v)            ((v) << 24)
  #define EFR32_PRS_ROUTELOC0_CH3LOC_SET(x, v)     do { (x) = (((x) & ~0x3f000000) | ((v) << 24)); } while(0)
  #define EFR32_PRS_ROUTELOC0_CH3LOC_GET(x)        (((x) >> 24) & 0x3f)

#define EFR32_PRS_ROUTELOC1_ADDR                     0x00000014
#define EFR32_PRS_ROUTELOC1_MASK                     0x3f3f3f3f
  #define EFR32_PRS_ROUTELOC1_CH4LOC(v)            ((v) << 0)
  #define EFR32_PRS_ROUTELOC1_CH4LOC_SET(x, v)     do { (x) = (((x) & ~0x3f) | ((v) << 0)); } while(0)
  #define EFR32_PRS_ROUTELOC1_CH4LOC_GET(x)        (((x) >> 0) & 0x3f)
  #define EFR32_PRS_ROUTELOC1_CH5LOC(v)            ((v) << 8)
  #define EFR32_PRS_ROUTELOC1_CH5LOC_SET(x, v)     do { (x) = (((x) & ~0x3f00) | ((v) << 8)); } while(0)
  #define EFR32_PRS_ROUTELOC1_CH5LOC_GET(x)        (((x) >> 8) & 0x3f)
  #define EFR32_PRS_ROUTELOC1_CH6LOC(v)            ((v) << 16)
  #define EFR32_PRS_ROUTELOC1_CH6LOC_SET(x, v)     do { (x) = (((x) & ~0x3f0000) | ((v) << 16)); } while(0)
  #define EFR32_PRS_ROUTELOC1_CH6LOC_GET(x)        (((x) >> 16) & 0x3f)
  #define EFR32_PRS_ROUTELOC1_CH7LOC(v)            ((v) << 24)
  #define EFR32_PRS_ROUTELOC1_CH7LOC_SET(x, v)     do { (x) = (((x) & ~0x3f000000) | ((v) << 24)); } while(0)
  #define EFR32_PRS_ROUTELOC1_CH7LOC_GET(x)        (((x) >> 24) & 0x3f)

#define EFR32_PRS_ROUTELOC2_ADDR                     0x00000018
#define EFR32_PRS_ROUTELOC2_MASK                     0x3f3f3f3f
  #define EFR32_PRS_ROUTELOC2_CH8LOC(v)            ((v) << 0)
  #define EFR32_PRS_ROUTELOC2_CH8LOC_SET(x, v)     do { (x) = (((x) & ~0x3f) | ((v) << 0)); } while(0)
  #define EFR32_PRS_ROUTELOC2_CH8LOC_GET(x)        (((x) >> 0) & 0x3f)
  #define EFR32_PRS_ROUTELOC2_CH9LOC(v)            ((v) << 8)
  #define EFR32_PRS_ROUTELOC2_CH9LOC_SET(x, v)     do { (x) = (((x) & ~0x3f00) | ((v) << 8)); } while(0)
  #define EFR32_PRS_ROUTELOC2_CH9LOC_GET(x)        (((x) >> 8) & 0x3f)
  #define EFR32_PRS_ROUTELOC2_CH10LOC(v)           ((v) << 16)
  #define EFR32_PRS_ROUTELOC2_CH10LOC_SET(x, v)    do { (x) = (((x) & ~0x3f0000) | ((v) << 16)); } while(0)
  #define EFR32_PRS_ROUTELOC2_CH10LOC_GET(x)       (((x) >> 16) & 0x3f)
  #define EFR32_PRS_ROUTELOC2_CH11LOC(v)           ((v) << 24)
  #define EFR32_PRS_ROUTELOC2_CH11LOC_SET(x, v)    do { (x) = (((x) & ~0x3f000000) | ((v) << 24)); } while(0)
  #define EFR32_PRS_ROUTELOC2_CH11LOC_GET(x)       (((x) >> 24) & 0x3f)

#define EFR32_PRS_CTRL_ADDR                          0x00000030
#define EFR32_PRS_CTRL_MASK                          0x0000001f
  #define EFR32_PRS_CTRL_SEVONPRS                  0x00000001
  #define EFR32_PRS_CTRL_SEVONPRS_SET(x, v)        do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define EFR32_PRS_CTRL_SEVONPRSSEL(v)            ((EFR32_PRS_CTRL_SEVONPRSSEL_##v) << 1)
  #define EFR32_PRS_CTRL_SEVONPRSSEL_SET(x, v)     do { (x) = (((x) & ~0x1e) | ((EFR32_PRS_CTRL_SEVONPRSSEL_##v) << 1)); } while(0)
  #define EFR32_PRS_CTRL_SEVONPRSSEL_SETVAL(x, v)  do { (x) = (((x) & ~0x1e) | ((v) << 1)); } while(0)
  #define EFR32_PRS_CTRL_SEVONPRSSEL_GET(x)        (((x) >> 1) & 0xf)
/**  */
    #define EFR32_PRS_CTRL_SEVONPRSSEL_PRSCH0        0x00000000
/**  */
    #define EFR32_PRS_CTRL_SEVONPRSSEL_PRSCH1        0x00000001
/**  */
    #define EFR32_PRS_CTRL_SEVONPRSSEL_PRSCH2        0x00000002
/**  */
    #define EFR32_PRS_CTRL_SEVONPRSSEL_PRSCH3        0x00000003
/**  */
    #define EFR32_PRS_CTRL_SEVONPRSSEL_PRSCH4        0x00000004
/**  */
    #define EFR32_PRS_CTRL_SEVONPRSSEL_PRSCH5        0x00000005
/**  */
    #define EFR32_PRS_CTRL_SEVONPRSSEL_PRSCH6        0x00000006
/**  */
    #define EFR32_PRS_CTRL_SEVONPRSSEL_PRSCH7        0x00000007
/**  */
    #define EFR32_PRS_CTRL_SEVONPRSSEL_PRSCH8        0x00000008
/**  */
    #define EFR32_PRS_CTRL_SEVONPRSSEL_PRSCH9        0x00000009
/**  */
    #define EFR32_PRS_CTRL_SEVONPRSSEL_PRSCH10       0x0000000a
/**  */
    #define EFR32_PRS_CTRL_SEVONPRSSEL_PRSCH11       0x0000000b

#define EFR32_PRS_DMAREQ0_ADDR                       0x00000034
#define EFR32_PRS_DMAREQ0_MASK                       0x000003c0
  #define EFR32_PRS_DMAREQ0_PRSSEL(v)              ((EFR32_PRS_DMAREQ0_PRSSEL_##v) << 6)
  #define EFR32_PRS_DMAREQ0_PRSSEL_SET(x, v)       do { (x) = (((x) & ~0x3c0) | ((EFR32_PRS_DMAREQ0_PRSSEL_##v) << 6)); } while(0)
  #define EFR32_PRS_DMAREQ0_PRSSEL_SETVAL(x, v)    do { (x) = (((x) & ~0x3c0) | ((v) << 6)); } while(0)
  #define EFR32_PRS_DMAREQ0_PRSSEL_GET(x)          (((x) >> 6) & 0xf)
/**  */
    #define EFR32_PRS_DMAREQ0_PRSSEL_PRSCH0          0x00000000
/**  */
    #define EFR32_PRS_DMAREQ0_PRSSEL_PRSCH1          0x00000001
/**  */
    #define EFR32_PRS_DMAREQ0_PRSSEL_PRSCH2          0x00000002
/**  */
    #define EFR32_PRS_DMAREQ0_PRSSEL_PRSCH3          0x00000003
/**  */
    #define EFR32_PRS_DMAREQ0_PRSSEL_PRSCH4          0x00000004
/**  */
    #define EFR32_PRS_DMAREQ0_PRSSEL_PRSCH5          0x00000005
/**  */
    #define EFR32_PRS_DMAREQ0_PRSSEL_PRSCH6          0x00000006
/**  */
    #define EFR32_PRS_DMAREQ0_PRSSEL_PRSCH7          0x00000007
/**  */
    #define EFR32_PRS_DMAREQ0_PRSSEL_PRSCH8          0x00000008
/**  */
    #define EFR32_PRS_DMAREQ0_PRSSEL_PRSCH9          0x00000009
/**  */
    #define EFR32_PRS_DMAREQ0_PRSSEL_PRSCH10         0x0000000a
/**  */
    #define EFR32_PRS_DMAREQ0_PRSSEL_PRSCH11         0x0000000b

#define EFR32_PRS_DMAREQ1_ADDR                       0x00000038
#define EFR32_PRS_DMAREQ1_MASK                       0x000003c0
  #define EFR32_PRS_DMAREQ1_PRSSEL(v)              ((EFR32_PRS_DMAREQ1_PRSSEL_##v) << 6)
  #define EFR32_PRS_DMAREQ1_PRSSEL_SET(x, v)       do { (x) = (((x) & ~0x3c0) | ((EFR32_PRS_DMAREQ1_PRSSEL_##v) << 6)); } while(0)
  #define EFR32_PRS_DMAREQ1_PRSSEL_SETVAL(x, v)    do { (x) = (((x) & ~0x3c0) | ((v) << 6)); } while(0)
  #define EFR32_PRS_DMAREQ1_PRSSEL_GET(x)          (((x) >> 6) & 0xf)
/**  */
    #define EFR32_PRS_DMAREQ1_PRSSEL_PRSCH0          0x00000000
/**  */
    #define EFR32_PRS_DMAREQ1_PRSSEL_PRSCH1          0x00000001
/**  */
    #define EFR32_PRS_DMAREQ1_PRSSEL_PRSCH2          0x00000002
/**  */
    #define EFR32_PRS_DMAREQ1_PRSSEL_PRSCH3          0x00000003
/**  */
    #define EFR32_PRS_DMAREQ1_PRSSEL_PRSCH4          0x00000004
/**  */
    #define EFR32_PRS_DMAREQ1_PRSSEL_PRSCH5          0x00000005
/**  */
    #define EFR32_PRS_DMAREQ1_PRSSEL_PRSCH6          0x00000006
/**  */
    #define EFR32_PRS_DMAREQ1_PRSSEL_PRSCH7          0x00000007
/**  */
    #define EFR32_PRS_DMAREQ1_PRSSEL_PRSCH8          0x00000008
/**  */
    #define EFR32_PRS_DMAREQ1_PRSSEL_PRSCH9          0x00000009
/**  */
    #define EFR32_PRS_DMAREQ1_PRSSEL_PRSCH10         0x0000000a
/**  */
    #define EFR32_PRS_DMAREQ1_PRSSEL_PRSCH11         0x0000000b

#define EFR32_PRS_PEEK_ADDR                          0x00000040
#define EFR32_PRS_PEEK_MASK                          0x00000fff
  #define EFR32_PRS_PEEK_CH0VAL                    0x00000001
  #define EFR32_PRS_PEEK_CH0VAL_SET(x, v)          do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define EFR32_PRS_PEEK_CH1VAL                    0x00000002
  #define EFR32_PRS_PEEK_CH1VAL_SET(x, v)          do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define EFR32_PRS_PEEK_CH2VAL                    0x00000004
  #define EFR32_PRS_PEEK_CH2VAL_SET(x, v)          do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
  #define EFR32_PRS_PEEK_CH3VAL                    0x00000008
  #define EFR32_PRS_PEEK_CH3VAL_SET(x, v)          do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
  #define EFR32_PRS_PEEK_CH4VAL                    0x00000010
  #define EFR32_PRS_PEEK_CH4VAL_SET(x, v)          do { (x) = (((x) & ~0x10) | ((v) << 4)); } while(0)
  #define EFR32_PRS_PEEK_CH5VAL                    0x00000020
  #define EFR32_PRS_PEEK_CH5VAL_SET(x, v)          do { (x) = (((x) & ~0x20) | ((v) << 5)); } while(0)
  #define EFR32_PRS_PEEK_CH6VAL                    0x00000040
  #define EFR32_PRS_PEEK_CH6VAL_SET(x, v)          do { (x) = (((x) & ~0x40) | ((v) << 6)); } while(0)
  #define EFR32_PRS_PEEK_CH7VAL                    0x00000080
  #define EFR32_PRS_PEEK_CH7VAL_SET(x, v)          do { (x) = (((x) & ~0x80) | ((v) << 7)); } while(0)
  #define EFR32_PRS_PEEK_CH8VAL                    0x00000100
  #define EFR32_PRS_PEEK_CH8VAL_SET(x, v)          do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define EFR32_PRS_PEEK_CH9VAL                    0x00000200
  #define EFR32_PRS_PEEK_CH9VAL_SET(x, v)          do { (x) = (((x) & ~0x200) | ((v) << 9)); } while(0)
  #define EFR32_PRS_PEEK_CH10VAL                   0x00000400
  #define EFR32_PRS_PEEK_CH10VAL_SET(x, v)         do { (x) = (((x) & ~0x400) | ((v) << 10)); } while(0)
  #define EFR32_PRS_PEEK_CH11VAL                   0x00000800
  #define EFR32_PRS_PEEK_CH11VAL_SET(x, v)         do { (x) = (((x) & ~0x800) | ((v) << 11)); } while(0)

#define EFR32_PRS_CH_CTRL_ADDR(ridx)                 (0x00000050 + (ridx) * 4)
#define EFR32_PRS_CH_CTRL_COUNT                      12
#define EFR32_PRS_CH_CTRL_MASK                       0x5e307f07
  #define EFR32_PRS_CH_CTRL_SIGSEL(v)              ((EFR32_PRS_CH_CTRL_SIGSEL_##v) << 0)
  #define EFR32_PRS_CH_CTRL_SIGSEL_SET(x, v)       do { (x) = (((x) & ~0x7) | ((EFR32_PRS_CH_CTRL_SIGSEL_##v) << 0)); } while(0)
  #define EFR32_PRS_CH_CTRL_SIGSEL_SETVAL(x, v)    do { (x) = (((x) & ~0x7) | ((v) << 0)); } while(0)
  #define EFR32_PRS_CH_CTRL_SIGSEL_GET(x)          (((x) >> 0) & 0x7)
    #define EFR32_PRS_CH_CTRL_SIGSEL_RACTX           0x00000001
    #define EFR32_PRS_CH_CTRL_SIGSEL_RACRX           0x00000002
    #define EFR32_PRS_CH_CTRL_SIGSEL_RACLNAEN        0x00000003
    #define EFR32_PRS_CH_CTRL_SIGSEL_RACPAEN         0x00000004
  #define EFR32_PRS_CH_CTRL_SOURCESEL(v)           ((EFR32_PRS_CH_CTRL_SOURCESEL_##v) << 8)
  #define EFR32_PRS_CH_CTRL_SOURCESEL_SET(x, v)    do { (x) = (((x) & ~0x7f00) | ((EFR32_PRS_CH_CTRL_SOURCESEL_##v) << 8)); } while(0)
  #define EFR32_PRS_CH_CTRL_SOURCESEL_SETVAL(x, v) do { (x) = (((x) & ~0x7f00) | ((v) << 8)); } while(0)
  #define EFR32_PRS_CH_CTRL_SOURCESEL_GET(x)       (((x) >> 8) & 0x7f)
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_NONE         0x00000000
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_PRSL         0x00000001
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_PRSH         0x00000002
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_ACMP0        0x00000003
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_ACMP1        0x00000004
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_ADC0         0x00000005
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_CSEN         0x00000006
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_LESENSEL     0x00000007
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_LESENSEH     0x00000008
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_LESENSED     0x00000009
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_LESENSE      0x0000000a
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_RTCC         0x0000000b
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_GPIOL        0x0000000c
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_GPIOH        0x0000000d
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_LETIMER0     0x0000000e
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_PCNT0        0x0000000f
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_PCNT1        0x00000010
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_PCNT2        0x00000011
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_CMU          0x00000012
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_LETEST       0x00000013
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_MANTESTL     0x00000014
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_MANTESTH     0x00000015
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_EMUL         0x00000016
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_EMUH         0x00000017
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_VDAC0        0x00000018
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_RFSENSE      0x00000019
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_CRYOTIMER    0x0000001a
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_USART0       0x00000030
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_USART1       0x00000031
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_USART2       0x00000032
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_USART3       0x00000033
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_TIMER0       0x0000003c
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_TIMER1       0x0000003d
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_WTIMER0      0x0000003e
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_WTIMER1      0x0000003f
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_CM4          0x00000043
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_DMEM0        0x00000047
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_DMEM1        0x00000048
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_RACH         0x00000050
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_RAC          0x00000051
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_BUFC         0x00000052
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_PROTIMERL    0x00000053
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_PROTIMERH    0x00000054
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_FRC          0x00000055
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_MODEML       0x00000056
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_MODEMH       0x00000057
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_AGC          0x00000058
/**  */
    #define EFR32_PRS_CH_CTRL_SOURCESEL_SYNTH        0x0000005a
  #define EFR32_PRS_CH_CTRL_EDSEL(v)               ((EFR32_PRS_CH_CTRL_EDSEL_##v) << 20)
  #define EFR32_PRS_CH_CTRL_EDSEL_SET(x, v)        do { (x) = (((x) & ~0x300000) | ((EFR32_PRS_CH_CTRL_EDSEL_##v) << 20)); } while(0)
  #define EFR32_PRS_CH_CTRL_EDSEL_SETVAL(x, v)     do { (x) = (((x) & ~0x300000) | ((v) << 20)); } while(0)
  #define EFR32_PRS_CH_CTRL_EDSEL_GET(x)           (((x) >> 20) & 0x3)
/**  */
    #define EFR32_PRS_CH_CTRL_EDSEL_OFF              0x00000000
/**  */
    #define EFR32_PRS_CH_CTRL_EDSEL_POSEDGE          0x00000001
/**  */
    #define EFR32_PRS_CH_CTRL_EDSEL_NEGEDGE          0x00000002
/**  */
    #define EFR32_PRS_CH_CTRL_EDSEL_BOTHEDGES        0x00000003
  #define EFR32_PRS_CH_CTRL_STRETCH                0x02000000
  #define EFR32_PRS_CH_CTRL_STRETCH_SET(x, v)      do { (x) = (((x) & ~0x2000000) | ((v) << 25)); } while(0)
  #define EFR32_PRS_CH_CTRL_INV                    0x04000000
  #define EFR32_PRS_CH_CTRL_INV_SET(x, v)          do { (x) = (((x) & ~0x4000000) | ((v) << 26)); } while(0)
  #define EFR32_PRS_CH_CTRL_ORPREV                 0x08000000
  #define EFR32_PRS_CH_CTRL_ORPREV_SET(x, v)       do { (x) = (((x) & ~0x8000000) | ((v) << 27)); } while(0)
  #define EFR32_PRS_CH_CTRL_ANDNEXT                0x10000000
  #define EFR32_PRS_CH_CTRL_ANDNEXT_SET(x, v)      do { (x) = (((x) & ~0x10000000) | ((v) << 28)); } while(0)
  #define EFR32_PRS_CH_CTRL_ASYNC                  0x40000000
  #define EFR32_PRS_CH_CTRL_ASYNC_SET(x, v)        do { (x) = (((x) & ~0x40000000) | ((v) << 30)); } while(0)

#endif

