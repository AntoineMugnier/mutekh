/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs -I defs/cc26xx/aux_wuc.bf cdefs_use_reg_comment=0            \
     cdefs_use_field_comment=0 cdefs_use_field_get=2 cdefs_use_field_set=2     \
     cdefs_use_field_shift=1 cdefs_use_field_shifted_mask=1                    \
     cdefs_use_field_shifter=0 cdefs_use_reg_comment=0                         \
     cdefs_use_value_comment=0 doc_field_doc_column=0                          \
     doc_field_longname_column=0 doc_lsb_on_left=0 doc_reg_address_column=0    \
     doc_reg_direction_column=0 doc_reg_doc_column=0 doc_reg_longname_column=0 \
     doc_split_width=0
*/

#ifndef _CC26XX_AUX_WUC_BFGEN_DEFS_
#define _CC26XX_AUX_WUC_BFGEN_DEFS_

#define CC26XX_AUX_WUC_MODCLKEN0_ADDR                0x00000000
  #define CC26XX_AUX_WUC_MODCLKEN0_SMPH            0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN0_SMPH_SHIFT      0
  #define CC26XX_AUX_WUC_MODCLKEN0_SMPH_SET(x, v)  do { (x) = (((x) & ~0x1) | ((CC26XX_AUX_WUC_MODCLKEN0_SMPH_##v) << 0)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN0_SMPH_GET(x)     (((x) >> 0) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN0_SMPH_DIS        0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN0_SMPH_EN         0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN0_AIODIO0         0x00000002
  #define CC26XX_AUX_WUC_MODCLKEN0_AIODIO0_SHIFT   1
  #define CC26XX_AUX_WUC_MODCLKEN0_AIODIO0_SET(x, v) do { (x) = (((x) & ~0x2) | ((CC26XX_AUX_WUC_MODCLKEN0_AIODIO0_##v) << 1)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN0_AIODIO0_GET(x)  (((x) >> 1) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN0_AIODIO0_DIS     0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN0_AIODIO0_EN      0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN0_AIODIO1         0x00000004
  #define CC26XX_AUX_WUC_MODCLKEN0_AIODIO1_SHIFT   2
  #define CC26XX_AUX_WUC_MODCLKEN0_AIODIO1_SET(x, v) do { (x) = (((x) & ~0x4) | ((CC26XX_AUX_WUC_MODCLKEN0_AIODIO1_##v) << 2)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN0_AIODIO1_GET(x)  (((x) >> 2) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN0_AIODIO1_DIS     0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN0_AIODIO1_EN      0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN0_TIMER           0x00000008
  #define CC26XX_AUX_WUC_MODCLKEN0_TIMER_SHIFT     3
  #define CC26XX_AUX_WUC_MODCLKEN0_TIMER_SET(x, v) do { (x) = (((x) & ~0x8) | ((CC26XX_AUX_WUC_MODCLKEN0_TIMER_##v) << 3)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN0_TIMER_GET(x)    (((x) >> 3) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN0_TIMER_DIS       0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN0_TIMER_EN        0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN0_SOC             0x00000010
  #define CC26XX_AUX_WUC_MODCLKEN0_SOC_SHIFT       4
  #define CC26XX_AUX_WUC_MODCLKEN0_SOC_SET(x, v)   do { (x) = (((x) & ~0x10) | ((CC26XX_AUX_WUC_MODCLKEN0_SOC_##v) << 4)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN0_SOC_GET(x)      (((x) >> 4) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN0_SOC_DIS         0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN0_SOC_EN          0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN0_TDC             0x00000020
  #define CC26XX_AUX_WUC_MODCLKEN0_TDC_SHIFT       5
  #define CC26XX_AUX_WUC_MODCLKEN0_TDC_SET(x, v)   do { (x) = (((x) & ~0x20) | ((CC26XX_AUX_WUC_MODCLKEN0_TDC_##v) << 5)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN0_TDC_GET(x)      (((x) >> 5) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN0_TDC_DIS         0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN0_TDC_EN          0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN0_AUX_DDI0_OSC    0x00000040
  #define CC26XX_AUX_WUC_MODCLKEN0_AUX_DDI0_OSC_SHIFT 6
  #define CC26XX_AUX_WUC_MODCLKEN0_AUX_DDI0_OSC_SET(x, v) do { (x) = (((x) & ~0x40) | ((CC26XX_AUX_WUC_MODCLKEN0_AUX_DDI0_OSC_##v) << 6)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN0_AUX_DDI0_OSC_GET(x) (((x) >> 6) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN0_AUX_DDI0_OSC_DIS 0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN0_AUX_DDI0_OSC_EN 0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN0_AUX_ADI4        0x00000080
  #define CC26XX_AUX_WUC_MODCLKEN0_AUX_ADI4_SHIFT  7
  #define CC26XX_AUX_WUC_MODCLKEN0_AUX_ADI4_SET(x, v) do { (x) = (((x) & ~0x80) | ((CC26XX_AUX_WUC_MODCLKEN0_AUX_ADI4_##v) << 7)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN0_AUX_ADI4_GET(x) (((x) >> 7) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN0_AUX_ADI4_DIS    0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN0_AUX_ADI4_EN     0x00000001

#define CC26XX_AUX_WUC_PWROFFREQ_ADDR                0x00000004
  #define CC26XX_AUX_WUC_PWROFFREQ_REQ             0x00000001
  #define CC26XX_AUX_WUC_PWROFFREQ_REQ_SHIFT       0

#define CC26XX_AUX_WUC_PWRDWNREQ_ADDR                0x00000008
  #define CC26XX_AUX_WUC_PWRDWNREQ_REQ             0x00000001
  #define CC26XX_AUX_WUC_PWRDWNREQ_REQ_SHIFT       0

#define CC26XX_AUX_WUC_PWRDWNACK_ADDR                0x0000000c
  #define CC26XX_AUX_WUC_PWRDWNACK_ACK             0x00000001
  #define CC26XX_AUX_WUC_PWRDWNACK_ACK_SHIFT       0

#define CC26XX_AUX_WUC_CLKLFREQ_ADDR                 0x00000010
  #define CC26XX_AUX_WUC_CLKLFREQ_REQ              0x00000001
  #define CC26XX_AUX_WUC_CLKLFREQ_REQ_SHIFT        0

#define CC26XX_AUX_WUC_CLKLFACK_ADDR                 0x00000014
  #define CC26XX_AUX_WUC_CLKLFACK_ACK              0x00000001
  #define CC26XX_AUX_WUC_CLKLFACK_ACK_SHIFT        0

#define CC26XX_AUX_WUC_GBIASREQ_ADDR                 0x00000018
  #define CC26XX_AUX_WUC_GBIASREQ_REQ              0x00000001
  #define CC26XX_AUX_WUC_GBIASREQ_REQ_SHIFT        0

#define CC26XX_AUX_WUC_GBIASACK_ADDR                 0x0000001c
  #define CC26XX_AUX_WUC_GBIASACK_ACK              0x00000001
  #define CC26XX_AUX_WUC_GBIASACK_ACK_SHIFT        0

#define CC26XX_AUX_WUC_BGAPREQ_ADDR                  0x00000020
  #define CC26XX_AUX_WUC_BGAPREQ_REQ               0x00000001
  #define CC26XX_AUX_WUC_BGAPREQ_REQ_SHIFT         0

#define CC26XX_AUX_WUC_BGAPACK_ADDR                  0x00000024
  #define CC26XX_AUX_WUC_BGAPACK_ACK               0x00000001
  #define CC26XX_AUX_WUC_BGAPACK_ACK_SHIFT         0

#define CC26XX_AUX_WUC_WUEVFLAGS_ADDR                0x00000028
  #define CC26XX_AUX_WUC_WUEVFLAGS_AON_PROG_WU     0x00000001
  #define CC26XX_AUX_WUC_WUEVFLAGS_AON_PROG_WU_SHIFT 0
  #define CC26XX_AUX_WUC_WUEVFLAGS_AON_SW          0x00000002
  #define CC26XX_AUX_WUC_WUEVFLAGS_AON_SW_SHIFT    1
  #define CC26XX_AUX_WUC_WUEVFLAGS_AON_RTC_CH2     0x00000004
  #define CC26XX_AUX_WUC_WUEVFLAGS_AON_RTC_CH2_SHIFT 2

#define CC26XX_AUX_WUC_WUEVCLR_ADDR                  0x0000002c
  #define CC26XX_AUX_WUC_WUEVCLR_AON_PROG_WU       0x00000001
  #define CC26XX_AUX_WUC_WUEVCLR_AON_PROG_WU_SHIFT 0
  #define CC26XX_AUX_WUC_WUEVCLR_AON_SW            0x00000002
  #define CC26XX_AUX_WUC_WUEVCLR_AON_SW_SHIFT      1
  #define CC26XX_AUX_WUC_WUEVCLR_AON_RTC_CH2       0x00000004
  #define CC26XX_AUX_WUC_WUEVCLR_AON_RTC_CH2_SHIFT 2

#define CC26XX_AUX_WUC_ADCCLKCTL_ADDR                0x00000030
  #define CC26XX_AUX_WUC_ADCCLKCTL_REQ             0x00000001
  #define CC26XX_AUX_WUC_ADCCLKCTL_REQ_SHIFT       0
  #define CC26XX_AUX_WUC_ADCCLKCTL_ACK             0x00000002
  #define CC26XX_AUX_WUC_ADCCLKCTL_ACK_SHIFT       1

#define CC26XX_AUX_WUC_TDCCLKCTL_ADDR                0x00000034
  #define CC26XX_AUX_WUC_TDCCLKCTL_REQ             0x00000001
  #define CC26XX_AUX_WUC_TDCCLKCTL_REQ_SHIFT       0
  #define CC26XX_AUX_WUC_TDCCLKCTL_ACK             0x00000002
  #define CC26XX_AUX_WUC_TDCCLKCTL_ACK_SHIFT       1

#define CC26XX_AUX_WUC_REFCLKCTL_ADDR                0x00000038
  #define CC26XX_AUX_WUC_REFCLKCTL_REQ             0x00000001
  #define CC26XX_AUX_WUC_REFCLKCTL_REQ_SHIFT       0
  #define CC26XX_AUX_WUC_REFCLKCTL_ACK             0x00000002
  #define CC26XX_AUX_WUC_REFCLKCTL_ACK_SHIFT       1

#define CC26XX_AUX_WUC_RTCSUBSECINC0_ADDR            0x0000003c
  #define CC26XX_AUX_WUC_RTCSUBSECINC0_INC15_0     0x0000ffff
  #define CC26XX_AUX_WUC_RTCSUBSECINC0_INC15_0_SHIFT 0
  #define CC26XX_AUX_WUC_RTCSUBSECINC0_INC15_0_SET(x, v) do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define CC26XX_AUX_WUC_RTCSUBSECINC0_INC15_0_GET(x) (((x) >> 0) & 0xffff)

#define CC26XX_AUX_WUC_RTCSUBSECINC1_ADDR            0x00000040
  #define CC26XX_AUX_WUC_RTCSUBSECINC1_INC23_16    0x000000ff
  #define CC26XX_AUX_WUC_RTCSUBSECINC1_INC23_16_SHIFT 0
  #define CC26XX_AUX_WUC_RTCSUBSECINC1_INC23_16_SET(x, v) do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define CC26XX_AUX_WUC_RTCSUBSECINC1_INC23_16_GET(x) (((x) >> 0) & 0xff)

#define CC26XX_AUX_WUC_RTCSUBSECINCCTL_ADDR          0x00000044
  #define CC26XX_AUX_WUC_RTCSUBSECINCCTL_UPD_REQ   0x00000001
  #define CC26XX_AUX_WUC_RTCSUBSECINCCTL_UPD_REQ_SHIFT 0
  #define CC26XX_AUX_WUC_RTCSUBSECINCCTL_UPD_ACK   0x00000002
  #define CC26XX_AUX_WUC_RTCSUBSECINCCTL_UPD_ACK_SHIFT 1

#define CC26XX_AUX_WUC_MCUBUSCTL_ADDR                0x00000048
  #define CC26XX_AUX_WUC_MCUBUSCTL_DISCONNECT_REQ  0x00000001
  #define CC26XX_AUX_WUC_MCUBUSCTL_DISCONNECT_REQ_SHIFT 0

#define CC26XX_AUX_WUC_MCUBUSSTAT_ADDR               0x0000004c
  #define CC26XX_AUX_WUC_MCUBUSSTAT_DISCONNECT_ACK 0x00000001
  #define CC26XX_AUX_WUC_MCUBUSSTAT_DISCONNECT_ACK_SHIFT 0
  #define CC26XX_AUX_WUC_MCUBUSSTAT_DISCONNECTED   0x00000002
  #define CC26XX_AUX_WUC_MCUBUSSTAT_DISCONNECTED_SHIFT 1

#define CC26XX_AUX_WUC_AONCTLSTAT_ADDR               0x00000050
  #define CC26XX_AUX_WUC_AONCTLSTAT_SCE_RUN_EN     0x00000001
  #define CC26XX_AUX_WUC_AONCTLSTAT_SCE_RUN_EN_SHIFT 0
  #define CC26XX_AUX_WUC_AONCTLSTAT_AUX_FORCE_ON   0x00000002
  #define CC26XX_AUX_WUC_AONCTLSTAT_AUX_FORCE_ON_SHIFT 1

#define CC26XX_AUX_WUC_AUXIOLATCH_ADDR               0x00000054
  #define CC26XX_AUX_WUC_AUXIOLATCH_EN             0x00000001
  #define CC26XX_AUX_WUC_AUXIOLATCH_EN_SHIFT       0
  #define CC26XX_AUX_WUC_AUXIOLATCH_EN_SET(x, v)   do { (x) = (((x) & ~0x1) | ((CC26XX_AUX_WUC_AUXIOLATCH_EN_##v) << 0)); } while(0)
  #define CC26XX_AUX_WUC_AUXIOLATCH_EN_GET(x)      (((x) >> 0) & 0x1)
    #define CC26XX_AUX_WUC_AUXIOLATCH_EN_STATIC      0x00000000
    #define CC26XX_AUX_WUC_AUXIOLATCH_EN_TRANSP      0x00000001

#define CC26XX_AUX_WUC_DEFAULTALIAS_ADDR             0x00000058

#define CC26XX_AUX_WUC_MODCLKEN1_ADDR                0x0000005c
  #define CC26XX_AUX_WUC_MODCLKEN1_SMPH            0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN1_SMPH_SHIFT      0
  #define CC26XX_AUX_WUC_MODCLKEN1_SMPH_SET(x, v)  do { (x) = (((x) & ~0x1) | ((CC26XX_AUX_WUC_MODCLKEN1_SMPH_##v) << 0)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN1_SMPH_GET(x)     (((x) >> 0) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN1_SMPH_DIS        0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN1_SMPH_EN         0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN1_AIODIO0         0x00000002
  #define CC26XX_AUX_WUC_MODCLKEN1_AIODIO0_SHIFT   1
  #define CC26XX_AUX_WUC_MODCLKEN1_AIODIO0_SET(x, v) do { (x) = (((x) & ~0x2) | ((CC26XX_AUX_WUC_MODCLKEN1_AIODIO0_##v) << 1)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN1_AIODIO0_GET(x)  (((x) >> 1) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN1_AIODIO0_DIS     0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN1_AIODIO0_EN      0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN1_AIODIO1         0x00000004
  #define CC26XX_AUX_WUC_MODCLKEN1_AIODIO1_SHIFT   2
  #define CC26XX_AUX_WUC_MODCLKEN1_AIODIO1_SET(x, v) do { (x) = (((x) & ~0x4) | ((CC26XX_AUX_WUC_MODCLKEN1_AIODIO1_##v) << 2)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN1_AIODIO1_GET(x)  (((x) >> 2) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN1_AIODIO1_DIS     0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN1_AIODIO1_EN      0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN1_TIMER           0x00000008
  #define CC26XX_AUX_WUC_MODCLKEN1_TIMER_SHIFT     3
  #define CC26XX_AUX_WUC_MODCLKEN1_TIMER_SET(x, v) do { (x) = (((x) & ~0x8) | ((CC26XX_AUX_WUC_MODCLKEN1_TIMER_##v) << 3)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN1_TIMER_GET(x)    (((x) >> 3) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN1_TIMER_DIS       0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN1_TIMER_EN        0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN1_SOC             0x00000010
  #define CC26XX_AUX_WUC_MODCLKEN1_SOC_SHIFT       4
  #define CC26XX_AUX_WUC_MODCLKEN1_SOC_SET(x, v)   do { (x) = (((x) & ~0x10) | ((CC26XX_AUX_WUC_MODCLKEN1_SOC_##v) << 4)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN1_SOC_GET(x)      (((x) >> 4) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN1_SOC_DIS         0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN1_SOC_EN          0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN1_TDC             0x00000020
  #define CC26XX_AUX_WUC_MODCLKEN1_TDC_SHIFT       5
  #define CC26XX_AUX_WUC_MODCLKEN1_AUX_DDI0_OSC    0x00000040
  #define CC26XX_AUX_WUC_MODCLKEN1_AUX_DDI0_OSC_SHIFT 6
  #define CC26XX_AUX_WUC_MODCLKEN1_AUX_DDI0_OSC_SET(x, v) do { (x) = (((x) & ~0x40) | ((CC26XX_AUX_WUC_MODCLKEN1_AUX_DDI0_OSC_##v) << 6)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN1_AUX_DDI0_OSC_GET(x) (((x) >> 6) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN1_AUX_DDI0_OSC_DIS 0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN1_AUX_DDI0_OSC_EN 0x00000001
  #define CC26XX_AUX_WUC_MODCLKEN1_AUX_ADI4        0x00000080
  #define CC26XX_AUX_WUC_MODCLKEN1_AUX_ADI4_SHIFT  7
  #define CC26XX_AUX_WUC_MODCLKEN1_AUX_ADI4_SET(x, v) do { (x) = (((x) & ~0x80) | ((CC26XX_AUX_WUC_MODCLKEN1_AUX_ADI4_##v) << 7)); } while(0)
  #define CC26XX_AUX_WUC_MODCLKEN1_AUX_ADI4_GET(x) (((x) >> 7) & 0x1)
    #define CC26XX_AUX_WUC_MODCLKEN1_AUX_ADI4_DIS    0x00000000
    #define CC26XX_AUX_WUC_MODCLKEN1_AUX_ADI4_EN     0x00000001

#define CC26XX_AUX_WUC_PWRSTAT_ADDR                  0x00000060
  #define CC26XX_AUX_WUC_PWRSTAT_PWRPROF           0x00000003
  #define CC26XX_AUX_WUC_PWRSTAT_PWRPROF_SHIFT     0
  #define CC26XX_AUX_WUC_PWRSTAT_PWRPROF_SET(x, v) do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define CC26XX_AUX_WUC_PWRSTAT_PWRPROF_GET(x)    (((x) >> 0) & 0x3)
  #define CC26XX_AUX_WUC_PWRSTAT_PMSTATE           0x0000003c
  #define CC26XX_AUX_WUC_PWRSTAT_PMSTATE_SHIFT     2
  #define CC26XX_AUX_WUC_PWRSTAT_PMSTATE_SET(x, v) do { (x) = (((x) & ~0x3c) | ((CC26XX_AUX_WUC_PWRSTAT_PMSTATE_##v) << 2)); } while(0)
  #define CC26XX_AUX_WUC_PWRSTAT_PMSTATE_GET(x)    (((x) >> 2) & 0xf)
    #define CC26XX_AUX_WUC_PWRSTAT_PMSTATE_CON_ACT   0x00000000
    #define CC26XX_AUX_WUC_PWRSTAT_PMSTATE_DIS_PD    0x00000000
    #define CC26XX_AUX_WUC_PWRSTAT_PMSTATE_DIS_GPD   0x00000000
    #define CC26XX_AUX_WUC_PWRSTAT_PMSTATE_DIS_GPOFF 0x00000000
    #define CC26XX_AUX_WUC_PWRSTAT_PMSTATE_GCON_ACT  0x00000001
    #define CC26XX_AUX_WUC_PWRSTAT_PMSTATE_GDIS_ACT  0x00000004
    #define CC26XX_AUX_WUC_PWRSTAT_PMSTATE_DIS_ACT   0x00000005
    #define CC26XX_AUX_WUC_PWRSTAT_PMSTATE_DIS_GACT  0x00000006

#endif

