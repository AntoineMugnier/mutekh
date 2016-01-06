/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs -I defs/cc26xx/ddi_0_osc.bf cdefs_use_reg_comment=0          \
     cdefs_use_field_comment=0 cdefs_use_field_get=2 cdefs_use_field_set=2     \
     cdefs_use_field_shift=1 cdefs_use_field_shifted_mask=1                    \
     cdefs_use_field_shifter=0 cdefs_use_reg_comment=0                         \
     cdefs_use_value_comment=0 doc_field_doc_column=0                          \
     doc_field_longname_column=0 doc_lsb_on_left=0 doc_reg_address_column=0    \
     doc_reg_direction_column=0 doc_reg_doc_column=0 doc_reg_longname_column=0 \
     doc_split_width=0
*/

#ifndef _CC26XX_AUX_DDI0_OSC_BFGEN_DEFS_
#define _CC26XX_AUX_DDI0_OSC_BFGEN_DEFS_

#define CC26XX_AUX_DDI0_OSC_CTL0_ADDR                0x00000000
  #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_HF_SRC_SEL 0x00000001
  #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_HF_SRC_SEL_SHIFT 0
  #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_HF_SRC_SEL_SET(x, v) do { (x) = (((x) & ~0x1) | ((CC26XX_AUX_DDI0_OSC_CTL0_SCLK_HF_SRC_SEL_##v) << 0)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_HF_SRC_SEL_GET(x) (((x) >> 0) & 0x1)
    #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_HF_SRC_SEL_RCOSC 0x00000000
    #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_HF_SRC_SEL_XOSC 0x00000001
  #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_MF_SRC_SEL 0x00000002
  #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_MF_SRC_SEL_SHIFT 1
  #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_MF_SRC_SEL_SET(x, v) do { (x) = (((x) & ~0x2) | ((CC26XX_AUX_DDI0_OSC_CTL0_SCLK_MF_SRC_SEL_##v) << 1)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_MF_SRC_SEL_GET(x) (((x) >> 1) & 0x1)
    #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_MF_SRC_SEL_RCOSCHFDMF 0x00000000
    #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_MF_SRC_SEL_XCOSCHFDMF 0x00000001
  #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_LF_SRC_SEL 0x0000000c
  #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_SHIFT 2
  #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_SET(x, v) do { (x) = (((x) & ~0xc) | ((CC26XX_AUX_DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_##v) << 2)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_GET(x) (((x) >> 2) & 0x3)
    #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_RCOSCHFDLF 0x00000000
    #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_XOSCHFDLF 0x00000001
    #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_RCOSCLF 0x00000002
    #define CC26XX_AUX_DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_XOSCLF 0x00000003
  #define CC26XX_AUX_DDI0_OSC_CTL0_ACLK_REF_SRC_SEL 0x00000060
  #define CC26XX_AUX_DDI0_OSC_CTL0_ACLK_REF_SRC_SEL_SHIFT 5
  #define CC26XX_AUX_DDI0_OSC_CTL0_ACLK_REF_SRC_SEL_SET(x, v) do { (x) = (((x) & ~0x60) | ((v) << 5)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_CTL0_ACLK_REF_SRC_SEL_GET(x) (((x) >> 5) & 0x3)
  #define CC26XX_AUX_DDI0_OSC_CTL0_ACLK_TDC_SRC_SEL 0x00000180
  #define CC26XX_AUX_DDI0_OSC_CTL0_ACLK_TDC_SRC_SEL_SHIFT 7
  #define CC26XX_AUX_DDI0_OSC_CTL0_ACLK_TDC_SRC_SEL_SET(x, v) do { (x) = (((x) & ~0x180) | ((v) << 7)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_CTL0_ACLK_TDC_SRC_SEL_GET(x) (((x) >> 7) & 0x3)
  #define CC26XX_AUX_DDI0_OSC_CTL0_CLK_LOSS_EN     0x00000200
  #define CC26XX_AUX_DDI0_OSC_CTL0_CLK_LOSS_EN_SHIFT 9
  #define CC26XX_AUX_DDI0_OSC_CTL0_XOSC_LF_DIG_BYPASS 0x00000400
  #define CC26XX_AUX_DDI0_OSC_CTL0_XOSC_LF_DIG_BYPASS_SHIFT 10
  #define CC26XX_AUX_DDI0_OSC_CTL0_XOSC_HF_POWER_MODE 0x00000800
  #define CC26XX_AUX_DDI0_OSC_CTL0_XOSC_HF_POWER_MODE_SHIFT 11
  #define CC26XX_AUX_DDI0_OSC_CTL0_RCOSC_LF_TRIMMED 0x00001000
  #define CC26XX_AUX_DDI0_OSC_CTL0_RCOSC_LF_TRIMMED_SHIFT 12
  #define CC26XX_AUX_DDI0_OSC_CTL0_OSCDIG_BYP_CLK_SRC_IS_INT 0x00002000
  #define CC26XX_AUX_DDI0_OSC_CTL0_OSCDIG_BYP_CLK_SRC_IS_INT_SHIFT 13
  #define CC26XX_AUX_DDI0_OSC_CTL0_BAW_MODE_EN     0x00004000
  #define CC26XX_AUX_DDI0_OSC_CTL0_BAW_MODE_EN_SHIFT 14
  #define CC26XX_AUX_DDI0_OSC_CTL0_PD_PWR_SAVING_EN 0x00008000
  #define CC26XX_AUX_DDI0_OSC_CTL0_PD_PWR_SAVING_EN_SHIFT 15
  #define CC26XX_AUX_DDI0_OSC_CTL0_ALLOW_SCLK_HF_SWITCHING 0x00010000
  #define CC26XX_AUX_DDI0_OSC_CTL0_ALLOW_SCLK_HF_SWITCHING_SHIFT 16
  #define CC26XX_AUX_DDI0_OSC_CTL0_AMPCOMP_REQ_IGNORES_GBIAS 0x00020000
  #define CC26XX_AUX_DDI0_OSC_CTL0_AMPCOMP_REQ_IGNORES_GBIAS_SHIFT 17
  #define CC26XX_AUX_DDI0_OSC_CTL0_GBIAS_NEEDED_SW_EN 0x00040000
  #define CC26XX_AUX_DDI0_OSC_CTL0_GBIAS_NEEDED_SW_EN_SHIFT 18
  #define CC26XX_AUX_DDI0_OSC_CTL0_GBIAS_NEEDED_SW_CTL 0x00080000
  #define CC26XX_AUX_DDI0_OSC_CTL0_GBIAS_NEEDED_SW_CTL_SHIFT 19
  #define CC26XX_AUX_DDI0_OSC_CTL0_BGAP_NEEDED_SW_EN 0x00100000
  #define CC26XX_AUX_DDI0_OSC_CTL0_BGAP_NEEDED_SW_EN_SHIFT 20
  #define CC26XX_AUX_DDI0_OSC_CTL0_BGAP_NEEDED_SW_CTL 0x00200000
  #define CC26XX_AUX_DDI0_OSC_CTL0_BGAP_NEEDED_SW_CTL_SHIFT 21
  #define CC26XX_AUX_DDI0_OSC_CTL0_FORCE_KICKSTART_EN 0x00400000
  #define CC26XX_AUX_DDI0_OSC_CTL0_FORCE_KICKSTART_EN_SHIFT 22
  #define CC26XX_AUX_DDI0_OSC_CTL0_ACT_PWR_SAVING_EN 0x00800000
  #define CC26XX_AUX_DDI0_OSC_CTL0_ACT_PWR_SAVING_EN_SHIFT 23
  #define CC26XX_AUX_DDI0_OSC_CTL0_DOUBLER_RESET_DURATION 0x02000000
  #define CC26XX_AUX_DDI0_OSC_CTL0_DOUBLER_RESET_DURATION_SHIFT 25
  #define CC26XX_AUX_DDI0_OSC_CTL0_DOUBLER_START_DURATION 0x0c000000
  #define CC26XX_AUX_DDI0_OSC_CTL0_DOUBLER_START_DURATION_SHIFT 26
  #define CC26XX_AUX_DDI0_OSC_CTL0_DOUBLER_START_DURATION_SET(x, v) do { (x) = (((x) & ~0xc000000) | ((v) << 26)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_CTL0_DOUBLER_START_DURATION_GET(x) (((x) >> 26) & 0x3)
  #define CC26XX_AUX_DDI0_OSC_CTL0_BYPASS_RCOSC_LF_CLK_QUAL 0x10000000
  #define CC26XX_AUX_DDI0_OSC_CTL0_BYPASS_RCOSC_LF_CLK_QUAL_SHIFT 28
  #define CC26XX_AUX_DDI0_OSC_CTL0_BYPASS_XOSC_LF_CLK_QUAL 0x20000000
  #define CC26XX_AUX_DDI0_OSC_CTL0_BYPASS_XOSC_LF_CLK_QUAL_SHIFT 29
  #define CC26XX_AUX_DDI0_OSC_CTL0_DOUBLER_BYPASS_CTL 0x40000000
  #define CC26XX_AUX_DDI0_OSC_CTL0_DOUBLER_BYPASS_CTL_SHIFT 30
  #define CC26XX_AUX_DDI0_OSC_CTL0_XTAL_IS_24M     0x80000000
  #define CC26XX_AUX_DDI0_OSC_CTL0_XTAL_IS_24M_SHIFT 31
  #define CC26XX_AUX_DDI0_OSC_CTL0_XTAL_IS_24M_SET(x, v) do { (x) = (((x) & ~0x80000000) | ((CC26XX_AUX_DDI0_OSC_CTL0_XTAL_IS_24M_##v) << 31)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_CTL0_XTAL_IS_24M_GET(x) (((x) >> 31) & 0x1)
    #define CC26XX_AUX_DDI0_OSC_CTL0_XTAL_IS_24M_48M 0x00000000
    #define CC26XX_AUX_DDI0_OSC_CTL0_XTAL_IS_24M_24M 0x00000001

#define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_ADDR          0x00000008
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_RADC_EXTERNAL_USE_EN 0x00000001
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_RADC_EXTERNAL_USE_EN_SHIFT 0
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_DDI_RADC_CLRZ 0x00000002
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_DDI_RADC_CLRZ_SHIFT 1
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_RADC_USE_EXTERNAL_CLK 0x00000004
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_RADC_USE_EXTERNAL_CLK_SHIFT 2
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_DDI_RADC_CLK 0x00000008
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_DDI_RADC_CLK_SHIFT 3
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_RADC_START_CONV 0x00000010
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_RADC_START_CONV_SHIFT 4
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_RADC_MODE_IS_SAR 0x00000020
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_RADC_MODE_IS_SAR_SHIFT 5
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_RADC_DAC_TH 0x00000fc0
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_RADC_DAC_TH_SHIFT 6
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_RADC_DAC_TH_SET(x, v) do { (x) = (((x) & ~0xfc0) | ((v) << 6)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_RADC_DAC_TH_GET(x) (((x) >> 6) & 0x3f)
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_IDAC_STEP 0x0000f000
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_IDAC_STEP_SHIFT 12
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_IDAC_STEP_SET(x, v) do { (x) = (((x) & ~0xf000) | ((v) << 12)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_IDAC_STEP_GET(x) (((x) >> 12) & 0xf)
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_LPM_IBIAS_WAIT_CNT 0x003f0000
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_LPM_IBIAS_WAIT_CNT_SHIFT 16
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_LPM_IBIAS_WAIT_CNT_SET(x, v) do { (x) = (((x) & ~0x3f0000) | ((v) << 16)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_LPM_IBIAS_WAIT_CNT_GET(x) (((x) >> 16) & 0x3f)
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_HPM_IBIAS_WAIT_CNT 0xffc00000
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_HPM_IBIAS_WAIT_CNT_SHIFT 22
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_HPM_IBIAS_WAIT_CNT_SET(x, v) do { (x) = (((x) & ~0xffc00000) | ((v) << 22)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_RADCEXTCFG_HPM_IBIAS_WAIT_CNT_GET(x) (((x) >> 22) & 0x3ff)

#define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_ADDR          0x0000000c
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIASCAP_HPTOLP_OL_CNT 0x0000000f
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIASCAP_HPTOLP_OL_CNT_SHIFT 0
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIASCAP_HPTOLP_OL_CNT_SET(x, v) do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIASCAP_HPTOLP_OL_CNT_GET(x) (((x) >> 0) & 0xf)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_CAP_STEP  0x000000f0
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_CAP_STEP_SHIFT 4
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_CAP_STEP_SET(x, v) do { (x) = (((x) & ~0xf0) | ((v) << 4)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_CAP_STEP_GET(x) (((x) >> 4) & 0xf)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_LPM_IBIAS_WAIT_CNT_FINAL 0x0000ff00
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_LPM_IBIAS_WAIT_CNT_FINAL_SHIFT 8
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_LPM_IBIAS_WAIT_CNT_FINAL_SET(x, v) do { (x) = (((x) & ~0xff00) | ((v) << 8)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_LPM_IBIAS_WAIT_CNT_FINAL_GET(x) (((x) >> 8) & 0xff)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIAS_INIT 0x000f0000
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIAS_INIT_SHIFT 16
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIAS_INIT_SET(x, v) do { (x) = (((x) & ~0xf0000) | ((v) << 16)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIAS_INIT_GET(x) (((x) >> 16) & 0xf)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIAS_OFFSET 0x00f00000
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIAS_OFFSET_SHIFT 20
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIAS_OFFSET_SET(x, v) do { (x) = (((x) & ~0xf00000) | ((v) << 20)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_IBIAS_OFFSET_GET(x) (((x) >> 20) & 0xf)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_XOSC_HF_HP_BUF_SW_EN 0x01000000
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_XOSC_HF_HP_BUF_SW_EN_SHIFT 24
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_XOSC_HF_HP_BUF_SW_CTRL 0x02000000
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_XOSC_HF_HP_BUF_SW_CTRL_SHIFT 25
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_SW_EN 0x04000000
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_SW_EN_SHIFT 26
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_SW_CTRL 0x08000000
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_SW_CTRL_SHIFT 27
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE 0x30000000
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_SHIFT 28
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_SET(x, v) do { (x) = (((x) & ~0x30000000) | ((CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_##v) << 28)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_GET(x) (((x) >> 28) & 0x3)
    #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_2MHZ 0x00000000
    #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_1MHZ 0x00000001
    #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_500KHZ 0x00000002
    #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_250KHZ 0x00000003
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_REQ_MODE 0x40000000
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPCTL_AMPCOMP_REQ_MODE_SHIFT 30

#define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_ADDR          0x00000010
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP1_TH 0x0000003f
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP1_TH_SHIFT 0
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP1_TH_SET(x, v) do { (x) = (((x) & ~0x3f) | ((v) << 0)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP1_TH_GET(x) (((x) >> 0) & 0x3f)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_IBIASCAP_LPTOHP_OL_CNT 0x000003c0
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_IBIASCAP_LPTOHP_OL_CNT_SHIFT 6
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_IBIASCAP_LPTOHP_OL_CNT_SET(x, v) do { (x) = (((x) & ~0x3c0) | ((v) << 6)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_IBIASCAP_LPTOHP_OL_CNT_GET(x) (((x) >> 6) & 0xf)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP3_HTH 0x0000fc00
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP3_HTH_SHIFT 10
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP3_HTH_SET(x, v) do { (x) = (((x) & ~0xfc00) | ((v) << 10)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP3_HTH_GET(x) (((x) >> 10) & 0x3f)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP3_LTH 0x00fc0000
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP3_LTH_SHIFT 18
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP3_LTH_SET(x, v) do { (x) = (((x) & ~0xfc0000) | ((v) << 18)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP3_LTH_GET(x) (((x) >> 18) & 0x3f)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP2_TH 0xfc000000
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP2_TH_SHIFT 26
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP2_TH_SET(x, v) do { (x) = (((x) & ~0xfc000000) | ((v) << 26)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH1_HPMRAMP2_TH_GET(x) (((x) >> 26) & 0x3f)

#define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_ADDR          0x00000014
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_HPM 0x000000fc
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_HPM_SHIFT 2
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_HPM_SET(x, v) do { (x) = (((x) & ~0xfc) | ((v) << 2)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_HPM_GET(x) (((x) >> 2) & 0x3f)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_LPM 0x0000fc00
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_LPM_SHIFT 10
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_LPM_SET(x, v) do { (x) = (((x) & ~0xfc00) | ((v) << 10)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_LPM_GET(x) (((x) >> 10) & 0x3f)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_LPMUPDATE_HTH 0x00fc0000
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_LPMUPDATE_HTH_SHIFT 18
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_LPMUPDATE_HTH_SET(x, v) do { (x) = (((x) & ~0xfc0000) | ((v) << 18)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_LPMUPDATE_HTH_GET(x) (((x) >> 18) & 0x3f)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_LPMUPDATE_LTH 0xfc000000
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_LPMUPDATE_LTH_SHIFT 26
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_LPMUPDATE_LTH_SET(x, v) do { (x) = (((x) & ~0xfc000000) | ((v) << 26)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_AMPCOMPTH2_LPMUPDATE_LTH_GET(x) (((x) >> 26) & 0x3f)

#define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_ADDR       0x00000018
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_COLUMN_Q12 0x0000ffff
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_COLUMN_Q12_SHIFT 0
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_COLUMN_Q12_SET(x, v) do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_COLUMN_Q12_GET(x) (((x) >> 0) & 0xffff)
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_ROW_Q12 0x000f0000
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_ROW_Q12_SHIFT 16
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_ROW_Q12_SET(x, v) do { (x) = (((x) & ~0xf0000) | ((v) << 16)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_ROW_Q12_GET(x) (((x) >> 16) & 0xf)
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_IDAC 0x07f00000
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_IDAC_SHIFT 20
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_IDAC_SET(x, v) do { (x) = (((x) & ~0x7f00000) | ((v) << 20)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_IDAC_GET(x) (((x) >> 20) & 0x7f)
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_EN 0x08000000
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_EN_SHIFT 27
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_MUX_SEL 0x10000000
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_MUX_SEL_SHIFT 28
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_PEAK_DET_EN 0x20000000
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_PEAK_DET_EN_SHIFT 29
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_HP_BUF_EN 0x40000000
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_HP_BUF_EN_SHIFT 30
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_LP_BUF_EN 0x80000000
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL1_XOSC_HF_LP_BUF_EN_SHIFT 31

#define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_ADDR       0x0000001c
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_XOSC_HF_IBIASTHERM 0x00003fff
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_XOSC_HF_IBIASTHERM_SHIFT 0
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_XOSC_HF_IBIASTHERM_SET(x, v) do { (x) = (((x) & ~0x3fff) | ((v) << 0)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_XOSC_HF_IBIASTHERM_GET(x) (((x) >> 0) & 0x3fff)
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_ADC_ANA_EN 0x04000000
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_ADC_ANA_EN_SHIFT 26
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_BGAP_NEEDED 0x08000000
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_BGAP_NEEDED_SHIFT 27
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_GBIAS_NEEDED 0x10000000
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_GBIAS_NEEDED_SHIFT 28
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_DOUBLER_EN 0x20000000
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_DOUBLER_EN_SHIFT 29
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_XOSC_LF_EN 0x40000000
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_XOSC_LF_EN_SHIFT 30
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_RCOSC_LF_EN 0x80000000
  #define CC26XX_AUX_DDI0_OSC_ANABYPASSVAL2_RCOSC_LF_EN_SHIFT 31

#define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADDR 0x00000024
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_IREF_CTRL 0x00000003
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_IREF_CTRL_SHIFT 0
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_IREF_CTRL_SET(x, v) do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_IREF_CTRL_GET(x) (((x) >> 0) & 0x3)
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_MUX_SEL 0x0000000c
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_MUX_SEL_SHIFT 2
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_MUX_SEL_SET(x, v) do { (x) = (((x) & ~0xc) | ((v) << 2)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_MUX_SEL_GET(x) (((x) >> 2) & 0x3)
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_VBUF_EN 0x00000010
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_VBUF_EN_SHIFT 4
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_MODE_EN 0x00000020
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_MODE_EN_SHIFT 5
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_EN_ATEST 0x00000040
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_EN_ATEST_SHIFT 6
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_ATEST_SRC 0x00000180
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_ATEST_SRC_SHIFT 7
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_ATEST_SRC_SET(x, v) do { (x) = (((x) & ~0x180) | ((v) << 7)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_ATEST_SRC_GET(x) (((x) >> 7) & 0x3)
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_RADC_BIAS_DIS 0x00000200
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_RADC_BIAS_DIS_SHIFT 9
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DOUBLER_BIAS_DIS 0x00010000
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DOUBLER_BIAS_DIS_SHIFT 16
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_LOOP_FILTER_RESET_VOLTAGE 0x00060000
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_LOOP_FILTER_RESET_VOLTAGE_SHIFT 17
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_LOOP_FILTER_RESET_VOLTAGE_SET(x, v) do { (x) = (((x) & ~0x60000) | ((v) << 17)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_LOOP_FILTER_RESET_VOLTAGE_GET(x) (((x) >> 17) & 0x3)
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_LOOP_FILTER_CAP 0x00080000
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_LOOP_FILTER_CAP_SHIFT 19
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_TOOHI_MODE 0x00100000
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_TOOHI_MODE_SHIFT 20
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_ATEST_ENABLE 0x00200000
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_ATEST_ENABLE_SHIFT 21
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_ATEST_SELECT 0x00400000
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_DBLR_ATEST_SELECT_SHIFT 22
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_NANOAMP_BIAS_ENABLE 0x01000000
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_NANOAMP_BIAS_ENABLE_SHIFT 24
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_NANOAMP_RES_TRIM 0x7e000000
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_NANOAMP_RES_TRIM_SHIFT 25
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_NANOAMP_RES_TRIM_SET(x, v) do { (x) = (((x) & ~0x7e000000) | ((v) << 25)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_NANOAMP_RES_TRIM_GET(x) (((x) >> 25) & 0x3f)
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_NANOAMP_TESTHIGH_I_EN 0x80000000
  #define CC26XX_AUX_DDI0_OSC_ADCDOUBLERNANOAMPCTL_NANOAMP_TESTHIGH_I_EN_SHIFT 31

#define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_ADDR           0x00000028
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_LP_BUF_ITRIM 0x00000003
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_LP_BUF_ITRIM_SHIFT 0
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_LP_BUF_ITRIM_SET(x, v) do { (x) = (((x) & ~0x3) | ((v) << 0)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_LP_BUF_ITRIM_GET(x) (((x) >> 0) & 0x3)
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_HP_BUF_ITRIM 0x0000001c
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_HP_BUF_ITRIM_SHIFT 2
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_HP_BUF_ITRIM_SET(x, v) do { (x) = (((x) & ~0x1c) | ((v) << 2)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_HP_BUF_ITRIM_GET(x) (((x) >> 2) & 0x7)
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_TESTMUX_EN 0x00000020
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_TESTMUX_EN_SHIFT 5
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_BYPASS     0x00000040
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_BYPASS_SHIFT 6
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_HP_ALT_BIAS 0x00000080
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_HP_ALT_BIAS_SHIFT 7
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_PEAK_DET_ITRIM 0x00000300
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_PEAK_DET_ITRIM_SHIFT 8
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_PEAK_DET_ITRIM_SET(x, v) do { (x) = (((x) & ~0x300) | ((v) << 8)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_PEAK_DET_ITRIM_GET(x) (((x) >> 8) & 0x3)
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_PEAKDET_BIAS_DIS 0x00000400
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_PEAKDET_BIAS_DIS_SHIFT 10
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_HPBUFF_BIAS_DIS 0x00000800
  #define CC26XX_AUX_DDI0_OSC_XOSCHFCTL_HPBUFF_BIAS_DIS_SHIFT 11

#define CC26XX_AUX_DDI0_OSC_LFOSCCTL_ADDR            0x0000002c
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM 0x000000ff
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_SHIFT 0
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_SET(x, v) do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_GET(x) (((x) >> 0) & 0xff)
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM 0x00000300
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_SHIFT 8
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_SET(x, v) do { (x) = (((x) & ~0x300) | ((CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_##v) << 8)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_GET(x) (((x) >> 8) & 0x3)
    #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_7P5MEG 0x00000000
    #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_7P0MEG 0x00000001
    #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_6P5MEG 0x00000002
    #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_6P0MEG 0x00000003
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_LOCAL_ATEST_EN 0x00000400
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_LOCAL_ATEST_EN_SHIFT 10
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_VDD_LOCAL_TRIM 0x00000800
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_RCOSCLF_VDD_LOCAL_TRIM_SHIFT 11
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_RXTX_MODE 0x00010000
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_RXTX_MODE_SHIFT 16
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_ANA_AMP_CTRL 0x00020000
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_ANA_AMP_CTRL_SHIFT 17
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_CMIRRWR_RATIO 0x003c0000
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_CMIRRWR_RATIO_SHIFT 18
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_CMIRRWR_RATIO_SET(x, v) do { (x) = (((x) & ~0x3c0000) | ((v) << 18)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_CMIRRWR_RATIO_GET(x) (((x) >> 18) & 0xf)
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_REGULATOR_TRIM 0x00c00000
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_REGULATOR_TRIM_SHIFT 22
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_REGULATOR_TRIM_SET(x, v) do { (x) = (((x) & ~0xc00000) | ((v) << 22)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_REGULATOR_TRIM_GET(x) (((x) >> 22) & 0x3)
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_AMP_BOOST 0x01000000
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_AMP_BOOST_SHIFT 24
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_TESTMUX_EN 0x02000000
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_TESTMUX_EN_SHIFT 25
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_BUFFER_TRIM 0x04000000
  #define CC26XX_AUX_DDI0_OSC_LFOSCCTL_XOSCLF_BUFFER_TRIM_SHIFT 26

#define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_ADDR          0x00000030
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_RCOSCHF_ITUNE_TRIM 0x0000000f
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_RCOSCHF_ITUNE_TRIM_SHIFT 0
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_RCOSCHF_ITUNE_TRIM_SET(x, v) do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_RCOSCHF_ITUNE_TRIM_GET(x) (((x) >> 0) & 0xf)
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_RCOSCHF_BYPASS_GATE 0x00000010
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_RCOSCHF_BYPASS_GATE_SHIFT 4
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_RCOSCHF_ATEST_EN 0x00000020
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_RCOSCHF_ATEST_EN_SHIFT 5
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_ATEST_VDD_LOCAL_SEL 0x00000040
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_ATEST_VDD_LOCAL_SEL_SHIFT 6
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_RCOSCHF_CTRIM 0x0000ff00
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_RCOSCHF_CTRIM_SHIFT 8
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_RCOSCHF_CTRIM_SET(x, v) do { (x) = (((x) & ~0xff00) | ((v) << 8)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_RCOSCHFCTL_RCOSCHF_CTRIM_GET(x) (((x) >> 8) & 0xff)

#define CC26XX_AUX_DDI0_OSC_STAT0_ADDR               0x00000034
  #define CC26XX_AUX_DDI0_OSC_STAT0_PENDINGSCLKHFSWITCHING 0x00000001
  #define CC26XX_AUX_DDI0_OSC_STAT0_PENDINGSCLKHFSWITCHING_SHIFT 0
  #define CC26XX_AUX_DDI0_OSC_STAT0_ADC_DATA       0x0000007e
  #define CC26XX_AUX_DDI0_OSC_STAT0_ADC_DATA_SHIFT 1
  #define CC26XX_AUX_DDI0_OSC_STAT0_ADC_DATA_SET(x, v) do { (x) = (((x) & ~0x7e) | ((v) << 1)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_STAT0_ADC_DATA_GET(x) (((x) >> 1) & 0x3f)
  #define CC26XX_AUX_DDI0_OSC_STAT0_ADC_DATA_READY 0x00000080
  #define CC26XX_AUX_DDI0_OSC_STAT0_ADC_DATA_READY_SHIFT 7
  #define CC26XX_AUX_DDI0_OSC_STAT0_ADC_THMET      0x00000100
  #define CC26XX_AUX_DDI0_OSC_STAT0_ADC_THMET_SHIFT 8
  #define CC26XX_AUX_DDI0_OSC_STAT0_RADC_DIG_CLRZ  0x00000200
  #define CC26XX_AUX_DDI0_OSC_STAT0_RADC_DIG_CLRZ_SHIFT 9
  #define CC26XX_AUX_DDI0_OSC_STAT0_XOSC_HF_HP_BUF_EN 0x00000400
  #define CC26XX_AUX_DDI0_OSC_STAT0_XOSC_HF_HP_BUF_EN_SHIFT 10
  #define CC26XX_AUX_DDI0_OSC_STAT0_XOSC_HF_LP_BUF_EN 0x00000800
  #define CC26XX_AUX_DDI0_OSC_STAT0_XOSC_HF_LP_BUF_EN_SHIFT 11
  #define CC26XX_AUX_DDI0_OSC_STAT0_ADC_EN         0x00001000
  #define CC26XX_AUX_DDI0_OSC_STAT0_ADC_EN_SHIFT   12
  #define CC26XX_AUX_DDI0_OSC_STAT0_XB_48M_CLK_EN  0x00002000
  #define CC26XX_AUX_DDI0_OSC_STAT0_XB_48M_CLK_EN_SHIFT 13
  #define CC26XX_AUX_DDI0_OSC_STAT0_XOSC_HF_PEAK_DET_EN 0x00004000
  #define CC26XX_AUX_DDI0_OSC_STAT0_XOSC_HF_PEAK_DET_EN_SHIFT 14
  #define CC26XX_AUX_DDI0_OSC_STAT0_XOSC_HF_EN     0x00008000
  #define CC26XX_AUX_DDI0_OSC_STAT0_XOSC_HF_EN_SHIFT 15
  #define CC26XX_AUX_DDI0_OSC_STAT0_SCLK_LF_LOSS   0x00010000
  #define CC26XX_AUX_DDI0_OSC_STAT0_SCLK_LF_LOSS_SHIFT 16
  #define CC26XX_AUX_DDI0_OSC_STAT0_SCLK_HF_LOSS   0x00020000
  #define CC26XX_AUX_DDI0_OSC_STAT0_SCLK_HF_LOSS_SHIFT 17
  #define CC26XX_AUX_DDI0_OSC_STAT0_CLK_DCDC_RDY_ACK 0x00040000
  #define CC26XX_AUX_DDI0_OSC_STAT0_CLK_DCDC_RDY_ACK_SHIFT 18
  #define CC26XX_AUX_DDI0_OSC_STAT0_CLK_DCDC_RDY   0x00080000
  #define CC26XX_AUX_DDI0_OSC_STAT0_CLK_DCDC_RDY_SHIFT 19
  #define CC26XX_AUX_DDI0_OSC_STAT0_XOSC_LF_EN     0x00100000
  #define CC26XX_AUX_DDI0_OSC_STAT0_XOSC_LF_EN_SHIFT 20
  #define CC26XX_AUX_DDI0_OSC_STAT0_RCOSC_LF_EN    0x00200000
  #define CC26XX_AUX_DDI0_OSC_STAT0_RCOSC_LF_EN_SHIFT 21
  #define CC26XX_AUX_DDI0_OSC_STAT0_RCOSC_HF_EN    0x00400000
  #define CC26XX_AUX_DDI0_OSC_STAT0_RCOSC_HF_EN_SHIFT 22
  #define CC26XX_AUX_DDI0_OSC_STAT0_GBIAS_RDY      0x00800000
  #define CC26XX_AUX_DDI0_OSC_STAT0_GBIAS_RDY_SHIFT 23
  #define CC26XX_AUX_DDI0_OSC_STAT0_GBIAS_NEEDED   0x01000000
  #define CC26XX_AUX_DDI0_OSC_STAT0_GBIAS_NEEDED_SHIFT 24
  #define CC26XX_AUX_DDI0_OSC_STAT0_BGAP_RDY       0x02000000
  #define CC26XX_AUX_DDI0_OSC_STAT0_BGAP_RDY_SHIFT 25
  #define CC26XX_AUX_DDI0_OSC_STAT0_BGAP_NEEDED    0x04000000
  #define CC26XX_AUX_DDI0_OSC_STAT0_BGAP_NEEDED_SHIFT 26
  #define CC26XX_AUX_DDI0_OSC_STAT0_BYPASS_OSCDIG  0x08000000
  #define CC26XX_AUX_DDI0_OSC_STAT0_BYPASS_OSCDIG_SHIFT 27
  #define CC26XX_AUX_DDI0_OSC_STAT0_SCLK_HF_SRC    0x10000000
  #define CC26XX_AUX_DDI0_OSC_STAT0_SCLK_HF_SRC_SHIFT 28
  #define CC26XX_AUX_DDI0_OSC_STAT0_SCLK_LF_SRC    0x60000000
  #define CC26XX_AUX_DDI0_OSC_STAT0_SCLK_LF_SRC_SHIFT 29
  #define CC26XX_AUX_DDI0_OSC_STAT0_SCLK_LF_SRC_SET(x, v) do { (x) = (((x) & ~0x60000000) | ((v) << 29)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_STAT0_SCLK_LF_SRC_GET(x) (((x) >> 29) & 0x3)

#define CC26XX_AUX_DDI0_OSC_STAT1_ADDR               0x00000038
  #define CC26XX_AUX_DDI0_OSC_STAT1_CLK_DCDC_GOOD  0x00000001
  #define CC26XX_AUX_DDI0_OSC_STAT1_CLK_DCDC_GOOD_SHIFT 0
  #define CC26XX_AUX_DDI0_OSC_STAT1_CLK_CHP_GOOD   0x00000002
  #define CC26XX_AUX_DDI0_OSC_STAT1_CLK_CHP_GOOD_SHIFT 1
  #define CC26XX_AUX_DDI0_OSC_STAT1_ACLK_REF_GOOD  0x00000004
  #define CC26XX_AUX_DDI0_OSC_STAT1_ACLK_REF_GOOD_SHIFT 2
  #define CC26XX_AUX_DDI0_OSC_STAT1_ACLK_TDC_GOOD  0x00000008
  #define CC26XX_AUX_DDI0_OSC_STAT1_ACLK_TDC_GOOD_SHIFT 3
  #define CC26XX_AUX_DDI0_OSC_STAT1_ACLK_ADC_GOOD  0x00000010
  #define CC26XX_AUX_DDI0_OSC_STAT1_ACLK_ADC_GOOD_SHIFT 4
  #define CC26XX_AUX_DDI0_OSC_STAT1_SCLK_LF_GOOD   0x00000020
  #define CC26XX_AUX_DDI0_OSC_STAT1_SCLK_LF_GOOD_SHIFT 5
  #define CC26XX_AUX_DDI0_OSC_STAT1_SCLK_MF_GOOD   0x00000040
  #define CC26XX_AUX_DDI0_OSC_STAT1_SCLK_MF_GOOD_SHIFT 6
  #define CC26XX_AUX_DDI0_OSC_STAT1_SCLK_HF_GOOD   0x00000080
  #define CC26XX_AUX_DDI0_OSC_STAT1_SCLK_HF_GOOD_SHIFT 7
  #define CC26XX_AUX_DDI0_OSC_STAT1_CLK_DCDC_EN    0x00000100
  #define CC26XX_AUX_DDI0_OSC_STAT1_CLK_DCDC_EN_SHIFT 8
  #define CC26XX_AUX_DDI0_OSC_STAT1_CLK_CHP_EN     0x00000200
  #define CC26XX_AUX_DDI0_OSC_STAT1_CLK_CHP_EN_SHIFT 9
  #define CC26XX_AUX_DDI0_OSC_STAT1_ACLK_REF_EN    0x00000400
  #define CC26XX_AUX_DDI0_OSC_STAT1_ACLK_REF_EN_SHIFT 10
  #define CC26XX_AUX_DDI0_OSC_STAT1_ACLK_TDC_EN    0x00000800
  #define CC26XX_AUX_DDI0_OSC_STAT1_ACLK_TDC_EN_SHIFT 11
  #define CC26XX_AUX_DDI0_OSC_STAT1_ACLK_ADC_EN    0x00001000
  #define CC26XX_AUX_DDI0_OSC_STAT1_ACLK_ADC_EN_SHIFT 12
  #define CC26XX_AUX_DDI0_OSC_STAT1_SCLK_MF_EN     0x00002000
  #define CC26XX_AUX_DDI0_OSC_STAT1_SCLK_MF_EN_SHIFT 13
  #define CC26XX_AUX_DDI0_OSC_STAT1_SCLK_HF_EN     0x00004000
  #define CC26XX_AUX_DDI0_OSC_STAT1_SCLK_HF_EN_SHIFT 14
  #define CC26XX_AUX_DDI0_OSC_STAT1_FORCE_RCOSC_HF 0x00008000
  #define CC26XX_AUX_DDI0_OSC_STAT1_FORCE_RCOSC_HF_SHIFT 15
  #define CC26XX_AUX_DDI0_OSC_STAT1_LPM_UPDATE_AMP 0x003f0000
  #define CC26XX_AUX_DDI0_OSC_STAT1_LPM_UPDATE_AMP_SHIFT 16
  #define CC26XX_AUX_DDI0_OSC_STAT1_LPM_UPDATE_AMP_SET(x, v) do { (x) = (((x) & ~0x3f0000) | ((v) << 16)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_STAT1_LPM_UPDATE_AMP_GET(x) (((x) >> 16) & 0x3f)
  #define CC26XX_AUX_DDI0_OSC_STAT1_HMP_UPDATE_AMP 0x0fc00000
  #define CC26XX_AUX_DDI0_OSC_STAT1_HMP_UPDATE_AMP_SHIFT 22
  #define CC26XX_AUX_DDI0_OSC_STAT1_HMP_UPDATE_AMP_SET(x, v) do { (x) = (((x) & ~0xfc00000) | ((v) << 22)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_STAT1_HMP_UPDATE_AMP_GET(x) (((x) >> 22) & 0x3f)
  #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE      0xf0000000
  #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_SHIFT 28
  #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_SET(x, v) do { (x) = (((x) & ~0xf0000000) | ((CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_##v) << 28)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_GET(x) (((x) >> 28) & 0xf)
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_RESET 0x00000000
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_IBIAS_INC 0x00000000
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_IDAC_DEC_W_MEASURE 0x00000000
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_DUMMY_TO_INIT_1 0x00000000
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_FAST_START 0x00000000
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_FAST_START_SETTLE 0x00000000
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_INITIALIZATION 0x00000001
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_HPM_RAMP1 0x00000002
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_HPM_RAMP2 0x00000003
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_HPM_RAMP3 0x00000004
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_HPM_UPDATE 0x00000005
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_IDAC_INCREMENT 0x00000006
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_IBIAS_CAP_UPDATE 0x00000007
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_IBIAS_DEC_W_MEASURE 0x00000008
    #define CC26XX_AUX_DDI0_OSC_STAT1_RAMPSTATE_LPM_UPDATE 0x00000009

#define CC26XX_AUX_DDI0_OSC_STAT2_ADDR               0x0000003c
  #define CC26XX_AUX_DDI0_OSC_STAT2_XOSC_HF_RF_FREQGOOD 0x00000001
  #define CC26XX_AUX_DDI0_OSC_STAT2_XOSC_HF_RF_FREQGOOD_SHIFT 0
  #define CC26XX_AUX_DDI0_OSC_STAT2_XOSC_HF_FREQGOOD 0x00000002
  #define CC26XX_AUX_DDI0_OSC_STAT2_XOSC_HF_FREQGOOD_SHIFT 1
  #define CC26XX_AUX_DDI0_OSC_STAT2_XOSC_HF_AMPGOOD 0x00000004
  #define CC26XX_AUX_DDI0_OSC_STAT2_XOSC_HF_AMPGOOD_SHIFT 2
  #define CC26XX_AUX_DDI0_OSC_STAT2_AMPCOMP_REQ    0x00000008
  #define CC26XX_AUX_DDI0_OSC_STAT2_AMPCOMP_REQ_SHIFT 3
  #define CC26XX_AUX_DDI0_OSC_STAT2_AMPCOMP_OF_UF  0x00000070
  #define CC26XX_AUX_DDI0_OSC_STAT2_AMPCOMP_OF_UF_SHIFT 4
  #define CC26XX_AUX_DDI0_OSC_STAT2_AMPCOMP_OF_UF_SET(x, v) do { (x) = (((x) & ~0x70) | ((v) << 4)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_STAT2_AMPCOMP_OF_UF_GET(x) (((x) >> 4) & 0x7)
  #define CC26XX_AUX_DDI0_OSC_STAT2_ADC_COMP_M     0x00000080
  #define CC26XX_AUX_DDI0_OSC_STAT2_ADC_COMP_M_SHIFT 7
  #define CC26XX_AUX_DDI0_OSC_STAT2_ADC_COMP_P     0x00000100
  #define CC26XX_AUX_DDI0_OSC_STAT2_ADC_COMP_P_SHIFT 8
  #define CC26XX_AUX_DDI0_OSC_STAT2_ADCSTATE       0x00000e00
  #define CC26XX_AUX_DDI0_OSC_STAT2_ADCSTATE_SHIFT 9
  #define CC26XX_AUX_DDI0_OSC_STAT2_ADCSTATE_SET(x, v) do { (x) = (((x) & ~0xe00) | ((CC26XX_AUX_DDI0_OSC_STAT2_ADCSTATE_##v) << 9)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_STAT2_ADCSTATE_GET(x) (((x) >> 9) & 0x7)
    #define CC26XX_AUX_DDI0_OSC_STAT2_ADCSTATE_ADC_RESET 0x00000000
    #define CC26XX_AUX_DDI0_OSC_STAT2_ADCSTATE_ADC_IDLE 0x00000001
    #define CC26XX_AUX_DDI0_OSC_STAT2_ADCSTATE_ADC_SC 0x00000002
    #define CC26XX_AUX_DDI0_OSC_STAT2_ADCSTATE_ADC_RDDCB 0x00000003
    #define CC26XX_AUX_DDI0_OSC_STAT2_ADCSTATE_ADC_RDAMP 0x00000004
    #define CC26XX_AUX_DDI0_OSC_STAT2_ADCSTATE_ADC_CALC 0x00000005
  #define CC26XX_AUX_DDI0_OSC_STAT2_RAMPSTATE      0x0000f000
  #define CC26XX_AUX_DDI0_OSC_STAT2_RAMPSTATE_SHIFT 12
  #define CC26XX_AUX_DDI0_OSC_STAT2_RAMPSTATE_SET(x, v) do { (x) = (((x) & ~0xf000) | ((v) << 12)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_STAT2_RAMPSTATE_GET(x) (((x) >> 12) & 0xf)
  #define CC26XX_AUX_DDI0_OSC_STAT2_RAMP_DOWN_TO_INIT_DONE 0x00010000
  #define CC26XX_AUX_DDI0_OSC_STAT2_RAMP_DOWN_TO_INIT_DONE_SHIFT 16
  #define CC26XX_AUX_DDI0_OSC_STAT2_IBIAS_INCREMENT_DONE 0x00020000
  #define CC26XX_AUX_DDI0_OSC_STAT2_IBIAS_INCREMENT_DONE_SHIFT 17
  #define CC26XX_AUX_DDI0_OSC_STAT2_IDAC_DECREMENT_WITH_MEASURE_DONE 0x00040000
  #define CC26XX_AUX_DDI0_OSC_STAT2_IDAC_DECREMENT_WITH_MEASURE_DONE_SHIFT 18
  #define CC26XX_AUX_DDI0_OSC_STAT2_IBIAS_CAP_UPDATE_DONE 0x00080000
  #define CC26XX_AUX_DDI0_OSC_STAT2_IBIAS_CAP_UPDATE_DONE_SHIFT 19
  #define CC26XX_AUX_DDI0_OSC_STAT2_IDAC_INCREMENT_DONE 0x00100000
  #define CC26XX_AUX_DDI0_OSC_STAT2_IDAC_INCREMENT_DONE_SHIFT 20
  #define CC26XX_AUX_DDI0_OSC_STAT2_IBIAS_WAIT_CNTR_DONE 0x00200000
  #define CC26XX_AUX_DDI0_OSC_STAT2_IBIAS_WAIT_CNTR_DONE_SHIFT 21
  #define CC26XX_AUX_DDI0_OSC_STAT2_IBIAS_DEC_WITH_MEASURE_DONE 0x00400000
  #define CC26XX_AUX_DDI0_OSC_STAT2_IBIAS_DEC_WITH_MEASURE_DONE_SHIFT 22
  #define CC26XX_AUX_DDI0_OSC_STAT2_HPM_RAMP3_THMET 0x00800000
  #define CC26XX_AUX_DDI0_OSC_STAT2_HPM_RAMP3_THMET_SHIFT 23
  #define CC26XX_AUX_DDI0_OSC_STAT2_HPM_RAMP2_THMET 0x01000000
  #define CC26XX_AUX_DDI0_OSC_STAT2_HPM_RAMP2_THMET_SHIFT 24
  #define CC26XX_AUX_DDI0_OSC_STAT2_HPM_RAMP1_THMET 0x02000000
  #define CC26XX_AUX_DDI0_OSC_STAT2_HPM_RAMP1_THMET_SHIFT 25
  #define CC26XX_AUX_DDI0_OSC_STAT2_ADC_DCBIAS     0xfc000000
  #define CC26XX_AUX_DDI0_OSC_STAT2_ADC_DCBIAS_SHIFT 26
  #define CC26XX_AUX_DDI0_OSC_STAT2_ADC_DCBIAS_SET(x, v) do { (x) = (((x) & ~0xfc000000) | ((v) << 26)); } while(0)
  #define CC26XX_AUX_DDI0_OSC_STAT2_ADC_DCBIAS_GET(x) (((x) >> 26) & 0x3f)

#endif
