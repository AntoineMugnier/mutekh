/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs -I defs/cc26xx/aon_rtc.bf cdefs_use_reg_comment=0            \
     cdefs_use_field_comment=0 cdefs_use_field_get=0 cdefs_use_field_set=0     \
     cdefs_use_field_shift=0 cdefs_use_field_shifted_mask=2                    \
     cdefs_use_field_shifter=2 cdefs_use_reg_comment=0                         \
     cdefs_use_value_comment=0 doc_field_doc_column=0                          \
     doc_field_longname_column=0 doc_lsb_on_left=0 doc_reg_address_column=0    \
     doc_reg_direction_column=0 doc_reg_doc_column=0 doc_reg_longname_column=0 \
     doc_split_width=0
*/

#ifndef _CC26XX_AON_RTC_BFGEN_DEFS_
#define _CC26XX_AON_RTC_BFGEN_DEFS_

#define CC26XX_AON_RTC_CTL_ADDR                      0x00000000
  #define CC26XX_AON_RTC_CTL_EN                    0x00000001
  #define CC26XX_AON_RTC_CTL_RTC_UPD_EN            0x00000002
  #define CC26XX_AON_RTC_CTL_RTC_4KHZ_EN           0x00000004
  #define CC26XX_AON_RTC_CTL_RESET                 0x00000080
  #define CC26XX_AON_RTC_CTL_EV_DELAY(v)           ((CC26XX_AON_RTC_CTL_EV_DELAY_##v) << 8)
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D0           0x00000000
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D96          0x00000000
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D112         0x00000000
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D128         0x00000000
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D144         0x00000000
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D1           0x00000001
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D2           0x00000002
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D4           0x00000003
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D8           0x00000004
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D16          0x00000005
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D32          0x00000006
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D48          0x00000007
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D64          0x00000008
    #define CC26XX_AON_RTC_CTL_EV_DELAY_D80          0x00000009
  #define CC26XX_AON_RTC_CTL_COMB_EV_MASK(v)       ((CC26XX_AON_RTC_CTL_COMB_EV_MASK_##v) << 16)
    #define CC26XX_AON_RTC_CTL_COMB_EV_MASK_NONE     0x00000000
    #define CC26XX_AON_RTC_CTL_COMB_EV_MASK_CH0      0x00000001
    #define CC26XX_AON_RTC_CTL_COMB_EV_MASK_CH1      0x00000002
    #define CC26XX_AON_RTC_CTL_COMB_EV_MASK_CH2      0x00000004

#define CC26XX_AON_RTC_EVFLAGS_ADDR                  0x00000004
  #define CC26XX_AON_RTC_EVFLAGS_CH0               0x00000001
  #define CC26XX_AON_RTC_EVFLAGS_CH1               0x00000100
  #define CC26XX_AON_RTC_EVFLAGS_CH2               0x00010000

#define CC26XX_AON_RTC_SEC_ADDR                      0x00000008
  #define CC26XX_AON_RTC_SEC_VALUE(v)              ((v) << 0)

#define CC26XX_AON_RTC_SUBSEC_ADDR                   0x0000000c
  #define CC26XX_AON_RTC_SUBSEC_VALUE(v)           ((v) << 0)

#define CC26XX_AON_RTC_SUBSECINC_ADDR                0x00000010
  #define CC26XX_AON_RTC_SUBSECINC_VALUEINC(v)     ((v) << 0)

#define CC26XX_AON_RTC_CHCTL_ADDR                    0x00000014
  #define CC26XX_AON_RTC_CHCTL_CH0_EN              0x00000001
  #define CC26XX_AON_RTC_CHCTL_CH1_EN              0x00000100
  #define CC26XX_AON_RTC_CHCTL_CH1_CAPT_EN         0x00000200
  #define CC26XX_AON_RTC_CHCTL_CH2_EN              0x00010000
  #define CC26XX_AON_RTC_CHCTL_CH2_CONT_EN         0x00040000

#define CC26XX_AON_RTC_CH0CMP_ADDR                   0x00000018
  #define CC26XX_AON_RTC_CH0CMP_VALUE(v)           ((v) << 0)

#define CC26XX_AON_RTC_CH1CMP_ADDR                   0x0000001c
  #define CC26XX_AON_RTC_CH1CMP_VALUE(v)           ((v) << 0)

#define CC26XX_AON_RTC_CH2CMP_ADDR                   0x00000020
  #define CC26XX_AON_RTC_CH2CMP_VALUE(v)           ((v) << 0)

#define CC26XX_AON_RTC_CH2CMPINC_ADDR                0x00000024
  #define CC26XX_AON_RTC_CH2CMPINC_VALUE(v)        ((v) << 0)

#define CC26XX_AON_RTC_CH1CAPT_ADDR                  0x00000028
  #define CC26XX_AON_RTC_CH1CAPT_SUBSEC(v)         ((v) << 0)
  #define CC26XX_AON_RTC_CH1CAPT_SEC(v)            ((v) << 16)

#define CC26XX_AON_RTC_SYNC_ADDR                     0x0000002c
  #define CC26XX_AON_RTC_SYNC_WBUSY                0x00000001

#endif

