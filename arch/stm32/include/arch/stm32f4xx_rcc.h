/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs cdefs_use_reg_mask=1 cdefs_use_field_set=1                   \
     cdefs_use_field_get=1 reg_prefix=STM32F4xx field_prefix=STM32F4xx         \
     cdefs_use_field_mask=1
*/

#ifndef _RCC_BFGEN_DEFS_
#define _RCC_BFGEN_DEFS_

#define STM32F4xx_RCC_CR_ADDR                        0x00000000
#define STM32F4xx_RCC_CR_MASK                        0x070ffffb
  #define STM32F4xx_RCC_CR_HSI_MASK                0x00000001
  #define STM32F4xx_RCC_CR_HSI(v)                  ((STM32F4xx_RCC_CR_HSI_##v) << 0)
  #define STM32F4xx_RCC_CR_HSI_SET(x, v)           do { (x) = (((x) & ~0x1) | ((STM32F4xx_RCC_CR_HSI_##v) << 0)); } while(0)
  #define STM32F4xx_RCC_CR_HSI_GET(x)              (((x) >> 0) & 0x1)
    #define STM32F4xx_RCC_CR_HSI_OFF                 0x00000000
    #define STM32F4xx_RCC_CR_HSI_ON                  0x00000001
  #define STM32F4xx_RCC_CR_HSIDRY_MASK             0x00000001
  #define STM32F4xx_RCC_CR_HSIDRY                  0x00000002
  #define STM32F4xx_RCC_CR_HSIDRY_SET(x, v)        do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define STM32F4xx_RCC_CR_HSIDRY_GET(x)           (((x) >> 1) & 0x1)
  #define STM32F4xx_RCC_CR_HSITRIM_MASK            0x0000001f
  #define STM32F4xx_RCC_CR_HSITRIM(v)              ((v) << 3)
  #define STM32F4xx_RCC_CR_HSITRIM_SET(x, v)       do { (x) = (((x) & ~0xf8) | ((v) << 3)); } while(0)
  #define STM32F4xx_RCC_CR_HSITRIM_GET(x)          (((x) >> 3) & 0x1f)
  #define STM32F4xx_RCC_CR_HSICAL_MASK             0x000000ff
  #define STM32F4xx_RCC_CR_HSICAL(v)               ((v) << 8)
  #define STM32F4xx_RCC_CR_HSICAL_SET(x, v)        do { (x) = (((x) & ~0xff00) | ((v) << 8)); } while(0)
  #define STM32F4xx_RCC_CR_HSICAL_GET(x)           (((x) >> 8) & 0xff)
  #define STM32F4xx_RCC_CR_HSE_MASK                0x00000001
  #define STM32F4xx_RCC_CR_HSE(v)                  ((STM32F4xx_RCC_CR_HSE_##v) << 16)
  #define STM32F4xx_RCC_CR_HSE_SET(x, v)           do { (x) = (((x) & ~0x10000) | ((STM32F4xx_RCC_CR_HSE_##v) << 16)); } while(0)
  #define STM32F4xx_RCC_CR_HSE_GET(x)              (((x) >> 16) & 0x1)
    #define STM32F4xx_RCC_CR_HSE_OFF                 0x00000000
    #define STM32F4xx_RCC_CR_HSE_ON                  0x00000001
  #define STM32F4xx_RCC_CR_HSERDY_MASK             0x00000001
  #define STM32F4xx_RCC_CR_HSERDY                  0x00020000
  #define STM32F4xx_RCC_CR_HSERDY_SET(x, v)        do { (x) = (((x) & ~0x20000) | ((v) << 17)); } while(0)
  #define STM32F4xx_RCC_CR_HSERDY_GET(x)           (((x) >> 17) & 0x1)
  #define STM32F4xx_RCC_CR_HSEBYP_MASK             0x00000001
  #define STM32F4xx_RCC_CR_HSEBYP                  0x00040000
  #define STM32F4xx_RCC_CR_HSEBYP_SET(x, v)        do { (x) = (((x) & ~0x40000) | ((v) << 18)); } while(0)
  #define STM32F4xx_RCC_CR_HSEBYP_GET(x)           (((x) >> 18) & 0x1)
  #define STM32F4xx_RCC_CR_CSS_MASK                0x00000001
  #define STM32F4xx_RCC_CR_CSS(v)                  ((STM32F4xx_RCC_CR_CSS_##v) << 19)
  #define STM32F4xx_RCC_CR_CSS_SET(x, v)           do { (x) = (((x) & ~0x80000) | ((STM32F4xx_RCC_CR_CSS_##v) << 19)); } while(0)
  #define STM32F4xx_RCC_CR_CSS_GET(x)              (((x) >> 19) & 0x1)
    #define STM32F4xx_RCC_CR_CSS_OFF                 0x00000000
    #define STM32F4xx_RCC_CR_CSS_ON                  0x00000001
  #define STM32F4xx_RCC_CR_PLL_MASK                0x00000001
  #define STM32F4xx_RCC_CR_PLL(v)                  ((STM32F4xx_RCC_CR_PLL_##v) << 24)
  #define STM32F4xx_RCC_CR_PLL_SET(x, v)           do { (x) = (((x) & ~0x1000000) | ((STM32F4xx_RCC_CR_PLL_##v) << 24)); } while(0)
  #define STM32F4xx_RCC_CR_PLL_GET(x)              (((x) >> 24) & 0x1)
    #define STM32F4xx_RCC_CR_PLL_OFF                 0x00000000
    #define STM32F4xx_RCC_CR_PLL_ON                  0x00000001
  #define STM32F4xx_RCC_CR_PLLRDY_MASK             0x00000001
  #define STM32F4xx_RCC_CR_PLLRDY                  0x02000000
  #define STM32F4xx_RCC_CR_PLLRDY_SET(x, v)        do { (x) = (((x) & ~0x2000000) | ((v) << 25)); } while(0)
  #define STM32F4xx_RCC_CR_PLLRDY_GET(x)           (((x) >> 25) & 0x1)
  #define STM32F4xx_RCC_CR_PLLI2S_MASK             0x00000001
  #define STM32F4xx_RCC_CR_PLLI2S(v)               ((STM32F4xx_RCC_CR_PLLI2S_##v) << 26)
  #define STM32F4xx_RCC_CR_PLLI2S_SET(x, v)        do { (x) = (((x) & ~0x4000000) | ((STM32F4xx_RCC_CR_PLLI2S_##v) << 26)); } while(0)
  #define STM32F4xx_RCC_CR_PLLI2S_GET(x)           (((x) >> 26) & 0x1)
    #define STM32F4xx_RCC_CR_PLLI2S_OFF              0x00000000
    #define STM32F4xx_RCC_CR_PLLI2S_ON               0x00000001

#define STM32F4xx_RCC_PLLCFGR_ADDR                   0x00000004
#define STM32F4xx_RCC_PLLCFGR_MASK                   0x07a1ffff
  #define STM32F4xx_RCC_PLLCFGR_M_MASK             0x0000003f
  #define STM32F4xx_RCC_PLLCFGR_M(v)               ((v) << 0)
  #define STM32F4xx_RCC_PLLCFGR_M_SET(x, v)        do { (x) = (((x) & ~0x3f) | ((v) << 0)); } while(0)
  #define STM32F4xx_RCC_PLLCFGR_M_GET(x)           (((x) >> 0) & 0x3f)
  #define STM32F4xx_RCC_PLLCFGR_N_MASK             0x000001ff
  #define STM32F4xx_RCC_PLLCFGR_N(v)               ((v) << 6)
  #define STM32F4xx_RCC_PLLCFGR_N_SET(x, v)        do { (x) = (((x) & ~0x7fc0) | ((v) << 6)); } while(0)
  #define STM32F4xx_RCC_PLLCFGR_N_GET(x)           (((x) >> 6) & 0x1ff)
  #define STM32F4xx_RCC_PLLCFGR_P_MASK             0x00000003
  #define STM32F4xx_RCC_PLLCFGR_P(v)               ((STM32F4xx_RCC_PLLCFGR_P_##v) << 15)
  #define STM32F4xx_RCC_PLLCFGR_P_SET(x, v)        do { (x) = (((x) & ~0x18000) | ((STM32F4xx_RCC_PLLCFGR_P_##v) << 15)); } while(0)
  #define STM32F4xx_RCC_PLLCFGR_P_GET(x)           (((x) >> 15) & 0x3)
    #define STM32F4xx_RCC_PLLCFGR_P_2                0x00000000
    #define STM32F4xx_RCC_PLLCFGR_P_4                0x00000001
    #define STM32F4xx_RCC_PLLCFGR_P_6                0x00000002
    #define STM32F4xx_RCC_PLLCFGR_P_8                0x00000003
  #define STM32F4xx_RCC_PLLCFGR_PLLSRC_MASK        0x00000001
  #define STM32F4xx_RCC_PLLCFGR_PLLSRC(v)          ((STM32F4xx_RCC_PLLCFGR_PLLSRC_##v) << 21)
  #define STM32F4xx_RCC_PLLCFGR_PLLSRC_SET(x, v)   do { (x) = (((x) & ~0x200000) | ((STM32F4xx_RCC_PLLCFGR_PLLSRC_##v) << 21)); } while(0)
  #define STM32F4xx_RCC_PLLCFGR_PLLSRC_GET(x)      (((x) >> 21) & 0x1)
    #define STM32F4xx_RCC_PLLCFGR_PLLSRC_HSE         0x00000000
    #define STM32F4xx_RCC_PLLCFGR_PLLSRC_HSI         0x00000001
  #define STM32F4xx_RCC_PLLCFGR_Q_MASK             0x0000000f
  #define STM32F4xx_RCC_PLLCFGR_Q(v)               ((v) << 23)
  #define STM32F4xx_RCC_PLLCFGR_Q_SET(x, v)        do { (x) = (((x) & ~0x7800000) | ((v) << 23)); } while(0)
  #define STM32F4xx_RCC_PLLCFGR_Q_GET(x)           (((x) >> 23) & 0xf)

#define STM32F4xx_RCC_CFGR_ADDR                      0x00000008
#define STM32F4xx_RCC_CFGR_MASK                      0x3fffffff
  #define STM32F4xx_RCC_CFGR_SW_MASK               0x00000003
  #define STM32F4xx_RCC_CFGR_SW(v)                 ((STM32F4xx_RCC_CFGR_SW_##v) << 0)
  #define STM32F4xx_RCC_CFGR_SW_SET(x, v)          do { (x) = (((x) & ~0x3) | ((STM32F4xx_RCC_CFGR_SW_##v) << 0)); } while(0)
  #define STM32F4xx_RCC_CFGR_SW_GET(x)             (((x) >> 0) & 0x3)
    #define STM32F4xx_RCC_CFGR_SW_HSI                0x00000000
    #define STM32F4xx_RCC_CFGR_SW_HSE                0x00000001
    #define STM32F4xx_RCC_CFGR_SW_PLL                0x00000002
  #define STM32F4xx_RCC_CFGR_SWS_MASK              0x00000003
  #define STM32F4xx_RCC_CFGR_SWS(v)                ((STM32F4xx_RCC_CFGR_SWS_##v) << 2)
  #define STM32F4xx_RCC_CFGR_SWS_SET(x, v)         do { (x) = (((x) & ~0xc) | ((STM32F4xx_RCC_CFGR_SWS_##v) << 2)); } while(0)
  #define STM32F4xx_RCC_CFGR_SWS_GET(x)            (((x) >> 2) & 0x3)
    #define STM32F4xx_RCC_CFGR_SWS_HSI               0x00000000
    #define STM32F4xx_RCC_CFGR_SWS_HSE               0x00000001
    #define STM32F4xx_RCC_CFGR_SWS_PLL               0x00000002
  #define STM32F4xx_RCC_CFGR_HPRE_MASK             0x0000000f
  #define STM32F4xx_RCC_CFGR_HPRE(v)               ((STM32F4xx_RCC_CFGR_HPRE_##v) << 4)
  #define STM32F4xx_RCC_CFGR_HPRE_SET(x, v)        do { (x) = (((x) & ~0xf0) | ((STM32F4xx_RCC_CFGR_HPRE_##v) << 4)); } while(0)
  #define STM32F4xx_RCC_CFGR_HPRE_GET(x)           (((x) >> 4) & 0xf)
    #define STM32F4xx_RCC_CFGR_HPRE_DIV_1            0x00000000
    #define STM32F4xx_RCC_CFGR_HPRE_DIV_2            0x00000008
    #define STM32F4xx_RCC_CFGR_HPRE_DIV_4            0x00000009
    #define STM32F4xx_RCC_CFGR_HPRE_DIV_8            0x0000000a
    #define STM32F4xx_RCC_CFGR_HPRE_DIV_16           0x0000000b
    #define STM32F4xx_RCC_CFGR_HPRE_DIV_64           0x0000000c
    #define STM32F4xx_RCC_CFGR_HPRE_DIV_128          0x0000000d
    #define STM32F4xx_RCC_CFGR_HPRE_DIV_256          0x0000000e
    #define STM32F4xx_RCC_CFGR_HPRE_DIV_512          0x0000000f
  #define STM32F4xx_RCC_CFGR_PPRE1_MASK            0x00000007
  #define STM32F4xx_RCC_CFGR_PPRE1(v)              ((STM32F4xx_RCC_CFGR_PPRE1_##v) << 8)
  #define STM32F4xx_RCC_CFGR_PPRE1_SET(x, v)       do { (x) = (((x) & ~0x700) | ((STM32F4xx_RCC_CFGR_PPRE1_##v) << 8)); } while(0)
  #define STM32F4xx_RCC_CFGR_PPRE1_GET(x)          (((x) >> 8) & 0x7)
    #define STM32F4xx_RCC_CFGR_PPRE1_DIV_1           0x00000000
    #define STM32F4xx_RCC_CFGR_PPRE1_DIV_2           0x00000004
    #define STM32F4xx_RCC_CFGR_PPRE1_DIV_4           0x00000005
    #define STM32F4xx_RCC_CFGR_PPRE1_DIV_8           0x00000006
    #define STM32F4xx_RCC_CFGR_PPRE1_DIV_16          0x00000007
  #define STM32F4xx_RCC_CFGR_PPRE2_MASK            0x00000007
  #define STM32F4xx_RCC_CFGR_PPRE2(v)              ((STM32F4xx_RCC_CFGR_PPRE2_##v) << 11)
  #define STM32F4xx_RCC_CFGR_PPRE2_SET(x, v)       do { (x) = (((x) & ~0x3800) | ((STM32F4xx_RCC_CFGR_PPRE2_##v) << 11)); } while(0)
  #define STM32F4xx_RCC_CFGR_PPRE2_GET(x)          (((x) >> 11) & 0x7)
    #define STM32F4xx_RCC_CFGR_PPRE2_DIV_1           0x00000000
    #define STM32F4xx_RCC_CFGR_PPRE2_DIV_2           0x00000004
    #define STM32F4xx_RCC_CFGR_PPRE2_DIV_4           0x00000005
    #define STM32F4xx_RCC_CFGR_PPRE2_DIV_8           0x00000006
    #define STM32F4xx_RCC_CFGR_PPRE2_DIV_16          0x00000007
  #define STM32F4xx_RCC_CFGR_RTCPRE_MASK           0x0000001f
  #define STM32F4xx_RCC_CFGR_RTCPRE(v)             ((STM32F4xx_RCC_CFGR_RTCPRE_##v) << 14)
  #define STM32F4xx_RCC_CFGR_RTCPRE_SET(x, v)      do { (x) = (((x) & ~0x7c000) | ((STM32F4xx_RCC_CFGR_RTCPRE_##v) << 14)); } while(0)
  #define STM32F4xx_RCC_CFGR_RTCPRE_GET(x)         (((x) >> 14) & 0x1f)
    #define STM32F4xx_RCC_CFGR_RTCPRE_NO_CLOCK       0x00000000
    #define STM32F4xx_RCC_CFGR_RTCPRE_HSE_DIV_2      0x00000002
    #define STM32F4xx_RCC_CFGR_RTCPRE_HSE_DIV_3      0x00000003
    #define STM32F4xx_RCC_CFGR_RTCPRE_HSE_DIV_4      0x00000004
  #define STM32F4xx_RCC_CFGR_MCO1_MASK             0x00000003
  #define STM32F4xx_RCC_CFGR_MCO1(v)               ((STM32F4xx_RCC_CFGR_MCO1_##v) << 19)
  #define STM32F4xx_RCC_CFGR_MCO1_SET(x, v)        do { (x) = (((x) & ~0x180000) | ((STM32F4xx_RCC_CFGR_MCO1_##v) << 19)); } while(0)
  #define STM32F4xx_RCC_CFGR_MCO1_GET(x)           (((x) >> 19) & 0x3)
    #define STM32F4xx_RCC_CFGR_MCO1_HSI              0x00000000
    #define STM32F4xx_RCC_CFGR_MCO1_LSE              0x00000001
    #define STM32F4xx_RCC_CFGR_MCO1_HSE              0x00000002
    #define STM32F4xx_RCC_CFGR_MCO1_PLL              0x00000003
  #define STM32F4xx_RCC_CFGR_I2SSRC_MASK           0x00000001
  #define STM32F4xx_RCC_CFGR_I2SSRC(v)             ((STM32F4xx_RCC_CFGR_I2SSRC_##v) << 21)
  #define STM32F4xx_RCC_CFGR_I2SSRC_SET(x, v)      do { (x) = (((x) & ~0x200000) | ((STM32F4xx_RCC_CFGR_I2SSRC_##v) << 21)); } while(0)
  #define STM32F4xx_RCC_CFGR_I2SSRC_GET(x)         (((x) >> 21) & 0x1)
    #define STM32F4xx_RCC_CFGR_I2SSRC_PLLI2S         0x00000000
    #define STM32F4xx_RCC_CFGR_I2SSRC_EXT_CLOCK      0x00000001
  #define STM32F4xx_RCC_CFGR_MCO1PRE_MASK          0x00000007
  #define STM32F4xx_RCC_CFGR_MCO1PRE(v)            ((STM32F4xx_RCC_CFGR_MCO1PRE_##v) << 22)
  #define STM32F4xx_RCC_CFGR_MCO1PRE_SET(x, v)     do { (x) = (((x) & ~0x1c00000) | ((STM32F4xx_RCC_CFGR_MCO1PRE_##v) << 22)); } while(0)
  #define STM32F4xx_RCC_CFGR_MCO1PRE_GET(x)        (((x) >> 22) & 0x7)
    #define STM32F4xx_RCC_CFGR_MCO1PRE_DIV_1         0x00000000
    #define STM32F4xx_RCC_CFGR_MCO1PRE_DIV_2         0x00000004
    #define STM32F4xx_RCC_CFGR_MCO1PRE_DIV_3         0x00000005
    #define STM32F4xx_RCC_CFGR_MCO1PRE_DIV_4         0x00000006
    #define STM32F4xx_RCC_CFGR_MCO1PRE_DIV_5         0x00000007
  #define STM32F4xx_RCC_CFGR_MCO2PRE_MASK          0x00000007
  #define STM32F4xx_RCC_CFGR_MCO2PRE(v)            ((STM32F4xx_RCC_CFGR_MCO2PRE_##v) << 25)
  #define STM32F4xx_RCC_CFGR_MCO2PRE_SET(x, v)     do { (x) = (((x) & ~0xe000000) | ((STM32F4xx_RCC_CFGR_MCO2PRE_##v) << 25)); } while(0)
  #define STM32F4xx_RCC_CFGR_MCO2PRE_GET(x)        (((x) >> 25) & 0x7)
    #define STM32F4xx_RCC_CFGR_MCO2PRE_DIV_1         0x00000000
    #define STM32F4xx_RCC_CFGR_MCO2PRE_DIV_2         0x00000004
    #define STM32F4xx_RCC_CFGR_MCO2PRE_DIV_3         0x00000005
    #define STM32F4xx_RCC_CFGR_MCO2PRE_DIV_4         0x00000006
    #define STM32F4xx_RCC_CFGR_MCO2PRE_DIV_5         0x00000007
  #define STM32F4xx_RCC_CFGR_MCO2_MASK             0x00000003
  #define STM32F4xx_RCC_CFGR_MCO2(v)               ((STM32F4xx_RCC_CFGR_MCO2_##v) << 28)
  #define STM32F4xx_RCC_CFGR_MCO2_SET(x, v)        do { (x) = (((x) & ~0x30000000) | ((STM32F4xx_RCC_CFGR_MCO2_##v) << 28)); } while(0)
  #define STM32F4xx_RCC_CFGR_MCO2_GET(x)           (((x) >> 28) & 0x3)
    #define STM32F4xx_RCC_CFGR_MCO2_SYS_CLOCK        0x00000000
    #define STM32F4xx_RCC_CFGR_MCO2_PLLI2S           0x00000001
    #define STM32F4xx_RCC_CFGR_MCO2_HSE              0x00000002
    #define STM32F4xx_RCC_CFGR_MCO2_PLL              0x00000003

#define STM32F4xx_RCC_CIR_ADDR                       0x0000000c
#define STM32F4xx_RCC_CIR_MASK                       0x00bf3fbf
  #define STM32F4xx_RCC_CIR_LSIRDY_MASK            0x00000001
  #define STM32F4xx_RCC_CIR_LSIRDY                 0x00000001
  #define STM32F4xx_RCC_CIR_LSIRDY_SET(x, v)       do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define STM32F4xx_RCC_CIR_LSIRDY_GET(x)          (((x) >> 0) & 0x1)
  #define STM32F4xx_RCC_CIR_LSERDY_MASK            0x00000001
  #define STM32F4xx_RCC_CIR_LSERDY                 0x00000002
  #define STM32F4xx_RCC_CIR_LSERDY_SET(x, v)       do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define STM32F4xx_RCC_CIR_LSERDY_GET(x)          (((x) >> 1) & 0x1)
  #define STM32F4xx_RCC_CIR_HSIRDY_MASK            0x00000001
  #define STM32F4xx_RCC_CIR_HSIRDY                 0x00000004
  #define STM32F4xx_RCC_CIR_HSIRDY_SET(x, v)       do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
  #define STM32F4xx_RCC_CIR_HSIRDY_GET(x)          (((x) >> 2) & 0x1)
  #define STM32F4xx_RCC_CIR_HSERDY_MASK            0x00000001
  #define STM32F4xx_RCC_CIR_HSERDY                 0x00000008
  #define STM32F4xx_RCC_CIR_HSERDY_SET(x, v)       do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
  #define STM32F4xx_RCC_CIR_HSERDY_GET(x)          (((x) >> 3) & 0x1)
  #define STM32F4xx_RCC_CIR_PLLRDY_MASK            0x00000001
  #define STM32F4xx_RCC_CIR_PLLRDY                 0x00000010
  #define STM32F4xx_RCC_CIR_PLLRDY_SET(x, v)       do { (x) = (((x) & ~0x10) | ((v) << 4)); } while(0)
  #define STM32F4xx_RCC_CIR_PLLRDY_GET(x)          (((x) >> 4) & 0x1)
  #define STM32F4xx_RCC_CIR_PLLI2SRDY_MASK         0x00000001
  #define STM32F4xx_RCC_CIR_PLLI2SRDY              0x00000020
  #define STM32F4xx_RCC_CIR_PLLI2SRDY_SET(x, v)    do { (x) = (((x) & ~0x20) | ((v) << 5)); } while(0)
  #define STM32F4xx_RCC_CIR_PLLI2SRDY_GET(x)       (((x) >> 5) & 0x1)
  #define STM32F4xx_RCC_CIR_CSSRDY_MASK            0x00000001
  #define STM32F4xx_RCC_CIR_CSSRDY                 0x00000080
  #define STM32F4xx_RCC_CIR_CSSRDY_SET(x, v)       do { (x) = (((x) & ~0x80) | ((v) << 7)); } while(0)
  #define STM32F4xx_RCC_CIR_CSSRDY_GET(x)          (((x) >> 7) & 0x1)
  #define STM32F4xx_RCC_CIR_LSIRDYIE_MASK          0x00000001
  #define STM32F4xx_RCC_CIR_LSIRDYIE               0x00000100
  #define STM32F4xx_RCC_CIR_LSIRDYIE_SET(x, v)     do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define STM32F4xx_RCC_CIR_LSIRDYIE_GET(x)        (((x) >> 8) & 0x1)
  #define STM32F4xx_RCC_CIR_LSERDYIE_MASK          0x00000001
  #define STM32F4xx_RCC_CIR_LSERDYIE               0x00000200
  #define STM32F4xx_RCC_CIR_LSERDYIE_SET(x, v)     do { (x) = (((x) & ~0x200) | ((v) << 9)); } while(0)
  #define STM32F4xx_RCC_CIR_LSERDYIE_GET(x)        (((x) >> 9) & 0x1)
  #define STM32F4xx_RCC_CIR_HSIRDYIE_MASK          0x00000001
  #define STM32F4xx_RCC_CIR_HSIRDYIE               0x00000400
  #define STM32F4xx_RCC_CIR_HSIRDYIE_SET(x, v)     do { (x) = (((x) & ~0x400) | ((v) << 10)); } while(0)
  #define STM32F4xx_RCC_CIR_HSIRDYIE_GET(x)        (((x) >> 10) & 0x1)
  #define STM32F4xx_RCC_CIR_HSERDYIE_MASK          0x00000001
  #define STM32F4xx_RCC_CIR_HSERDYIE               0x00000800
  #define STM32F4xx_RCC_CIR_HSERDYIE_SET(x, v)     do { (x) = (((x) & ~0x800) | ((v) << 11)); } while(0)
  #define STM32F4xx_RCC_CIR_HSERDYIE_GET(x)        (((x) >> 11) & 0x1)
  #define STM32F4xx_RCC_CIR_PLLRDYIE_MASK          0x00000001
  #define STM32F4xx_RCC_CIR_PLLRDYIE               0x00001000
  #define STM32F4xx_RCC_CIR_PLLRDYIE_SET(x, v)     do { (x) = (((x) & ~0x1000) | ((v) << 12)); } while(0)
  #define STM32F4xx_RCC_CIR_PLLRDYIE_GET(x)        (((x) >> 12) & 0x1)
  #define STM32F4xx_RCC_CIR_PLLI2SRDYIE_MASK       0x00000001
  #define STM32F4xx_RCC_CIR_PLLI2SRDYIE            0x00002000
  #define STM32F4xx_RCC_CIR_PLLI2SRDYIE_SET(x, v)  do { (x) = (((x) & ~0x2000) | ((v) << 13)); } while(0)
  #define STM32F4xx_RCC_CIR_PLLI2SRDYIE_GET(x)     (((x) >> 13) & 0x1)
  #define STM32F4xx_RCC_CIR_LSIRDYC_MASK           0x00000001
  #define STM32F4xx_RCC_CIR_LSIRDYC                0x00010000
  #define STM32F4xx_RCC_CIR_LSIRDYC_SET(x, v)      do { (x) = (((x) & ~0x10000) | ((v) << 16)); } while(0)
  #define STM32F4xx_RCC_CIR_LSIRDYC_GET(x)         (((x) >> 16) & 0x1)
  #define STM32F4xx_RCC_CIR_LSERDYC_MASK           0x00000001
  #define STM32F4xx_RCC_CIR_LSERDYC                0x00020000
  #define STM32F4xx_RCC_CIR_LSERDYC_SET(x, v)      do { (x) = (((x) & ~0x20000) | ((v) << 17)); } while(0)
  #define STM32F4xx_RCC_CIR_LSERDYC_GET(x)         (((x) >> 17) & 0x1)
  #define STM32F4xx_RCC_CIR_HSIRDYC_MASK           0x00000001
  #define STM32F4xx_RCC_CIR_HSIRDYC                0x00040000
  #define STM32F4xx_RCC_CIR_HSIRDYC_SET(x, v)      do { (x) = (((x) & ~0x40000) | ((v) << 18)); } while(0)
  #define STM32F4xx_RCC_CIR_HSIRDYC_GET(x)         (((x) >> 18) & 0x1)
  #define STM32F4xx_RCC_CIR_HSERDYC_MASK           0x00000001
  #define STM32F4xx_RCC_CIR_HSERDYC                0x00080000
  #define STM32F4xx_RCC_CIR_HSERDYC_SET(x, v)      do { (x) = (((x) & ~0x80000) | ((v) << 19)); } while(0)
  #define STM32F4xx_RCC_CIR_HSERDYC_GET(x)         (((x) >> 19) & 0x1)
  #define STM32F4xx_RCC_CIR_PLLRDYC_MASK           0x00000001
  #define STM32F4xx_RCC_CIR_PLLRDYC                0x00100000
  #define STM32F4xx_RCC_CIR_PLLRDYC_SET(x, v)      do { (x) = (((x) & ~0x100000) | ((v) << 20)); } while(0)
  #define STM32F4xx_RCC_CIR_PLLRDYC_GET(x)         (((x) >> 20) & 0x1)
  #define STM32F4xx_RCC_CIR_PLLI2SRDYC_MASK        0x00000001
  #define STM32F4xx_RCC_CIR_PLLI2SRDYC             0x00200000
  #define STM32F4xx_RCC_CIR_PLLI2SRDYC_SET(x, v)   do { (x) = (((x) & ~0x200000) | ((v) << 21)); } while(0)
  #define STM32F4xx_RCC_CIR_PLLI2SRDYC_GET(x)      (((x) >> 21) & 0x1)
  #define STM32F4xx_RCC_CIR_CSSC_MASK              0x00000001
  #define STM32F4xx_RCC_CIR_CSSC                   0x00800000
  #define STM32F4xx_RCC_CIR_CSSC_SET(x, v)         do { (x) = (((x) & ~0x800000) | ((v) << 23)); } while(0)
  #define STM32F4xx_RCC_CIR_CSSC_GET(x)            (((x) >> 23) & 0x1)

#define STM32F4xx_RCC_AHB1RSTR_ADDR                  0x00000010
#define STM32F4xx_RCC_AHB1RSTR_MASK                  0x000c109f
  #define STM32F4xx_RCC_AHB1RSTR_GPIOARST_MASK     0x00000001
  #define STM32F4xx_RCC_AHB1RSTR_GPIOARST          0x00000001
  #define STM32F4xx_RCC_AHB1RSTR_GPIOARST_SET(x, v) do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define STM32F4xx_RCC_AHB1RSTR_GPIOARST_GET(x)   (((x) >> 0) & 0x1)
  #define STM32F4xx_RCC_AHB1RSTR_GPIOBRST_MASK     0x00000001
  #define STM32F4xx_RCC_AHB1RSTR_GPIOBRST          0x00000002
  #define STM32F4xx_RCC_AHB1RSTR_GPIOBRST_SET(x, v) do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define STM32F4xx_RCC_AHB1RSTR_GPIOBRST_GET(x)   (((x) >> 1) & 0x1)
  #define STM32F4xx_RCC_AHB1RSTR_GPIOCRST_MASK     0x00000001
  #define STM32F4xx_RCC_AHB1RSTR_GPIOCRST          0x00000004
  #define STM32F4xx_RCC_AHB1RSTR_GPIOCRST_SET(x, v) do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
  #define STM32F4xx_RCC_AHB1RSTR_GPIOCRST_GET(x)   (((x) >> 2) & 0x1)
  #define STM32F4xx_RCC_AHB1RSTR_GPIODRST_MASK     0x00000001
  #define STM32F4xx_RCC_AHB1RSTR_GPIODRST          0x00000008
  #define STM32F4xx_RCC_AHB1RSTR_GPIODRST_SET(x, v) do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
  #define STM32F4xx_RCC_AHB1RSTR_GPIODRST_GET(x)   (((x) >> 3) & 0x1)
  #define STM32F4xx_RCC_AHB1RSTR_GPIOERST_MASK     0x00000001
  #define STM32F4xx_RCC_AHB1RSTR_GPIOERST          0x00000010
  #define STM32F4xx_RCC_AHB1RSTR_GPIOERST_SET(x, v) do { (x) = (((x) & ~0x10) | ((v) << 4)); } while(0)
  #define STM32F4xx_RCC_AHB1RSTR_GPIOERST_GET(x)   (((x) >> 4) & 0x1)
  #define STM32F4xx_RCC_AHB1RSTR_GPIOHRST_MASK     0x00000001
  #define STM32F4xx_RCC_AHB1RSTR_GPIOHRST          0x00000080
  #define STM32F4xx_RCC_AHB1RSTR_GPIOHRST_SET(x, v) do { (x) = (((x) & ~0x80) | ((v) << 7)); } while(0)
  #define STM32F4xx_RCC_AHB1RSTR_GPIOHRST_GET(x)   (((x) >> 7) & 0x1)
  #define STM32F4xx_RCC_AHB1RSTR_CRCRST_MASK       0x00000001
  #define STM32F4xx_RCC_AHB1RSTR_CRCRST            0x00001000
  #define STM32F4xx_RCC_AHB1RSTR_CRCRST_SET(x, v)  do { (x) = (((x) & ~0x1000) | ((v) << 12)); } while(0)
  #define STM32F4xx_RCC_AHB1RSTR_CRCRST_GET(x)     (((x) >> 12) & 0x1)
  #define STM32F4xx_RCC_AHB1RSTR_DMA1RST_MASK      0x00000001
  #define STM32F4xx_RCC_AHB1RSTR_DMA1RST           0x00040000
  #define STM32F4xx_RCC_AHB1RSTR_DMA1RST_SET(x, v) do { (x) = (((x) & ~0x40000) | ((v) << 18)); } while(0)
  #define STM32F4xx_RCC_AHB1RSTR_DMA1RST_GET(x)    (((x) >> 18) & 0x1)
  #define STM32F4xx_RCC_AHB1RSTR_DMA2RST_MASK      0x00000001
  #define STM32F4xx_RCC_AHB1RSTR_DMA2RST           0x00080000
  #define STM32F4xx_RCC_AHB1RSTR_DMA2RST_SET(x, v) do { (x) = (((x) & ~0x80000) | ((v) << 19)); } while(0)
  #define STM32F4xx_RCC_AHB1RSTR_DMA2RST_GET(x)    (((x) >> 19) & 0x1)

#define STM32F4xx_RCC_AHB2RSTR_ADDR                  0x00000014
#define STM32F4xx_RCC_AHB2RSTR_MASK                  0x00000080
  #define STM32F4xx_RCC_AHB2RSTR_OTGFSRST_MASK     0x00000001
  #define STM32F4xx_RCC_AHB2RSTR_OTGFSRST          0x00000080
  #define STM32F4xx_RCC_AHB2RSTR_OTGFSRST_SET(x, v) do { (x) = (((x) & ~0x80) | ((v) << 7)); } while(0)
  #define STM32F4xx_RCC_AHB2RSTR_OTGFSRST_GET(x)   (((x) >> 7) & 0x1)

#define STM32F4xx_RCC_AHB1ENR_ADDR                   0x00000030
#define STM32F4xx_RCC_AHB1ENR_MASK                   0x0060109f
  #define STM32F4xx_RCC_AHB1ENR_GPIOAEN_MASK       0x00000001
  #define STM32F4xx_RCC_AHB1ENR_GPIOAEN            0x00000001
  #define STM32F4xx_RCC_AHB1ENR_GPIOAEN_SET(x, v)  do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define STM32F4xx_RCC_AHB1ENR_GPIOAEN_GET(x)     (((x) >> 0) & 0x1)
  #define STM32F4xx_RCC_AHB1ENR_GPIOBEN_MASK       0x00000001
  #define STM32F4xx_RCC_AHB1ENR_GPIOBEN            0x00000002
  #define STM32F4xx_RCC_AHB1ENR_GPIOBEN_SET(x, v)  do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define STM32F4xx_RCC_AHB1ENR_GPIOBEN_GET(x)     (((x) >> 1) & 0x1)
  #define STM32F4xx_RCC_AHB1ENR_GPIOCEN_MASK       0x00000001
  #define STM32F4xx_RCC_AHB1ENR_GPIOCEN            0x00000004
  #define STM32F4xx_RCC_AHB1ENR_GPIOCEN_SET(x, v)  do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
  #define STM32F4xx_RCC_AHB1ENR_GPIOCEN_GET(x)     (((x) >> 2) & 0x1)
  #define STM32F4xx_RCC_AHB1ENR_GPIODEN_MASK       0x00000001
  #define STM32F4xx_RCC_AHB1ENR_GPIODEN            0x00000008
  #define STM32F4xx_RCC_AHB1ENR_GPIODEN_SET(x, v)  do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
  #define STM32F4xx_RCC_AHB1ENR_GPIODEN_GET(x)     (((x) >> 3) & 0x1)
  #define STM32F4xx_RCC_AHB1ENR_GPIOEEN_MASK       0x00000001
  #define STM32F4xx_RCC_AHB1ENR_GPIOEEN            0x00000010
  #define STM32F4xx_RCC_AHB1ENR_GPIOEEN_SET(x, v)  do { (x) = (((x) & ~0x10) | ((v) << 4)); } while(0)
  #define STM32F4xx_RCC_AHB1ENR_GPIOEEN_GET(x)     (((x) >> 4) & 0x1)
  #define STM32F4xx_RCC_AHB1ENR_GPIOHEN_MASK       0x00000001
  #define STM32F4xx_RCC_AHB1ENR_GPIOHEN            0x00000080
  #define STM32F4xx_RCC_AHB1ENR_GPIOHEN_SET(x, v)  do { (x) = (((x) & ~0x80) | ((v) << 7)); } while(0)
  #define STM32F4xx_RCC_AHB1ENR_GPIOHEN_GET(x)     (((x) >> 7) & 0x1)
  #define STM32F4xx_RCC_AHB1ENR_CRCEN_MASK         0x00000001
  #define STM32F4xx_RCC_AHB1ENR_CRCEN              0x00001000
  #define STM32F4xx_RCC_AHB1ENR_CRCEN_SET(x, v)    do { (x) = (((x) & ~0x1000) | ((v) << 12)); } while(0)
  #define STM32F4xx_RCC_AHB1ENR_CRCEN_GET(x)       (((x) >> 12) & 0x1)
  #define STM32F4xx_RCC_AHB1ENR_DMA1EN_MASK        0x00000001
  #define STM32F4xx_RCC_AHB1ENR_DMA1EN             0x00200000
  #define STM32F4xx_RCC_AHB1ENR_DMA1EN_SET(x, v)   do { (x) = (((x) & ~0x200000) | ((v) << 21)); } while(0)
  #define STM32F4xx_RCC_AHB1ENR_DMA1EN_GET(x)      (((x) >> 21) & 0x1)
  #define STM32F4xx_RCC_AHB1ENR_DMA2EN_MASK        0x00000001
  #define STM32F4xx_RCC_AHB1ENR_DMA2EN             0x00400000
  #define STM32F4xx_RCC_AHB1ENR_DMA2EN_SET(x, v)   do { (x) = (((x) & ~0x400000) | ((v) << 22)); } while(0)
  #define STM32F4xx_RCC_AHB1ENR_DMA2EN_GET(x)      (((x) >> 22) & 0x1)

#define STM32F4xx_RCC_AHB2ENR_ADDR                   0x00000034
#define STM32F4xx_RCC_AHB2ENR_MASK                   0x00000001
  #define STM32F4xx_RCC_AHB2ENR_OTGFSEN_MASK       0x00000001
  #define STM32F4xx_RCC_AHB2ENR_OTGFSEN            0x00000001
  #define STM32F4xx_RCC_AHB2ENR_OTGFSEN_SET(x, v)  do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define STM32F4xx_RCC_AHB2ENR_OTGFSEN_GET(x)     (((x) >> 0) & 0x1)

#define STM32F4xx_RCC_APB1ENR_ADDR                   0x00000040
#define STM32F4xx_RCC_APB1ENR_MASK                   0x10e2c80f
  #define STM32F4xx_RCC_APB1ENR_TIM2EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB1ENR_TIM2EN             0x00000001
  #define STM32F4xx_RCC_APB1ENR_TIM2EN_SET(x, v)   do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define STM32F4xx_RCC_APB1ENR_TIM2EN_GET(x)      (((x) >> 0) & 0x1)
  #define STM32F4xx_RCC_APB1ENR_TIM3EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB1ENR_TIM3EN             0x00000002
  #define STM32F4xx_RCC_APB1ENR_TIM3EN_SET(x, v)   do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define STM32F4xx_RCC_APB1ENR_TIM3EN_GET(x)      (((x) >> 1) & 0x1)
  #define STM32F4xx_RCC_APB1ENR_TIM4EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB1ENR_TIM4EN             0x00000004
  #define STM32F4xx_RCC_APB1ENR_TIM4EN_SET(x, v)   do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
  #define STM32F4xx_RCC_APB1ENR_TIM4EN_GET(x)      (((x) >> 2) & 0x1)
  #define STM32F4xx_RCC_APB1ENR_TIM5EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB1ENR_TIM5EN             0x00000008
  #define STM32F4xx_RCC_APB1ENR_TIM5EN_SET(x, v)   do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
  #define STM32F4xx_RCC_APB1ENR_TIM5EN_GET(x)      (((x) >> 3) & 0x1)
  #define STM32F4xx_RCC_APB1ENR_WWDGEN_MASK        0x00000001
  #define STM32F4xx_RCC_APB1ENR_WWDGEN             0x00000800
  #define STM32F4xx_RCC_APB1ENR_WWDGEN_SET(x, v)   do { (x) = (((x) & ~0x800) | ((v) << 11)); } while(0)
  #define STM32F4xx_RCC_APB1ENR_WWDGEN_GET(x)      (((x) >> 11) & 0x1)
  #define STM32F4xx_RCC_APB1ENR_SPI2EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB1ENR_SPI2EN             0x00004000
  #define STM32F4xx_RCC_APB1ENR_SPI2EN_SET(x, v)   do { (x) = (((x) & ~0x4000) | ((v) << 14)); } while(0)
  #define STM32F4xx_RCC_APB1ENR_SPI2EN_GET(x)      (((x) >> 14) & 0x1)
  #define STM32F4xx_RCC_APB1ENR_SPI3EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB1ENR_SPI3EN             0x00008000
  #define STM32F4xx_RCC_APB1ENR_SPI3EN_SET(x, v)   do { (x) = (((x) & ~0x8000) | ((v) << 15)); } while(0)
  #define STM32F4xx_RCC_APB1ENR_SPI3EN_GET(x)      (((x) >> 15) & 0x1)
  #define STM32F4xx_RCC_APB1ENR_USART2EN_MASK      0x00000001
  #define STM32F4xx_RCC_APB1ENR_USART2EN           0x00020000
  #define STM32F4xx_RCC_APB1ENR_USART2EN_SET(x, v) do { (x) = (((x) & ~0x20000) | ((v) << 17)); } while(0)
  #define STM32F4xx_RCC_APB1ENR_USART2EN_GET(x)    (((x) >> 17) & 0x1)
  #define STM32F4xx_RCC_APB1ENR_I2C1EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB1ENR_I2C1EN             0x00200000
  #define STM32F4xx_RCC_APB1ENR_I2C1EN_SET(x, v)   do { (x) = (((x) & ~0x200000) | ((v) << 21)); } while(0)
  #define STM32F4xx_RCC_APB1ENR_I2C1EN_GET(x)      (((x) >> 21) & 0x1)
  #define STM32F4xx_RCC_APB1ENR_I2C2EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB1ENR_I2C2EN             0x00400000
  #define STM32F4xx_RCC_APB1ENR_I2C2EN_SET(x, v)   do { (x) = (((x) & ~0x400000) | ((v) << 22)); } while(0)
  #define STM32F4xx_RCC_APB1ENR_I2C2EN_GET(x)      (((x) >> 22) & 0x1)
  #define STM32F4xx_RCC_APB1ENR_I2C3EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB1ENR_I2C3EN             0x00800000
  #define STM32F4xx_RCC_APB1ENR_I2C3EN_SET(x, v)   do { (x) = (((x) & ~0x800000) | ((v) << 23)); } while(0)
  #define STM32F4xx_RCC_APB1ENR_I2C3EN_GET(x)      (((x) >> 23) & 0x1)
  #define STM32F4xx_RCC_APB1ENR_PWREN_MASK         0x00000001
  #define STM32F4xx_RCC_APB1ENR_PWREN              0x10000000
  #define STM32F4xx_RCC_APB1ENR_PWREN_SET(x, v)    do { (x) = (((x) & ~0x10000000) | ((v) << 28)); } while(0)
  #define STM32F4xx_RCC_APB1ENR_PWREN_GET(x)       (((x) >> 28) & 0x1)

#define STM32F4xx_RCC_APB2ENR_ADDR                   0x00000044
#define STM32F4xx_RCC_APB2ENR_MASK                   0x00077931
  #define STM32F4xx_RCC_APB2ENR_TIM1EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB2ENR_TIM1EN             0x00000001
  #define STM32F4xx_RCC_APB2ENR_TIM1EN_SET(x, v)   do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define STM32F4xx_RCC_APB2ENR_TIM1EN_GET(x)      (((x) >> 0) & 0x1)
  #define STM32F4xx_RCC_APB2ENR_USART1EN_MASK      0x00000001
  #define STM32F4xx_RCC_APB2ENR_USART1EN           0x00000010
  #define STM32F4xx_RCC_APB2ENR_USART1EN_SET(x, v) do { (x) = (((x) & ~0x10) | ((v) << 4)); } while(0)
  #define STM32F4xx_RCC_APB2ENR_USART1EN_GET(x)    (((x) >> 4) & 0x1)
  #define STM32F4xx_RCC_APB2ENR_USART6EN_MASK      0x00000001
  #define STM32F4xx_RCC_APB2ENR_USART6EN           0x00000020
  #define STM32F4xx_RCC_APB2ENR_USART6EN_SET(x, v) do { (x) = (((x) & ~0x20) | ((v) << 5)); } while(0)
  #define STM32F4xx_RCC_APB2ENR_USART6EN_GET(x)    (((x) >> 5) & 0x1)
  #define STM32F4xx_RCC_APB2ENR_ADC1EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB2ENR_ADC1EN             0x00000100
  #define STM32F4xx_RCC_APB2ENR_ADC1EN_SET(x, v)   do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define STM32F4xx_RCC_APB2ENR_ADC1EN_GET(x)      (((x) >> 8) & 0x1)
  #define STM32F4xx_RCC_APB2ENR_SDIOEN_MASK        0x00000001
  #define STM32F4xx_RCC_APB2ENR_SDIOEN             0x00000800
  #define STM32F4xx_RCC_APB2ENR_SDIOEN_SET(x, v)   do { (x) = (((x) & ~0x800) | ((v) << 11)); } while(0)
  #define STM32F4xx_RCC_APB2ENR_SDIOEN_GET(x)      (((x) >> 11) & 0x1)
  #define STM32F4xx_RCC_APB2ENR_SPI1EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB2ENR_SPI1EN             0x00001000
  #define STM32F4xx_RCC_APB2ENR_SPI1EN_SET(x, v)   do { (x) = (((x) & ~0x1000) | ((v) << 12)); } while(0)
  #define STM32F4xx_RCC_APB2ENR_SPI1EN_GET(x)      (((x) >> 12) & 0x1)
  #define STM32F4xx_RCC_APB2ENR_SPI4EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB2ENR_SPI4EN             0x00002000
  #define STM32F4xx_RCC_APB2ENR_SPI4EN_SET(x, v)   do { (x) = (((x) & ~0x2000) | ((v) << 13)); } while(0)
  #define STM32F4xx_RCC_APB2ENR_SPI4EN_GET(x)      (((x) >> 13) & 0x1)
  #define STM32F4xx_RCC_APB2ENR_SYSCFGEN_MASK      0x00000001
  #define STM32F4xx_RCC_APB2ENR_SYSCFGEN           0x00004000
  #define STM32F4xx_RCC_APB2ENR_SYSCFGEN_SET(x, v) do { (x) = (((x) & ~0x4000) | ((v) << 14)); } while(0)
  #define STM32F4xx_RCC_APB2ENR_SYSCFGEN_GET(x)    (((x) >> 14) & 0x1)
  #define STM32F4xx_RCC_APB2ENR_TIM9EN_MASK        0x00000001
  #define STM32F4xx_RCC_APB2ENR_TIM9EN             0x00010000
  #define STM32F4xx_RCC_APB2ENR_TIM9EN_SET(x, v)   do { (x) = (((x) & ~0x10000) | ((v) << 16)); } while(0)
  #define STM32F4xx_RCC_APB2ENR_TIM9EN_GET(x)      (((x) >> 16) & 0x1)
  #define STM32F4xx_RCC_APB2ENR_TIM10EN_MASK       0x00000001
  #define STM32F4xx_RCC_APB2ENR_TIM10EN            0x00020000
  #define STM32F4xx_RCC_APB2ENR_TIM10EN_SET(x, v)  do { (x) = (((x) & ~0x20000) | ((v) << 17)); } while(0)
  #define STM32F4xx_RCC_APB2ENR_TIM10EN_GET(x)     (((x) >> 17) & 0x1)
  #define STM32F4xx_RCC_APB2ENR_TIM11EN_MASK       0x00000001
  #define STM32F4xx_RCC_APB2ENR_TIM11EN            0x00040000
  #define STM32F4xx_RCC_APB2ENR_TIM11EN_SET(x, v)  do { (x) = (((x) & ~0x40000) | ((v) << 18)); } while(0)
  #define STM32F4xx_RCC_APB2ENR_TIM11EN_GET(x)     (((x) >> 18) & 0x1)

#endif

