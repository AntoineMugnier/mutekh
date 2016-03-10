/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs cdefs_use_reg_mask=1 cdefs_use_field_set=1                   \
     cdefs_use_field_get=1 reg_prefix=STM32 field_prefix=STM32                 \
     cdefs_use_field_mask=1
*/

#ifndef _AFIO_BFGEN_DEFS_
#define _AFIO_BFGEN_DEFS_

#define STM32_AFIO_EVCR_ADDR                         0x00000000
#define STM32_AFIO_EVCR_MASK                         0x000000ff
  #define STM32_AFIO_EVCR_PIN_MASK                 0x0000000f
  #define STM32_AFIO_EVCR_PIN(v)                   ((v) << 0)
  #define STM32_AFIO_EVCR_PIN_SET(x, v)            do { (x) = (((x) & ~0xf) | ((v) << 0)); } while(0)
  #define STM32_AFIO_EVCR_PIN_GET(x)               (((x) >> 0) & 0xf)
  #define STM32_AFIO_EVCR_PORT_MASK                0x00000007
  #define STM32_AFIO_EVCR_PORT(v)                  ((v) << 4)
  #define STM32_AFIO_EVCR_PORT_SET(x, v)           do { (x) = (((x) & ~0x70) | ((v) << 4)); } while(0)
  #define STM32_AFIO_EVCR_PORT_GET(x)              (((x) >> 4) & 0x7)
  #define STM32_AFIO_EVCR_EVOE_MASK                0x00000001
  #define STM32_AFIO_EVCR_EVOE                     0x00000080
  #define STM32_AFIO_EVCR_EVOE_SET(x, v)           do { (x) = (((x) & ~0x80) | ((v) << 7)); } while(0)
  #define STM32_AFIO_EVCR_EVOE_GET(x)              (((x) >> 7) & 0x1)

#define STM32_AFIO_MAPR_ADDR                         0x00000004
#define STM32_AFIO_MAPR_MASK                         0x0038ffff
  #define STM32_AFIO_MAPR_SPI1_REMAP_MASK          0x00000001
  #define STM32_AFIO_MAPR_SPI1_REMAP               0x00000001
  #define STM32_AFIO_MAPR_SPI1_REMAP_SET(x, v)     do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define STM32_AFIO_MAPR_SPI1_REMAP_GET(x)        (((x) >> 0) & 0x1)
  #define STM32_AFIO_MAPR_I2C1_REMAP_MASK          0x00000001
  #define STM32_AFIO_MAPR_I2C1_REMAP               0x00000002
  #define STM32_AFIO_MAPR_I2C1_REMAP_SET(x, v)     do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define STM32_AFIO_MAPR_I2C1_REMAP_GET(x)        (((x) >> 1) & 0x1)
  #define STM32_AFIO_MAPR_USART1_REMAP_MASK        0x00000001
  #define STM32_AFIO_MAPR_USART1_REMAP             0x00000004
  #define STM32_AFIO_MAPR_USART1_REMAP_SET(x, v)   do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
  #define STM32_AFIO_MAPR_USART1_REMAP_GET(x)      (((x) >> 2) & 0x1)
  #define STM32_AFIO_MAPR_USART2_REMAP_MASK        0x00000001
  #define STM32_AFIO_MAPR_USART2_REMAP             0x00000008
  #define STM32_AFIO_MAPR_USART2_REMAP_SET(x, v)   do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
  #define STM32_AFIO_MAPR_USART2_REMAP_GET(x)      (((x) >> 3) & 0x1)
  #define STM32_AFIO_MAPR_USART3_REMAP_MASK        0x00000001
  #define STM32_AFIO_MAPR_USART3_REMAP             0x00000010
  #define STM32_AFIO_MAPR_USART3_REMAP_SET(x, v)   do { (x) = (((x) & ~0x10) | ((v) << 4)); } while(0)
  #define STM32_AFIO_MAPR_USART3_REMAP_GET(x)      (((x) >> 4) & 0x1)
  #define STM32_AFIO_MAPR_TIM1_REMAP_MASK          0x00000001
  #define STM32_AFIO_MAPR_TIM1_REMAP               0x00000020
  #define STM32_AFIO_MAPR_TIM1_REMAP_SET(x, v)     do { (x) = (((x) & ~0x20) | ((v) << 5)); } while(0)
  #define STM32_AFIO_MAPR_TIM1_REMAP_GET(x)        (((x) >> 5) & 0x1)
  #define STM32_AFIO_MAPR_TIM2_REMAP_MASK          0x00000001
  #define STM32_AFIO_MAPR_TIM2_REMAP               0x00000040
  #define STM32_AFIO_MAPR_TIM2_REMAP_SET(x, v)     do { (x) = (((x) & ~0x40) | ((v) << 6)); } while(0)
  #define STM32_AFIO_MAPR_TIM2_REMAP_GET(x)        (((x) >> 6) & 0x1)
  #define STM32_AFIO_MAPR_TIM3_REMAP_MASK          0x00000001
  #define STM32_AFIO_MAPR_TIM3_REMAP               0x00000080
  #define STM32_AFIO_MAPR_TIM3_REMAP_SET(x, v)     do { (x) = (((x) & ~0x80) | ((v) << 7)); } while(0)
  #define STM32_AFIO_MAPR_TIM3_REMAP_GET(x)        (((x) >> 7) & 0x1)
  #define STM32_AFIO_MAPR_TIM4_REMAP_MASK          0x00000001
  #define STM32_AFIO_MAPR_TIM4_REMAP               0x00000100
  #define STM32_AFIO_MAPR_TIM4_REMAP_SET(x, v)     do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define STM32_AFIO_MAPR_TIM4_REMAP_GET(x)        (((x) >> 8) & 0x1)
  #define STM32_AFIO_MAPR_CAN1_REMAP_MASK          0x00000001
  #define STM32_AFIO_MAPR_CAN1_REMAP               0x00000200
  #define STM32_AFIO_MAPR_CAN1_REMAP_SET(x, v)     do { (x) = (((x) & ~0x200) | ((v) << 9)); } while(0)
  #define STM32_AFIO_MAPR_CAN1_REMAP_GET(x)        (((x) >> 9) & 0x1)
  #define STM32_AFIO_MAPR_PD01_REMAP_MASK          0x00000001
  #define STM32_AFIO_MAPR_PD01_REMAP               0x00000400
  #define STM32_AFIO_MAPR_PD01_REMAP_SET(x, v)     do { (x) = (((x) & ~0x400) | ((v) << 10)); } while(0)
  #define STM32_AFIO_MAPR_PD01_REMAP_GET(x)        (((x) >> 10) & 0x1)
  #define STM32_AFIO_MAPR_TIM5CH4_IREMAP_MASK      0x00000001
  #define STM32_AFIO_MAPR_TIM5CH4_IREMAP           0x00000800
  #define STM32_AFIO_MAPR_TIM5CH4_IREMAP_SET(x, v) do { (x) = (((x) & ~0x800) | ((v) << 11)); } while(0)
  #define STM32_AFIO_MAPR_TIM5CH4_IREMAP_GET(x)    (((x) >> 11) & 0x1)
  #define STM32_AFIO_MAPR_ADC1_ETRGINJ_REMAP_MASK  0x00000001
  #define STM32_AFIO_MAPR_ADC1_ETRGINJ_REMAP       0x00001000
  #define STM32_AFIO_MAPR_ADC1_ETRGINJ_REMAP_SET(x, v) do { (x) = (((x) & ~0x1000) | ((v) << 12)); } while(0)
  #define STM32_AFIO_MAPR_ADC1_ETRGINJ_REMAP_GET(x) (((x) >> 12) & 0x1)
  #define STM32_AFIO_MAPR_ADC1_ETRGREG_REMAP_MASK  0x00000001
  #define STM32_AFIO_MAPR_ADC1_ETRGREG_REMAP       0x00002000
  #define STM32_AFIO_MAPR_ADC1_ETRGREG_REMAP_SET(x, v) do { (x) = (((x) & ~0x2000) | ((v) << 13)); } while(0)
  #define STM32_AFIO_MAPR_ADC1_ETRGREG_REMAP_GET(x) (((x) >> 13) & 0x1)
  #define STM32_AFIO_MAPR_ADC2_ETRGINJ_REMAP_MASK  0x00000001
  #define STM32_AFIO_MAPR_ADC2_ETRGINJ_REMAP       0x00004000
  #define STM32_AFIO_MAPR_ADC2_ETRGINJ_REMAP_SET(x, v) do { (x) = (((x) & ~0x4000) | ((v) << 14)); } while(0)
  #define STM32_AFIO_MAPR_ADC2_ETRGINJ_REMAP_GET(x) (((x) >> 14) & 0x1)
  #define STM32_AFIO_MAPR_ADC2_ETRGREG_REMAP_MASK  0x00000001
  #define STM32_AFIO_MAPR_ADC2_ETRGREG_REMAP       0x00008000
  #define STM32_AFIO_MAPR_ADC2_ETRGREG_REMAP_SET(x, v) do { (x) = (((x) & ~0x8000) | ((v) << 15)); } while(0)
  #define STM32_AFIO_MAPR_ADC2_ETRGREG_REMAP_GET(x) (((x) >> 15) & 0x1)
  #define STM32_AFIO_MAPR_SWJ_REMAP_MASK           0x00000007
  #define STM32_AFIO_MAPR_SWJ_REMAP(v)             ((STM32_AFIO_MAPR_SWJ_REMAP_##v) << 19)
  #define STM32_AFIO_MAPR_SWJ_REMAP_SET(x, v)      do { (x) = (((x) & ~0x380000) | ((STM32_AFIO_MAPR_SWJ_REMAP_##v) << 19)); } while(0)
  #define STM32_AFIO_MAPR_SWJ_REMAP_GET(x)         (((x) >> 19) & 0x7)
    #define STM32_AFIO_MAPR_SWJ_REMAP_FULL_SWJ       0x00000000
    #define STM32_AFIO_MAPR_SWJ_REMAP_FULL_SWJ_NO_NJTRST 0x00000001
    #define STM32_AFIO_MAPR_SWJ_REMAP_SW_DP_ENABLED  0x00000002
    #define STM32_AFIO_MAPR_SWJ_REMAP_SW_DP_DISABLED 0x00000004

#define STM32_AFIO_EXTICR_ADDR(ridx)                 (0x00000008 + (ridx) * 4)
#define STM32_AFIO_EXTICR_COUNT                      4
#define STM32_AFIO_EXTICR_MASK                       0x0000ffff
  #define STM32_AFIO_EXTICR_EXTI_MASK              0x0000000f
  #define STM32_AFIO_EXTICR_EXTI_COUNT             4
  #define STM32_AFIO_EXTICR_EXTI(fidx, v)          ((v) << ((fidx) * 4 + 0))
  #define STM32_AFIO_EXTICR_EXTI_SET(fidx, x, v)   do { (x) = (((x) & ~(0xf << ((fidx) * 4))) | ((v) << ((fidx) * 4 + 0))); } while(0)
  #define STM32_AFIO_EXTICR_EXTI_GET(fidx, x)      (((x) >> ((fidx) * 4 + 0)) & 0xf)

#define STM32_AFIO_MAPR2_ADDR                        0x0000000c
#define STM32_AFIO_MAPR2_MASK                        0x000007e0
  #define STM32_AFIO_MAPR2_TIM9_REMAP_MASK         0x00000001
  #define STM32_AFIO_MAPR2_TIM9_REMAP              0x00000020
  #define STM32_AFIO_MAPR2_TIM9_REMAP_SET(x, v)    do { (x) = (((x) & ~0x20) | ((v) << 5)); } while(0)
  #define STM32_AFIO_MAPR2_TIM9_REMAP_GET(x)       (((x) >> 5) & 0x1)
  #define STM32_AFIO_MAPR2_TIM10_REMAP_MASK        0x00000001
  #define STM32_AFIO_MAPR2_TIM10_REMAP             0x00000040
  #define STM32_AFIO_MAPR2_TIM10_REMAP_SET(x, v)   do { (x) = (((x) & ~0x40) | ((v) << 6)); } while(0)
  #define STM32_AFIO_MAPR2_TIM10_REMAP_GET(x)      (((x) >> 6) & 0x1)
  #define STM32_AFIO_MAPR2_TIM11_REMAP_MASK        0x00000001
  #define STM32_AFIO_MAPR2_TIM11_REMAP             0x00000080
  #define STM32_AFIO_MAPR2_TIM11_REMAP_SET(x, v)   do { (x) = (((x) & ~0x80) | ((v) << 7)); } while(0)
  #define STM32_AFIO_MAPR2_TIM11_REMAP_GET(x)      (((x) >> 7) & 0x1)
  #define STM32_AFIO_MAPR2_TIM13_REMAP_MASK        0x00000001
  #define STM32_AFIO_MAPR2_TIM13_REMAP             0x00000100
  #define STM32_AFIO_MAPR2_TIM13_REMAP_SET(x, v)   do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define STM32_AFIO_MAPR2_TIM13_REMAP_GET(x)      (((x) >> 8) & 0x1)
  #define STM32_AFIO_MAPR2_TIM14_REMAP_MASK        0x00000001
  #define STM32_AFIO_MAPR2_TIM14_REMAP             0x00000200
  #define STM32_AFIO_MAPR2_TIM14_REMAP_SET(x, v)   do { (x) = (((x) & ~0x200) | ((v) << 9)); } while(0)
  #define STM32_AFIO_MAPR2_TIM14_REMAP_GET(x)      (((x) >> 9) & 0x1)
  #define STM32_AFIO_MAPR2_FSMC_NADV_MASK          0x00000001
  #define STM32_AFIO_MAPR2_FSMC_NADV               0x00000400
  #define STM32_AFIO_MAPR2_FSMC_NADV_SET(x, v)     do { (x) = (((x) & ~0x400) | ((v) << 10)); } while(0)
  #define STM32_AFIO_MAPR2_FSMC_NADV_GET(x)        (((x) >> 10) & 0x1)

#endif
