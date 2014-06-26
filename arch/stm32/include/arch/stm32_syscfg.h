/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs cdefs_use_reg_mask=1 cdefs_use_field_set=1                   \
     reg_prefix=STM32 field_prefix=STM32 cdefs_use_field_mask=1        \
     cdefs_use_field_get=1
*/

#ifndef _SYSCFG_BFGEN_DEFS_
#define _SYSCFG_BFGEN_DEFS_

#define STM32_SYSCFG_MEMRMP_ADDR                 0x00000000
#define STM32_SYSCFG_MEMRMP_MASK                 0x00000003
  #define STM32_SYSCFG_MEMRMP_MEM_MODE_MASK    0x00000003
  #define STM32_SYSCFG_MEMRMP_MEM_MODE(v)      ((STM32_SYSCFG_MEMRMP_MEM_MODE_##v) << 0)
  #define STM32_SYSCFG_MEMRMP_MEM_MODE_SET(x, v) do { (x) = (((x) & ~0x3) | ((STM32_SYSCFG_MEMRMP_MEM_MODE_##v) << 0)); } while(0)
  #define STM32_SYSCFG_MEMRMP_MEM_MODE_GET(x)  (((x) >> 0) & 0x3)
    #define STM32_SYSCFG_MEMRMP_MEM_MODE_FLASH   0x00000000
    #define STM32_SYSCFG_MEMRMP_MEM_MODE_SYSTEM  0x00000001
    #define STM32_SYSCFG_MEMRMP_MEM_MODE_SRAM    0x00000003

#define STM32_SYSCFG_PMC_ADDR                    0x00000004
#define STM32_SYSCFG_PMC_MASK                    0x00000001
  #define STM32_SYSCFG_PMC_ADC1DC2_MASK        0x00000001
  #define STM32_SYSCFG_PMC_ADC1DC2             0x00000001
  #define STM32_SYSCFG_PMC_ADC1DC2_SET(x, v)   do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define STM32_SYSCFG_PMC_ADC1DC2_GET(x)      (((x) >> 0) & 0x1)

#define STM32_SYSCFG_EXTICR_ADDR(ridx)           (0x00000008 + (ridx) * 4)
#define STM32_SYSCFG_EXTICR_COUNT                4
#define STM32_SYSCFG_EXTICR_MASK                 0x0000ffff
  #define STM32_SYSCFG_EXTICR_EXTI_MASK        0x0000000f
  #define STM32_SYSCFG_EXTICR_EXTI_COUNT       4
  #define STM32_SYSCFG_EXTICR_EXTI(fidx, v)    ((v) << ((fidx) * 4 + 0))
  #define STM32_SYSCFG_EXTICR_EXTI_SET(fidx, x, v) do { (x) = (((x) & ~(0xf << ((fidx) * 4))) | ((v) << ((fidx) * 4 + 0))); } while(0)
  #define STM32_SYSCFG_EXTICR_EXTI_GET(fidx, x) (((x) >> ((fidx) * 4 + 0)) & 0xf)

#define STM32_SYSCFG_CMPCR_ADDR                  0x00000020
#define STM32_SYSCFG_CMPCR_MASK                  0x00000003
  #define STM32_SYSCFG_CMPCR_CMP_PD_MASK       0x00000001
  #define STM32_SYSCFG_CMPCR_CMP_PD(v)         ((STM32_SYSCFG_CMPCR_CMP_PD_##v) << 0)
  #define STM32_SYSCFG_CMPCR_CMP_PD_SET(x, v)  do { (x) = (((x) & ~0x1) | ((STM32_SYSCFG_CMPCR_CMP_PD_##v) << 0)); } while(0)
  #define STM32_SYSCFG_CMPCR_CMP_PD_GET(x)     (((x) >> 0) & 0x1)
    #define STM32_SYSCFG_CMPCR_CMP_PD_ON         0x00000000
    #define STM32_SYSCFG_CMPCR_CMP_PD_POWER_DOWN 0x00000001
  #define STM32_SYSCFG_CMPCR_READY_MASK        0x00000001
  #define STM32_SYSCFG_CMPCR_READY             0x00000002
  #define STM32_SYSCFG_CMPCR_READY_SET(x, v)   do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define STM32_SYSCFG_CMPCR_READY_GET(x)      (((x) >> 1) & 0x1)

#endif

