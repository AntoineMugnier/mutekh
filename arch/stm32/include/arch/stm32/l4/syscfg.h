/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs cdefs_use_reg_meask=1 cdefs_use_field_set=1                  \
     cdefs_use_field_get=1 reg_prefix=STM32 field_prefix=STM32                 \
     cdefs_use_field_mask=1 cdefs_use_field_setval=1
*/

#ifndef _SYSCFG_BFGEN_DEFS_
#define _SYSCFG_BFGEN_DEFS_

/** memory remap register @multiple */
#define STM32_SYSCFG_MEMRMP_ADDR                     0x00000000
/** Memory mapping selection @multiple */
  #define STM32_SYSCFG_MEMRMP_MEM_MODE_MASK        0x00000007
  #define STM32_SYSCFG_MEMRMP_MEM_MODE(v)          ((v) << 0)
  #define STM32_SYSCFG_MEMRMP_MEM_MODE_SET(x, v)   do { (x) = (((x) & ~0x7) | ((v) << 0)); } while(0)
  #define STM32_SYSCFG_MEMRMP_MEM_MODE_GET(x)      (((x) >> 0) & 0x7)
/** QUADSPI memory mapping swap @multiple */
  #define STM32_SYSCFG_MEMRMP_QFS_MASK             0x00000001
  #define STM32_SYSCFG_MEMRMP_QFS                  0x00000008
  #define STM32_SYSCFG_MEMRMP_QFS_SET(x, v)        do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
  #define STM32_SYSCFG_MEMRMP_QFS_GET(x)           (((x) >> 3) & 0x1)
/** Flash Bank mode selection @multiple */
  #define STM32_SYSCFG_MEMRMP_FB_MODE_MASK         0x00000001
  #define STM32_SYSCFG_MEMRMP_FB_MODE              0x00000100
  #define STM32_SYSCFG_MEMRMP_FB_MODE_SET(x, v)    do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define STM32_SYSCFG_MEMRMP_FB_MODE_GET(x)       (((x) >> 8) & 0x1)

/** configuration register 1 @multiple */
#define STM32_SYSCFG_CFGR1_ADDR                      0x00000004
/** Firewall disable @multiple */
  #define STM32_SYSCFG_CFGR1_FWDIS_MASK            0x00000001
  #define STM32_SYSCFG_CFGR1_FWDIS                 0x00000001
  #define STM32_SYSCFG_CFGR1_FWDIS_SET(x, v)       do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define STM32_SYSCFG_CFGR1_FWDIS_GET(x)          (((x) >> 0) & 0x1)
/** I/O analog switch voltage booster enable @multiple */
  #define STM32_SYSCFG_CFGR1_BOOSTEN_MASK          0x00000001
  #define STM32_SYSCFG_CFGR1_BOOSTEN               0x00000100
  #define STM32_SYSCFG_CFGR1_BOOSTEN_SET(x, v)     do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define STM32_SYSCFG_CFGR1_BOOSTEN_GET(x)        (((x) >> 8) & 0x1)
/** Fast-mode Plus (Fm+) driving capability activation on PB6 @multiple */
  #define STM32_SYSCFG_CFGR1_I2C_PB6_FMP_MASK      0x00000001
  #define STM32_SYSCFG_CFGR1_I2C_PB6_FMP           0x00010000
  #define STM32_SYSCFG_CFGR1_I2C_PB6_FMP_SET(x, v) do { (x) = (((x) & ~0x10000) | ((v) << 16)); } while(0)
  #define STM32_SYSCFG_CFGR1_I2C_PB6_FMP_GET(x)    (((x) >> 16) & 0x1)
/** Fast-mode Plus (Fm+) driving capability activation on PB7 @multiple */
  #define STM32_SYSCFG_CFGR1_I2C_PB7_FMP_MASK      0x00000001
  #define STM32_SYSCFG_CFGR1_I2C_PB7_FMP           0x00020000
  #define STM32_SYSCFG_CFGR1_I2C_PB7_FMP_SET(x, v) do { (x) = (((x) & ~0x20000) | ((v) << 17)); } while(0)
  #define STM32_SYSCFG_CFGR1_I2C_PB7_FMP_GET(x)    (((x) >> 17) & 0x1)
/** Fast-mode Plus (Fm+) driving capability activation on PB8 @multiple */
  #define STM32_SYSCFG_CFGR1_I2C_PB8_FMP_MASK      0x00000001
  #define STM32_SYSCFG_CFGR1_I2C_PB8_FMP           0x00040000
  #define STM32_SYSCFG_CFGR1_I2C_PB8_FMP_SET(x, v) do { (x) = (((x) & ~0x40000) | ((v) << 18)); } while(0)
  #define STM32_SYSCFG_CFGR1_I2C_PB8_FMP_GET(x)    (((x) >> 18) & 0x1)
/** Fast-mode Plus (Fm+) driving capability activation on PB9 @multiple */
  #define STM32_SYSCFG_CFGR1_I2C_PB9_FMP_MASK      0x00000001
  #define STM32_SYSCFG_CFGR1_I2C_PB9_FMP           0x00080000
  #define STM32_SYSCFG_CFGR1_I2C_PB9_FMP_SET(x, v) do { (x) = (((x) & ~0x80000) | ((v) << 19)); } while(0)
  #define STM32_SYSCFG_CFGR1_I2C_PB9_FMP_GET(x)    (((x) >> 19) & 0x1)
/** I2C1 Fast-mode Plus driving capability activation @multiple */
  #define STM32_SYSCFG_CFGR1_I2C1_FMP_MASK         0x00000001
  #define STM32_SYSCFG_CFGR1_I2C1_FMP              0x00100000
  #define STM32_SYSCFG_CFGR1_I2C1_FMP_SET(x, v)    do { (x) = (((x) & ~0x100000) | ((v) << 20)); } while(0)
  #define STM32_SYSCFG_CFGR1_I2C1_FMP_GET(x)       (((x) >> 20) & 0x1)
/** I2C2 Fast-mode Plus driving capability activation @multiple */
  #define STM32_SYSCFG_CFGR1_I2C2_FMP_MASK         0x00000001
  #define STM32_SYSCFG_CFGR1_I2C2_FMP              0x00200000
  #define STM32_SYSCFG_CFGR1_I2C2_FMP_SET(x, v)    do { (x) = (((x) & ~0x200000) | ((v) << 21)); } while(0)
  #define STM32_SYSCFG_CFGR1_I2C2_FMP_GET(x)       (((x) >> 21) & 0x1)
/** I2C3 Fast-mode Plus driving capability activation @multiple */
  #define STM32_SYSCFG_CFGR1_I2C3_FMP_MASK         0x00000001
  #define STM32_SYSCFG_CFGR1_I2C3_FMP              0x00400000
  #define STM32_SYSCFG_CFGR1_I2C3_FMP_SET(x, v)    do { (x) = (((x) & ~0x400000) | ((v) << 22)); } while(0)
  #define STM32_SYSCFG_CFGR1_I2C3_FMP_GET(x)       (((x) >> 22) & 0x1)
/** Floating Point Unit interrupts enable bits @multiple */
  #define STM32_SYSCFG_CFGR1_FPU_IE_MASK           0x0000003f
  #define STM32_SYSCFG_CFGR1_FPU_IE(v)             ((v) << 26)
  #define STM32_SYSCFG_CFGR1_FPU_IE_SET(x, v)      do { (x) = (((x) & ~0xfc000000) | ((v) << 26)); } while(0)
  #define STM32_SYSCFG_CFGR1_FPU_IE_GET(x)         (((x) >> 26) & 0x3f)

/** external interrupt configuration register @multiple */
#define STM32_SYSCFG_EXTICR_ADDR(ridx)               (0x00000008 + (ridx) * 4)
#define STM32_SYSCFG_EXTICR_COUNT                    4
/** EXTI x configuration bits @multiple */
  #define STM32_SYSCFG_EXTICR_EXTI_MASK            0x00000007
  #define STM32_SYSCFG_EXTICR_EXTI_COUNT           4
  #define STM32_SYSCFG_EXTICR_EXTI(fidx, v)        ((v) << ((fidx) * 4 + 0))
  #define STM32_SYSCFG_EXTICR_EXTI_SET(fidx, x, v) do { (x) = (((x) & ~(0x7 << ((fidx) * 4))) | ((v) << ((fidx) * 4 + 0))); } while(0)
  #define STM32_SYSCFG_EXTICR_EXTI_GET(fidx, x)    (((x) >> ((fidx) * 4 + 0)) & 0x7)

/** SCSR @multiple */
#define STM32_SYSCFG_SCSR_ADDR                       0x00000018
/** SRAM2 Erase @multiple */
  #define STM32_SYSCFG_SCSR_SRAM2ER_MASK           0x00000001
  #define STM32_SYSCFG_SCSR_SRAM2ER                0x00000001
  #define STM32_SYSCFG_SCSR_SRAM2ER_SET(x, v)      do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define STM32_SYSCFG_SCSR_SRAM2ER_GET(x)         (((x) >> 0) & 0x1)
/** SRAM2 busy by erase operation @multiple */
  #define STM32_SYSCFG_SCSR_SRAM2BSY_MASK          0x00000001
  #define STM32_SYSCFG_SCSR_SRAM2BSY               0x00000002
  #define STM32_SYSCFG_SCSR_SRAM2BSY_SET(x, v)     do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define STM32_SYSCFG_SCSR_SRAM2BSY_GET(x)        (((x) >> 1) & 0x1)

/** CFGR2 @multiple */
#define STM32_SYSCFG_CFGR2_ADDR                      0x0000001c
/**
   CortexÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€šÃ‚Â ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¾Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¦ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¾ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬ÃƒÂ¢Ã¢â‚¬Å¾Ã‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã‚Â¦ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã¢â‚¬Â ÃƒÂ¢Ã¢â€šÂ¬Ã¢â€žÂ¢ÃƒÆ’Ã†â€™Ãƒâ€šÃ‚Â¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡Ãƒâ€šÃ‚Â¬ÃƒÆ’Ã¢â‚¬Â¦Ãƒâ€šÃ‚Â¡ÃƒÆ’Ã†â€™Ãƒâ€ Ã¢â‚¬â„¢ÃƒÆ’Ã‚Â¢ÃƒÂ¢Ã¢â‚¬Å¡Ã‚Â¬Ãƒâ€¦Ã‚Â¡ÃƒÆ’Ã†â€™ÃƒÂ¢Ã¢â€šÂ¬Ã…Â¡ÃƒÆ’Ã¢â‚¬Å¡Ãƒâ€šÃ‚Â¢-M4
   LOCKUP (Hardfault) output enable bit @multiple */
  #define STM32_SYSCFG_CFGR2_CLL_MASK              0x00000001
  #define STM32_SYSCFG_CFGR2_CLL                   0x00000001
  #define STM32_SYSCFG_CFGR2_CLL_SET(x, v)         do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define STM32_SYSCFG_CFGR2_CLL_GET(x)            (((x) >> 0) & 0x1)
/** SRAM2 parity lock bit @multiple */
  #define STM32_SYSCFG_CFGR2_SPL_MASK              0x00000001
  #define STM32_SYSCFG_CFGR2_SPL                   0x00000002
  #define STM32_SYSCFG_CFGR2_SPL_SET(x, v)         do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define STM32_SYSCFG_CFGR2_SPL_GET(x)            (((x) >> 1) & 0x1)
/** PVD lock enable bit @multiple */
  #define STM32_SYSCFG_CFGR2_PVDL_MASK             0x00000001
  #define STM32_SYSCFG_CFGR2_PVDL                  0x00000004
  #define STM32_SYSCFG_CFGR2_PVDL_SET(x, v)        do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
  #define STM32_SYSCFG_CFGR2_PVDL_GET(x)           (((x) >> 2) & 0x1)
/** ECC Lock @multiple */
  #define STM32_SYSCFG_CFGR2_ECCL_MASK             0x00000001
  #define STM32_SYSCFG_CFGR2_ECCL                  0x00000008
  #define STM32_SYSCFG_CFGR2_ECCL_SET(x, v)        do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
  #define STM32_SYSCFG_CFGR2_ECCL_GET(x)           (((x) >> 3) & 0x1)
/** SRAM2 parity error flag @multiple */
  #define STM32_SYSCFG_CFGR2_SPF_MASK              0x00000001
  #define STM32_SYSCFG_CFGR2_SPF                   0x00000100
  #define STM32_SYSCFG_CFGR2_SPF_SET(x, v)         do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define STM32_SYSCFG_CFGR2_SPF_GET(x)            (((x) >> 8) & 0x1)

/** SWPR @multiple */
#define STM32_SYSCFG_SWPR_ADDR                       0x00000020
/** P0WP @multiple */
  #define STM32_SYSCFG_SWPR_P0WP_MASK              0x00000001
  #define STM32_SYSCFG_SWPR_P0WP                   0x00000001
  #define STM32_SYSCFG_SWPR_P0WP_SET(x, v)         do { (x) = (((x) & ~0x1) | ((v) << 0)); } while(0)
  #define STM32_SYSCFG_SWPR_P0WP_GET(x)            (((x) >> 0) & 0x1)
/** P1WP @multiple */
  #define STM32_SYSCFG_SWPR_P1WP_MASK              0x00000001
  #define STM32_SYSCFG_SWPR_P1WP                   0x00000002
  #define STM32_SYSCFG_SWPR_P1WP_SET(x, v)         do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define STM32_SYSCFG_SWPR_P1WP_GET(x)            (((x) >> 1) & 0x1)
/** P2WP @multiple */
  #define STM32_SYSCFG_SWPR_P2WP_MASK              0x00000001
  #define STM32_SYSCFG_SWPR_P2WP                   0x00000004
  #define STM32_SYSCFG_SWPR_P2WP_SET(x, v)         do { (x) = (((x) & ~0x4) | ((v) << 2)); } while(0)
  #define STM32_SYSCFG_SWPR_P2WP_GET(x)            (((x) >> 2) & 0x1)
/** P3WP @multiple */
  #define STM32_SYSCFG_SWPR_P3WP_MASK              0x00000001
  #define STM32_SYSCFG_SWPR_P3WP                   0x00000008
  #define STM32_SYSCFG_SWPR_P3WP_SET(x, v)         do { (x) = (((x) & ~0x8) | ((v) << 3)); } while(0)
  #define STM32_SYSCFG_SWPR_P3WP_GET(x)            (((x) >> 3) & 0x1)
/** P4WP @multiple */
  #define STM32_SYSCFG_SWPR_P4WP_MASK              0x00000001
  #define STM32_SYSCFG_SWPR_P4WP                   0x00000010
  #define STM32_SYSCFG_SWPR_P4WP_SET(x, v)         do { (x) = (((x) & ~0x10) | ((v) << 4)); } while(0)
  #define STM32_SYSCFG_SWPR_P4WP_GET(x)            (((x) >> 4) & 0x1)
/** P5WP @multiple */
  #define STM32_SYSCFG_SWPR_P5WP_MASK              0x00000001
  #define STM32_SYSCFG_SWPR_P5WP                   0x00000020
  #define STM32_SYSCFG_SWPR_P5WP_SET(x, v)         do { (x) = (((x) & ~0x20) | ((v) << 5)); } while(0)
  #define STM32_SYSCFG_SWPR_P5WP_GET(x)            (((x) >> 5) & 0x1)
/** P6WP @multiple */
  #define STM32_SYSCFG_SWPR_P6WP_MASK              0x00000001
  #define STM32_SYSCFG_SWPR_P6WP                   0x00000040
  #define STM32_SYSCFG_SWPR_P6WP_SET(x, v)         do { (x) = (((x) & ~0x40) | ((v) << 6)); } while(0)
  #define STM32_SYSCFG_SWPR_P6WP_GET(x)            (((x) >> 6) & 0x1)
/** P7WP @multiple */
  #define STM32_SYSCFG_SWPR_P7WP_MASK              0x00000001
  #define STM32_SYSCFG_SWPR_P7WP                   0x00000080
  #define STM32_SYSCFG_SWPR_P7WP_SET(x, v)         do { (x) = (((x) & ~0x80) | ((v) << 7)); } while(0)
  #define STM32_SYSCFG_SWPR_P7WP_GET(x)            (((x) >> 7) & 0x1)
/** P8WP @multiple */
  #define STM32_SYSCFG_SWPR_P8WP_MASK              0x00000001
  #define STM32_SYSCFG_SWPR_P8WP                   0x00000100
  #define STM32_SYSCFG_SWPR_P8WP_SET(x, v)         do { (x) = (((x) & ~0x100) | ((v) << 8)); } while(0)
  #define STM32_SYSCFG_SWPR_P8WP_GET(x)            (((x) >> 8) & 0x1)
/** P9WP @multiple */
  #define STM32_SYSCFG_SWPR_P9WP_MASK              0x00000001
  #define STM32_SYSCFG_SWPR_P9WP                   0x00000200
  #define STM32_SYSCFG_SWPR_P9WP_SET(x, v)         do { (x) = (((x) & ~0x200) | ((v) << 9)); } while(0)
  #define STM32_SYSCFG_SWPR_P9WP_GET(x)            (((x) >> 9) & 0x1)
/** P10WP @multiple */
  #define STM32_SYSCFG_SWPR_P10WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P10WP                  0x00000400
  #define STM32_SYSCFG_SWPR_P10WP_SET(x, v)        do { (x) = (((x) & ~0x400) | ((v) << 10)); } while(0)
  #define STM32_SYSCFG_SWPR_P10WP_GET(x)           (((x) >> 10) & 0x1)
/** P11WP @multiple */
  #define STM32_SYSCFG_SWPR_P11WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P11WP                  0x00000800
  #define STM32_SYSCFG_SWPR_P11WP_SET(x, v)        do { (x) = (((x) & ~0x800) | ((v) << 11)); } while(0)
  #define STM32_SYSCFG_SWPR_P11WP_GET(x)           (((x) >> 11) & 0x1)
/** P12WP @multiple */
  #define STM32_SYSCFG_SWPR_P12WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P12WP                  0x00001000
  #define STM32_SYSCFG_SWPR_P12WP_SET(x, v)        do { (x) = (((x) & ~0x1000) | ((v) << 12)); } while(0)
  #define STM32_SYSCFG_SWPR_P12WP_GET(x)           (((x) >> 12) & 0x1)
/** P13WP @multiple */
  #define STM32_SYSCFG_SWPR_P13WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P13WP                  0x00002000
  #define STM32_SYSCFG_SWPR_P13WP_SET(x, v)        do { (x) = (((x) & ~0x2000) | ((v) << 13)); } while(0)
  #define STM32_SYSCFG_SWPR_P13WP_GET(x)           (((x) >> 13) & 0x1)
/** P14WP @multiple */
  #define STM32_SYSCFG_SWPR_P14WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P14WP                  0x00004000
  #define STM32_SYSCFG_SWPR_P14WP_SET(x, v)        do { (x) = (((x) & ~0x4000) | ((v) << 14)); } while(0)
  #define STM32_SYSCFG_SWPR_P14WP_GET(x)           (((x) >> 14) & 0x1)
/** P15WP @multiple */
  #define STM32_SYSCFG_SWPR_P15WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P15WP                  0x00008000
  #define STM32_SYSCFG_SWPR_P15WP_SET(x, v)        do { (x) = (((x) & ~0x8000) | ((v) << 15)); } while(0)
  #define STM32_SYSCFG_SWPR_P15WP_GET(x)           (((x) >> 15) & 0x1)
/** P16WP @multiple */
  #define STM32_SYSCFG_SWPR_P16WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P16WP                  0x00010000
  #define STM32_SYSCFG_SWPR_P16WP_SET(x, v)        do { (x) = (((x) & ~0x10000) | ((v) << 16)); } while(0)
  #define STM32_SYSCFG_SWPR_P16WP_GET(x)           (((x) >> 16) & 0x1)
/** P17WP @multiple */
  #define STM32_SYSCFG_SWPR_P17WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P17WP                  0x00020000
  #define STM32_SYSCFG_SWPR_P17WP_SET(x, v)        do { (x) = (((x) & ~0x20000) | ((v) << 17)); } while(0)
  #define STM32_SYSCFG_SWPR_P17WP_GET(x)           (((x) >> 17) & 0x1)
/** P18WP @multiple */
  #define STM32_SYSCFG_SWPR_P18WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P18WP                  0x00040000
  #define STM32_SYSCFG_SWPR_P18WP_SET(x, v)        do { (x) = (((x) & ~0x40000) | ((v) << 18)); } while(0)
  #define STM32_SYSCFG_SWPR_P18WP_GET(x)           (((x) >> 18) & 0x1)
/** P19WP @multiple */
  #define STM32_SYSCFG_SWPR_P19WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P19WP                  0x00080000
  #define STM32_SYSCFG_SWPR_P19WP_SET(x, v)        do { (x) = (((x) & ~0x80000) | ((v) << 19)); } while(0)
  #define STM32_SYSCFG_SWPR_P19WP_GET(x)           (((x) >> 19) & 0x1)
/** P20WP @multiple */
  #define STM32_SYSCFG_SWPR_P20WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P20WP                  0x00100000
  #define STM32_SYSCFG_SWPR_P20WP_SET(x, v)        do { (x) = (((x) & ~0x100000) | ((v) << 20)); } while(0)
  #define STM32_SYSCFG_SWPR_P20WP_GET(x)           (((x) >> 20) & 0x1)
/** P21WP @multiple */
  #define STM32_SYSCFG_SWPR_P21WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P21WP                  0x00200000
  #define STM32_SYSCFG_SWPR_P21WP_SET(x, v)        do { (x) = (((x) & ~0x200000) | ((v) << 21)); } while(0)
  #define STM32_SYSCFG_SWPR_P21WP_GET(x)           (((x) >> 21) & 0x1)
/** P22WP @multiple */
  #define STM32_SYSCFG_SWPR_P22WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P22WP                  0x00400000
  #define STM32_SYSCFG_SWPR_P22WP_SET(x, v)        do { (x) = (((x) & ~0x400000) | ((v) << 22)); } while(0)
  #define STM32_SYSCFG_SWPR_P22WP_GET(x)           (((x) >> 22) & 0x1)
/** P23WP @multiple */
  #define STM32_SYSCFG_SWPR_P23WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P23WP                  0x00800000
  #define STM32_SYSCFG_SWPR_P23WP_SET(x, v)        do { (x) = (((x) & ~0x800000) | ((v) << 23)); } while(0)
  #define STM32_SYSCFG_SWPR_P23WP_GET(x)           (((x) >> 23) & 0x1)
/** P24WP @multiple */
  #define STM32_SYSCFG_SWPR_P24WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P24WP                  0x01000000
  #define STM32_SYSCFG_SWPR_P24WP_SET(x, v)        do { (x) = (((x) & ~0x1000000) | ((v) << 24)); } while(0)
  #define STM32_SYSCFG_SWPR_P24WP_GET(x)           (((x) >> 24) & 0x1)
/** P25WP @multiple */
  #define STM32_SYSCFG_SWPR_P25WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P25WP                  0x02000000
  #define STM32_SYSCFG_SWPR_P25WP_SET(x, v)        do { (x) = (((x) & ~0x2000000) | ((v) << 25)); } while(0)
  #define STM32_SYSCFG_SWPR_P25WP_GET(x)           (((x) >> 25) & 0x1)
/** P26WP @multiple */
  #define STM32_SYSCFG_SWPR_P26WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P26WP                  0x04000000
  #define STM32_SYSCFG_SWPR_P26WP_SET(x, v)        do { (x) = (((x) & ~0x4000000) | ((v) << 26)); } while(0)
  #define STM32_SYSCFG_SWPR_P26WP_GET(x)           (((x) >> 26) & 0x1)
/** P27WP @multiple */
  #define STM32_SYSCFG_SWPR_P27WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P27WP                  0x08000000
  #define STM32_SYSCFG_SWPR_P27WP_SET(x, v)        do { (x) = (((x) & ~0x8000000) | ((v) << 27)); } while(0)
  #define STM32_SYSCFG_SWPR_P27WP_GET(x)           (((x) >> 27) & 0x1)
/** P28WP @multiple */
  #define STM32_SYSCFG_SWPR_P28WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P28WP                  0x10000000
  #define STM32_SYSCFG_SWPR_P28WP_SET(x, v)        do { (x) = (((x) & ~0x10000000) | ((v) << 28)); } while(0)
  #define STM32_SYSCFG_SWPR_P28WP_GET(x)           (((x) >> 28) & 0x1)
/** P29WP @multiple */
  #define STM32_SYSCFG_SWPR_P29WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P29WP                  0x20000000
  #define STM32_SYSCFG_SWPR_P29WP_SET(x, v)        do { (x) = (((x) & ~0x20000000) | ((v) << 29)); } while(0)
  #define STM32_SYSCFG_SWPR_P29WP_GET(x)           (((x) >> 29) & 0x1)
/** P30WP @multiple */
  #define STM32_SYSCFG_SWPR_P30WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P30WP                  0x40000000
  #define STM32_SYSCFG_SWPR_P30WP_SET(x, v)        do { (x) = (((x) & ~0x40000000) | ((v) << 30)); } while(0)
  #define STM32_SYSCFG_SWPR_P30WP_GET(x)           (((x) >> 30) & 0x1)
/** SRAM2 page 31 write protection @multiple */
  #define STM32_SYSCFG_SWPR_P31WP_MASK             0x00000001
  #define STM32_SYSCFG_SWPR_P31WP                  0x80000000
  #define STM32_SYSCFG_SWPR_P31WP_SET(x, v)        do { (x) = (((x) & ~0x80000000) | ((v) << 31)); } while(0)
  #define STM32_SYSCFG_SWPR_P31WP_GET(x)           (((x) >> 31) & 0x1)

/** SKR @multiple */
#define STM32_SYSCFG_SKR_ADDR                        0x00000024
/** SRAM2 write protection key for software erase @multiple */
  #define STM32_SYSCFG_SKR_KEY_MASK                0x000000ff
  #define STM32_SYSCFG_SKR_KEY(v)                  ((v) << 0)
  #define STM32_SYSCFG_SKR_KEY_SET(x, v)           do { (x) = (((x) & ~0xff) | ((v) << 0)); } while(0)
  #define STM32_SYSCFG_SKR_KEY_GET(x)              (((x) >> 0) & 0xff)

#endif

