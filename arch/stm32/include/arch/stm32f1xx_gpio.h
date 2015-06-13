/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs cdefs_use_reg_mask=1 cdefs_use_field_mask=1 reg_prefix=STM32 \
     field_prefix=STM32 cdefs_use_field_set=1 cdefs_use_field_set=1
*/

#ifndef _GPIO_BFGEN_DEFS_
#define _GPIO_BFGEN_DEFS_

#define STM32_GPIO_CRL_ADDR                          0x00000000
#define STM32_GPIO_CRL_MASK                          0x0003ffff
  #define STM32_GPIO_CRL_MODE_MASK                 0x00000003
  #define STM32_GPIO_CRL_MODE_COUNT                8
  #define STM32_GPIO_CRL_MODE(fidx, v)             ((STM32_GPIO_CRL_MODE_##v) << ((fidx) * 2 + 0))
  #define STM32_GPIO_CRL_MODE_SET(fidx, x, v)      do { (x) = (((x) & ~(0x3 << ((fidx) * 2))) | ((STM32_GPIO_CRL_MODE_##v) << ((fidx) * 2 + 0))); } while(0)
  #define STM32_GPIO_CRL_MODE_GET(fidx, x)         (((x) >> ((fidx) * 2 + 0)) & 0x3)
  #define STM32_GPIO_CRL_MODE_INPUT                0x00000000
  #define STM32_GPIO_CRL_MODE_OUTPUT_10MHZ         0x00000001
  #define STM32_GPIO_CRL_MODE_OUTPUT_2MHZ          0x00000002
  #define STM32_GPIO_CRL_MODE_OUTPUT_50MHZ         0x00000003
  #define STM32_GPIO_CRL_CNF_MASK                  0x00000003
  #define STM32_GPIO_CRL_CNF_COUNT                 8
  #define STM32_GPIO_CRL_CNF(fidx, v)              ((STM32_GPIO_CRL_CNF_##v) << ((fidx) * 2 + 2))
  #define STM32_GPIO_CRL_CNF_SET(fidx, x, v)       do { (x) = (((x) & ~(0xc << ((fidx) * 2))) | ((STM32_GPIO_CRL_CNF_##v) << ((fidx) * 2 + 2))); } while(0)
  #define STM32_GPIO_CRL_CNF_GET(fidx, x)          (((x) >> ((fidx) * 2 + 2)) & 0x3)
  #define STM32_GPIO_CRL_CNF_ANALOG                0x00000000
  #define STM32_GPIO_CRL_CNF_GENERAL_PUSH_PULL     0x00000000
  #define STM32_GPIO_CRL_CNF_FLOATING              0x00000001
  #define STM32_GPIO_CRL_CNF_GENERAL_OPEN_DRAIN    0x00000001
  #define STM32_GPIO_CRL_CNF_PULL_UP_PULL_DOWN     0x00000002
  #define STM32_GPIO_CRL_CNF_ALT_PUSH_PULL         0x00000002
  #define STM32_GPIO_CRL_CNF_ATL_OPEN_DRAIN        0x00000003

#define STM32_GPIO_CRH_ADDR                          0x00000004
#define STM32_GPIO_CRH_MASK                          0x0003ffff
  #define STM32_GPIO_CRH_MODE_MASK                 0x00000003
  #define STM32_GPIO_CRH_MODE_COUNT                8
  #define STM32_GPIO_CRH_MODE(fidx, v)             ((STM32_GPIO_CRH_MODE_##v) << ((fidx) * 2 + 0))
  #define STM32_GPIO_CRH_MODE_SET(fidx, x, v)      do { (x) = (((x) & ~(0x3 << ((fidx) * 2))) | ((STM32_GPIO_CRH_MODE_##v) << ((fidx) * 2 + 0))); } while(0)
  #define STM32_GPIO_CRH_MODE_GET(fidx, x)         (((x) >> ((fidx) * 2 + 0)) & 0x3)
  #define STM32_GPIO_CRH_MODE_INPUT                0x00000000
  #define STM32_GPIO_CRH_MODE_OUTPUT_10_MHZ        0x00000001
  #define STM32_GPIO_CRH_MODE_OUTPUT_2_MHZ         0x00000002
  #define STM32_GPIO_CRH_MODE_OUTPUT_50_MHZ        0x00000003
  #define STM32_GPIO_CRH_CNF_MASK                  0x00000003
  #define STM32_GPIO_CRH_CNF_COUNT                 8
  #define STM32_GPIO_CRH_CNF(fidx, v)              ((STM32_GPIO_CRH_CNF_##v) << ((fidx) * 2 + 2))
  #define STM32_GPIO_CRH_CNF_SET(fidx, x, v)       do { (x) = (((x) & ~(0xc << ((fidx) * 2))) | ((STM32_GPIO_CRH_CNF_##v) << ((fidx) * 2 + 2))); } while(0)
  #define STM32_GPIO_CRH_CNF_GET(fidx, x)          (((x) >> ((fidx) * 2 + 2)) & 0x3)
  #define STM32_GPIO_CRH_CNF_ANALOG                0x00000000
  #define STM32_GPIO_CRH_CNF_GENERAL_PUSH_PULL     0x00000000
  #define STM32_GPIO_CRH_CNF_FLOATING              0x00000001
  #define STM32_GPIO_CRH_CNF_GENERAL_OPEN_DRAIN    0x00000001
  #define STM32_GPIO_CRH_CNF_PULLUP_PULLDOWN       0x00000002
  #define STM32_GPIO_CRH_CNF_ALT_PUSH_PULL         0x00000002
  #define STM32_GPIO_CRH_CNF_ATL_OPEN_DRAIN        0x00000003

#define STM32_GPIO_IDR_ADDR                          0x00000008
#define STM32_GPIO_IDR_MASK                          0x0000ffff
  #define STM32_GPIO_IDR_IRD_MASK                  0x00000001
  #define STM32_GPIO_IDR_IRD_COUNT                 16
  #define STM32_GPIO_IDR_IRD(fidx)                 (0x00000001 << ((fidx)))
  #define STM32_GPIO_IDR_IRD_SET(fidx, x, v)       do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)

#define STM32_GPIO_ODR_ADDR                          0x0000000c
#define STM32_GPIO_ODR_MASK                          0x0000ffff
  #define STM32_GPIO_ODR_ORD_MASK                  0x00000001
  #define STM32_GPIO_ODR_ORD_COUNT                 16
  #define STM32_GPIO_ODR_ORD(fidx)                 (0x00000001 << ((fidx)))
  #define STM32_GPIO_ODR_ORD_SET(fidx, x, v)       do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)

#define STM32_GPIO_BSRR_ADDR                         0x00000010
#define STM32_GPIO_BSRR_MASK                         0x0001ffff
  #define STM32_GPIO_BSRR_BS_MASK                  0x00000001
  #define STM32_GPIO_BSRR_BS_COUNT                 16
  #define STM32_GPIO_BSRR_BS(fidx)                 (0x00000001 << ((fidx)))
  #define STM32_GPIO_BSRR_BS_SET(fidx, x, v)       do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)
  #define STM32_GPIO_BSRR_BR_MASK                  0x00000001
  #define STM32_GPIO_BSRR_BR_COUNT                 16
  #define STM32_GPIO_BSRR_BR(fidx)                 (0x00000002 << ((fidx)))
  #define STM32_GPIO_BSRR_BR_SET(fidx, x, v)       do { (x) = (((x) & ~(0x2 << ((fidx)))) | ((v) << ((fidx) + 1))); } while(0)

#define STM32_GPIO_BRR_ADDR                          0x00000014
#define STM32_GPIO_BRR_MASK                          0x0000ffff
  #define STM32_GPIO_BRR_BR_MASK                   0x0000ffff
  #define STM32_GPIO_BRR_BR(v)                     ((v) << 0)
  #define STM32_GPIO_BRR_BR_SET(x, v)              do { (x) = (((x) & ~0xffff) | ((v) << 0)); } while(0)
  #define STM32_GPIO_BRR_BR_GET(x)                 (((x) >> 0) & 0xffff)

#define STM32_GPIO_LCKR_ADDR                         0x00000018
#define STM32_GPIO_LCKR_MASK                         0x0000ffff
  #define STM32_GPIO_LCKR_LCK_MASK                 0x00000001
  #define STM32_GPIO_LCKR_LCK_COUNT                16
  #define STM32_GPIO_LCKR_LCK(fidx)                (0x00000001 << ((fidx)))
  #define STM32_GPIO_LCKR_LCK_SET(fidx, x, v)      do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)
  #define STM32_GPIO_LCKR_LCKK_MASK                0x00000001
  #define STM32_GPIO_LCKR_LCKK                     0x00000002
  #define STM32_GPIO_LCKR_LCKK_SET(x, v)           do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)

#endif

