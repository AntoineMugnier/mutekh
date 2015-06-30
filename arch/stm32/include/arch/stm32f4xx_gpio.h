/***************************************
* Auto generated by BFGen, do not edit *
***************************************/

/*
   bfgen -o cdefs cdefs_use_reg_mask=1 cdefs_use_field_set=1                   \
     cdefs_use_field_get=1 reg_prefix=STM32 field_prefix=STM32                 \
     cdefs_use_field_mask=1
*/

#ifndef _GPIO_BFGEN_DEFS_
#define _GPIO_BFGEN_DEFS_

#define STM32_GPIO_MODER_ADDR(ridx)                  (0x00000000 + (ridx) * 1024)
#define STM32_GPIO_MODER_COUNT                       7
#define STM32_GPIO_MODER_MASK                        0xffffffff
  #define STM32_GPIO_MODER_MODE_MASK               0x00000003
  #define STM32_GPIO_MODER_MODE_COUNT              16
  #define STM32_GPIO_MODER_MODE(fidx, v)           ((STM32_GPIO_MODER_MODE_##v) << ((fidx) * 2 + 0))
  #define STM32_GPIO_MODER_MODE_SET(fidx, x, v)    do { (x) = (((x) & ~(0x3 << ((fidx) * 2))) | ((STM32_GPIO_MODER_MODE_##v) << ((fidx) * 2 + 0))); } while(0)
  #define STM32_GPIO_MODER_MODE_GET(fidx, x)       (((x) >> ((fidx) * 2 + 0)) & 0x3)
  #define STM32_GPIO_MODER_MODE_INPUT              0x00000000
  #define STM32_GPIO_MODER_MODE_OUTPUT             0x00000001
  #define STM32_GPIO_MODER_MODE_ALT                0x00000002
  #define STM32_GPIO_MODER_MODE_ANALOG             0x00000003

#define STM32_GPIO_OTYPER_ADDR(ridx)                 (0x00000004 + (ridx) * 1024)
#define STM32_GPIO_OTYPER_COUNT                      7
#define STM32_GPIO_OTYPER_MASK                       0x0000ffff
  #define STM32_GPIO_OTYPER_OT_MASK                0x00000001
  #define STM32_GPIO_OTYPER_OT_COUNT               16
  #define STM32_GPIO_OTYPER_OT(fidx, v)            ((STM32_GPIO_OTYPER_OT_##v) << ((fidx) + 0))
  #define STM32_GPIO_OTYPER_OT_SET(fidx, x, v)     do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((STM32_GPIO_OTYPER_OT_##v) << ((fidx) + 0))); } while(0)
  #define STM32_GPIO_OTYPER_OT_GET(fidx, x)        (((x) >> ((fidx) + 0)) & 0x1)
  #define STM32_GPIO_OTYPER_OT_PUSHPULL            0x00000000
  #define STM32_GPIO_OTYPER_OT_OPENDRAIN           0x00000001

#define STM32_GPIO_OSPEEDR_ADDR(ridx)                (0x00000008 + (ridx) * 1024)
#define STM32_GPIO_OSPEEDR_COUNT                     7
#define STM32_GPIO_OSPEEDR_MASK                      0xffffffff
  #define STM32_GPIO_OSPEEDR_OSPEED_MASK           0x00000003
  #define STM32_GPIO_OSPEEDR_OSPEED_COUNT          16
  #define STM32_GPIO_OSPEEDR_OSPEED(fidx, v)       ((STM32_GPIO_OSPEEDR_OSPEED_##v) << ((fidx) * 2 + 0))
  #define STM32_GPIO_OSPEEDR_OSPEED_SET(fidx, x, v) do { (x) = (((x) & ~(0x3 << ((fidx) * 2))) | ((STM32_GPIO_OSPEEDR_OSPEED_##v) << ((fidx) * 2 + 0))); } while(0)
  #define STM32_GPIO_OSPEEDR_OSPEED_GET(fidx, x)   (((x) >> ((fidx) * 2 + 0)) & 0x3)
  #define STM32_GPIO_OSPEEDR_OSPEED_LOW            0x00000000
  #define STM32_GPIO_OSPEEDR_OSPEED_MEDIUM         0x00000001
  #define STM32_GPIO_OSPEEDR_OSPEED_FAST           0x00000002
  #define STM32_GPIO_OSPEEDR_OSPEED_HIGH           0x00000003

#define STM32_GPIO_PUPDR_ADDR(ridx)                  (0x0000000c + (ridx) * 1024)
#define STM32_GPIO_PUPDR_COUNT                       7
#define STM32_GPIO_PUPDR_MASK                        0xffffffff
  #define STM32_GPIO_PUPDR_PUPD_MASK               0x00000003
  #define STM32_GPIO_PUPDR_PUPD_COUNT              16
  #define STM32_GPIO_PUPDR_PUPD(fidx, v)           ((STM32_GPIO_PUPDR_PUPD_##v) << ((fidx) * 2 + 0))
  #define STM32_GPIO_PUPDR_PUPD_SET(fidx, x, v)    do { (x) = (((x) & ~(0x3 << ((fidx) * 2))) | ((STM32_GPIO_PUPDR_PUPD_##v) << ((fidx) * 2 + 0))); } while(0)
  #define STM32_GPIO_PUPDR_PUPD_GET(fidx, x)       (((x) >> ((fidx) * 2 + 0)) & 0x3)
  #define STM32_GPIO_PUPDR_PUPD_NONE               0x00000000
  #define STM32_GPIO_PUPDR_PUPD_PULLUP             0x00000001
  #define STM32_GPIO_PUPDR_PUPD_PULLDOWN           0x00000002

#define STM32_GPIO_IDR_ADDR(ridx)                    (0x00000010 + (ridx) * 1024)
#define STM32_GPIO_IDR_COUNT                         7
#define STM32_GPIO_IDR_MASK                          0x0000ffff
  #define STM32_GPIO_IDR_ID_MASK                   0x00000001
  #define STM32_GPIO_IDR_ID_COUNT                  16
  #define STM32_GPIO_IDR_ID(fidx)                  (0x00000001 << ((fidx)))
  #define STM32_GPIO_IDR_ID_SET(fidx, x, v)        do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)
  #define STM32_GPIO_IDR_ID_GET(fidx, x)           (((x) >> ((fidx) + 0)) & 0x1)

#define STM32_GPIO_ODR_ADDR(ridx)                    (0x00000014 + (ridx) * 1024)
#define STM32_GPIO_ODR_COUNT                         7
#define STM32_GPIO_ODR_MASK                          0x0000ffff
  #define STM32_GPIO_ODR_OD_MASK                   0x00000001
  #define STM32_GPIO_ODR_OD_COUNT                  16
  #define STM32_GPIO_ODR_OD(fidx)                  (0x00000001 << ((fidx)))
  #define STM32_GPIO_ODR_OD_SET(fidx, x, v)        do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)
  #define STM32_GPIO_ODR_OD_GET(fidx, x)           (((x) >> ((fidx) + 0)) & 0x1)

#define STM32_GPIO_BSRR_ADDR(ridx)                   (0x00000018 + (ridx) * 1024)
#define STM32_GPIO_BSRR_COUNT                        7
#define STM32_GPIO_BSRR_MASK                         0x0001ffff
  #define STM32_GPIO_BSRR_BS_MASK                  0x00000001
  #define STM32_GPIO_BSRR_BS_COUNT                 16
  #define STM32_GPIO_BSRR_BS(fidx)                 (0x00000001 << ((fidx)))
  #define STM32_GPIO_BSRR_BS_SET(fidx, x, v)       do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)
  #define STM32_GPIO_BSRR_BS_GET(fidx, x)          (((x) >> ((fidx) + 0)) & 0x1)
  #define STM32_GPIO_BSRR_BR_MASK                  0x00000001
  #define STM32_GPIO_BSRR_BR_COUNT                 16
  #define STM32_GPIO_BSRR_BR(fidx)                 (0x00000002 << ((fidx)))
  #define STM32_GPIO_BSRR_BR_SET(fidx, x, v)       do { (x) = (((x) & ~(0x2 << ((fidx)))) | ((v) << ((fidx) + 1))); } while(0)
  #define STM32_GPIO_BSRR_BR_GET(fidx, x)          (((x) >> ((fidx) + 1)) & 0x1)

#define STM32_GPIO_LCKR_ADDR(ridx)                   (0x0000001c + (ridx) * 1024)
#define STM32_GPIO_LCKR_COUNT                        7
#define STM32_GPIO_LCKR_MASK                         0x0000ffff
  #define STM32_GPIO_LCKR_LCK_MASK                 0x00000001
  #define STM32_GPIO_LCKR_LCK_COUNT                16
  #define STM32_GPIO_LCKR_LCK(fidx)                (0x00000001 << ((fidx)))
  #define STM32_GPIO_LCKR_LCK_SET(fidx, x, v)      do { (x) = (((x) & ~(0x1 << ((fidx)))) | ((v) << ((fidx) + 0))); } while(0)
  #define STM32_GPIO_LCKR_LCK_GET(fidx, x)         (((x) >> ((fidx) + 0)) & 0x1)
  #define STM32_GPIO_LCKR_LCKK_MASK                0x00000001
  #define STM32_GPIO_LCKR_LCKK                     0x00000002
  #define STM32_GPIO_LCKR_LCKK_SET(x, v)           do { (x) = (((x) & ~0x2) | ((v) << 1)); } while(0)
  #define STM32_GPIO_LCKR_LCKK_GET(x)              (((x) >> 1) & 0x1)

#define STM32_GPIO_AFRL_ADDR(ridx)                   (0x00000020 + (ridx) * 1024)
#define STM32_GPIO_AFRL_COUNT                        7
#define STM32_GPIO_AFRL_MASK                         0xffffffff
  #define STM32_GPIO_AFRL_AF_MASK                  0x0000000f
  #define STM32_GPIO_AFRL_AF_COUNT                 8
  #define STM32_GPIO_AFRL_AF(fidx, v)              ((v) << ((fidx) * 4 + 0))
  #define STM32_GPIO_AFRL_AF_SET(fidx, x, v)       do { (x) = (((x) & ~(0xf << ((fidx) * 4))) | ((v) << ((fidx) * 4 + 0))); } while(0)
  #define STM32_GPIO_AFRL_AF_GET(fidx, x)          (((x) >> ((fidx) * 4 + 0)) & 0xf)

#define STM32_GPIO_AFRH_ADDR(ridx)                   (0x00000024 + (ridx) * 1024)
#define STM32_GPIO_AFRH_COUNT                        7
#define STM32_GPIO_AFRH_MASK                         0xffffffff
  #define STM32_GPIO_AFRH_AF_MASK                  0x0000000f
  #define STM32_GPIO_AFRH_AF_COUNT                 8
  #define STM32_GPIO_AFRH_AF(fidx, v)              ((v) << ((fidx) * 4 + 0))
  #define STM32_GPIO_AFRH_AF_SET(fidx, x, v)       do { (x) = (((x) & ~(0xf << ((fidx) * 4))) | ((v) << ((fidx) * 4 + 0))); } while(0)
  #define STM32_GPIO_AFRH_AF_GET(fidx, x)          (((x) >> ((fidx) * 4 + 0)) & 0xf)

#endif

