
#include <arch/efm32/cmu.h>
#include <arch/efm32/devaddr.h>



#define EFM32_DAC0_ADDR                         0x40004000

#define EFM32_DAC_CTRL                          0x000
#define   EFM32_DAC_CTRL_REFRSEL_MASK           0x00300000
#define     EFM32_DAC_CTRL_REFRSEL(v)           ((EFM32_DAC_CTRL_REFRSEL_##v) << 20)
#define       EFM32_DAC_CTRL_REFRSEL_8CYCLES    0
#define       EFM32_DAC_CTRL_REFRSEL_16CYCLES   1
#define       EFM32_DAC_CTRL_REFRSEL_32CYCLES   2
#define       EFM32_DAC_CTRL_REFRSEL_64CYCLES   3
#define   EFM32_DAC_CTRL_PRESC_MASK             0x00070000
#define   EFM32_DAC_CTRL_PRESC(v)               (((v) & 0x7) << 16)
#define   EFM32_DAC_CTRL_REFSEL_MASK            0x00000300
#define     EFM32_DAC_CTRL_REFSEL(v)           ((EFM32_DAC_CTRL_REFSEL_##v) << 8)
#define       EFM32_DAC_CTRL_REFSEL_1V25        0
#define       EFM32_DAC_CTRL_REFSEL_2V5         1
#define       EFM32_DAC_CTRL_REFSEL_VDD         2
#define   EFM32_DAC_CTRL_CH0PRESCRST_MASK       0x00000080
#define   EFM32_DAC_CTRL_OUTENPRS_MASK          0x00000040
#define   EFM32_DAC_CTRL_OUTMODE_MASK           0x00000030
#define     EFM32_DAC_CTRL_OUTMODE(v)           ((EFM32_DAC_CTRL_OUTMODE_##v) << 4)
#define     EFM32_DAC_CTRL_OUTMODE_DIS          0
#define     EFM32_DAC_CTRL_OUTMODE_PIN          1
#define     EFM32_DAC_CTRL_OUTMODE_ADC          2
#define     EFM32_DAC_CTRL_OUTMODE_PINADC       3
#define   EFM32_DAC_CTRL_CONVMODE_MASK          0x0000000c
#define     EFM32_DAC_CTRL_CONVMODE(v)          ((EFM32_DAC_CTRL_CONVMODE_##v) << 2)
#define     EFM32_DAC_CTRL_CONVMODE_CONT        0
#define     EFM32_DAC_CTRL_CONVMODE_HOLD        1
#define     EFM32_DAC_CTRL_CONVMODE_OFF         2
#define   EFM32_DAC_CTRL_SINEMODE_MASK          0x00000002
#define     EFM32_DAC_CTRL_SINEMODE(v)          ((EFM32_DAC_CTRL_SINEMODE_##v) << 1)
#define     EFM32_DAC_CTRL_SINEMODE_DIS         0
#define     EFM32_DAC_CTRL_SINEMODE_EN          1
#define   EFM32_DAC_CTRL_DIFF_MODE_MASK         0x00000001
#define     EFM32_DAC_CTRL_DIFF_MODE(v)         ((EFM32_DAC_CTRL_DIFF_MODE_##v) << 0)
#define     EFM32_DAC_CTRL_DIFF_MODE_SINGLE     0
#define     EFM32_DAC_CTRL_DIFF_MODE_DIFF       1

#define EFM32_DAC_STATUS                        0x004
#define EFM32_DAC_STATUS_MASK                   0x00000003

#define EFM32_DAC_CH0CTRL                       0x008
#define   EFM32_DAC_CH0CTRL_PRSSEL_MASK         0x00000070
#define     EFM32_DAC_CH0CTRL_PRSSEL(v)         ((EFM32_DAC_CH0CTRL_PRSSEL_##v) << 4)
#define       EFM32_DAC_CH0CTRL_PRSSEL_CH0      0
#define       EFM32_DAC_CH0CTRL_PRSSEL_CH1      1
#define       EFM32_DAC_CH0CTRL_PRSSEL_CH2      2
#define       EFM32_DAC_CH0CTRL_PRSSEL_CH3      3
#define       EFM32_DAC_CH0CTRL_PRSSEL_CH4      4
#define       EFM32_DAC_CH0CTRL_PRSSEL_CH5      5
#define       EFM32_DAC_CH0CTRL_PRSSEL_CH6      6
#define       EFM32_DAC_CH0CTRL_PRSSEL_CH7      7
#define   EFM32_DAC_CH0CTRL_PRSEN_MASK          0x00000004
#define   EFM32_DAC_CH0CTRL_REFEN_MASK          0x00000002
#define   EFM32_DAC_CH0CTRL_EN_MASK             0x00000001

#define EFM32_DAC_CH1CTRL                       0x00C
#define   EFM32_DAC_CH1CTRL_PRSSEL_MASK         0x00000070
#define     EFM32_DAC_CH1CTRL_PRSSEL(v)         ((EFM32_DAC_CH1CTRL_PRSSEL_##v) << 4)
#define       EFM32_DAC_CH1CTRL_PRSSEL_CH0      0
#define       EFM32_DAC_CH1CTRL_PRSSEL_CH1      1
#define       EFM32_DAC_CH1CTRL_PRSSEL_CH2      2
#define       EFM32_DAC_CH1CTRL_PRSSEL_CH3      3
#define       EFM32_DAC_CH1CTRL_PRSSEL_CH4      4
#define       EFM32_DAC_CH1CTRL_PRSSEL_CH5      5
#define       EFM32_DAC_CH1CTRL_PRSSEL_CH6      6
#define       EFM32_DAC_CH1CTRL_PRSSEL_CH7      7
#define   EFM32_DAC_CH1CTRL_PRSEN_MASK          0x00000004
#define   EFM32_DAC_CH1CTRL_REFEN_MASK          0x00000002
#define   EFM32_DAC_CH1CTRL_EN_MASK             0x00000001

#define EFM32_DAC_IEN                           0x010
#define EFM32_DAC_IF                            0x014
#define EFM32_DAC_IFS                           0x018
#define EFM32_DAC_IFC                           0x01C

#define EFM32_DAC_CH0DATA                       0x020
#define   EFM32_DAC_CH0DATA_MASK                0x00000fff

#define EFM32_DAC_CH1DATA                       0x024
#define   EFM32_DAC_CH1DATA_MASK                0x00000fff

#define EFM32_DAC_COMBDATA                      0x028
#define   EFM32_DAC_COMBDATA_CH1DATA_MASK       0x0fff0000
#define   EFM32_DAC_COMBDATA_CH1DATA(v)         (((v) & 0xfff) << 16)
#define   EFM32_DAC_COMBDATA_CH0DATA_MASK       0x00000fff
#define   EFM32_DAC_COMBDATA_CH0DATA(v)         (((v) & 0xfff) << 0)

#define EFM32_DAC_CAL                           0x02C
#define EFM32_DAC_BIASPROG                      0x030
