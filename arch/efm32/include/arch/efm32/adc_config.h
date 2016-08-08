#ifndef _EFM32_ADC_CONFIG_H_
#define _EFM32_ADC_CONFIG_H_

#include <arch/efm32/adc.h>

#define EFM32_ADC_CONFIG_RES_6BITS  (EFM32_ADC_SINGLECTRL_RES_6BITS << 4)
#define EFM32_ADC_CONFIG_RES_8BITS  (EFM32_ADC_SINGLECTRL_RES_8BITS << 4)
#define EFM32_ADC_CONFIG_RES_12BITS (EFM32_ADC_SINGLECTRL_RES_12BITS << 4)

#define EFM32_ADC_CONFIG_REF_1V25       (EFM32_ADC_SINGLECTRL_REF_1V25 << 16)
#define EFM32_ADC_CONFIG_REF_2V5        (EFM32_ADC_SINGLECTRL_REF_2V5 << 16)
#define EFM32_ADC_CONFIG_REF_VDD        (EFM32_ADC_SINGLECTRL_REF_VDD << 16)
#define EFM32_ADC_CONFIG_REF_5VDIFF     (EFM32_ADC_SINGLECTRL_REF_5VDIFF << 16)
#define EFM32_ADC_CONFIG_REF_EXTSINGLE  (EFM32_ADC_SINGLECTRL_REF_EXTSINGLE << 16)
#define EFM32_ADC_CONFIG_REF_2XEXTDIFF  (EFM32_ADC_SINGLECTRL_REF_2XEXTDIFF << 16)
#define EFM32_ADC_CONFIG_REF_2XVDD      (EFM32_ADC_SINGLECTRL_REF_2XVDD << 16)

#define EFM32_ADC_CONFIG_AT_LOG2(__value) ((__value) << 20)

#endif
