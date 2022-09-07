
#include "chips.h"

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
# if EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32LG || \
     EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32GG || \
     EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32GG
#  include "s0/leopard/devaddr.h"
# elif EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32G
#  include "s0/gecko/devaddr.h"
# elif EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32TG
#  include "s0/tiny/devaddr.h"
# elif EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32ZG
#  include "s0/zero/devaddr.h"
# else
#  error
# endif

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
# include "s1/devaddr.h"
#else
# error not supported
#endif
