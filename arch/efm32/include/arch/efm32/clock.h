
#include "chips.h"

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
# if EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32LG || \
     EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32GG || \
     EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32GG
#  include "s0/leopard/clock.h"
# elif EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32G
#  include "s0/gecko/clock.h"
# elif EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32TG
#  include "s0/tiny/clock.h"
# elif EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32ZG
#  include "s0/zero/clock.h"
# else
#  error
# endif
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
# if EFM32_CONFIG(CONFIG_EFM32_CFAMILY) == 4
#  include "s1/xg14/clock.h"
# else
#  error
# endif
#else
# error
#endif
