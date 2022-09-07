
#include "chips.h"

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
# if EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32LG || \
     EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32GG || \
     EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32GG
#  include "s0/leopard/irq.h"
# elif EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32G
#  include "s0/gecko/irq.h"
# elif EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32TG
#  include "s0/tiny/irq.h"
# elif EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32ZG
#  include "s0/zero/irq.h"
# else
#  error not supported
# endif

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
# if EFM32_CONFIG(CONFIG_EFM32_CFAMILY) == 0
#  include "s1/xg1/irq.h"
# elif EFM32_CONFIG(CONFIG_EFM32_CFAMILY) == 2
#  include "s1/xg12/irq.h"
# elif EFM32_CONFIG(CONFIG_EFM32_CFAMILY) == 4
#  include "s1/xg14/irq.h"
# endif
#else
# error not supported
#endif

