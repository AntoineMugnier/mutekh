
#include "chips.h"

# if (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32LG) \
  || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32WG) \
  || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32GG)
# include "s0/leopard/dma_source.h"
#elif (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32G)
# include "s0/gecko/dma_source.h"
#elif (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32TG)
# include "s0/tiny/dma_source.h"
#elif (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32ZG)
# include "s0/zero/dma_source.h"
#elif (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32HG)
# include "s0/happy/dma_source.h"
#else
# include "s1/dma_source.h"
#endif

