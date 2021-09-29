
#include "chips.h"

# if (CONFIG_EFM32_FAMILY == EFM32_FAMILY_LEOPARD) \
  || (CONFIG_EFM32_FAMILY == EFM32_FAMILY_WONDER) \
  || (CONFIG_EFM32_FAMILY == EFM32_FAMILY_GIANT)
# include "efm/leopard/dma_source.h"
#elif (CONFIG_EFM32_FAMILY == EFM32_FAMILY_GECKO)
# include "efm/gecko/dma_source.h"
#elif (CONFIG_EFM32_FAMILY == EFM32_FAMILY_TINY)
# include "efm/tiny/dma_source.h"
#elif (CONFIG_EFM32_FAMILY == EFM32_FAMILY_ZERO)
# include "efm/zero/dma_source.h"
#elif (CONFIG_EFM32_FAMILY == EFM32_FAMILY_HAPPY)
# include "efm/happy/dma_source.h"
#else
# include "efr/dma_source.h"
#endif

