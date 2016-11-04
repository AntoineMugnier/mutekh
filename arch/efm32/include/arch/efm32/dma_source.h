

# if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO)
# include "efm/leopard/dma_source.h"
#elif defined(CONFIG_EFM32_GECKO)
# include "efm/gecko/dma_source.h"
#elif defined(CONFIG_EFM32_ZERO_GECKO)
# include "efm/zero/dma_source.h"
#elif defined(CONFIG_EFM32_HAPPY_GECKO)
# include "efm/happy/dma_source.h"
#else
# error
#endif

