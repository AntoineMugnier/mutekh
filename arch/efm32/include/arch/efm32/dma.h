
#if defined(CONFIG_EFM32_GECKO) || \
   defined(CONFIG_EFM32_ZERO_GECKO) || \
   defined(CONFIG_EFM32_LEOPARD_GECKO) || \
   defined(CONFIG_EFM32_WONDER_GECKO) || \
   defined(CONFIG_EFM32_GIANT_GECKO)
# include "efm/dma.h"
#else
# error
#endif
