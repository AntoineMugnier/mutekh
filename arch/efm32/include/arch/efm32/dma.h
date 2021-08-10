
#include "chips.h"

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
# include "efm/dma.h"

# if CONFIG_EFM32_FAMILY == EFM32_FAMILY_GECKO \
     || CONFIG_EFM32_FAMILY == EFM32_FAMILY_TINY
#  define EFM32_DMA_CHAN_COUNT 8
# elif CONFIG_EFM32_FAMILY == EFM32_FAMILY_ZERO
#  define EFM32_DMA_CHAN_COUNT 4
# elif CONFIG_EFM32_FAMILY == EFM32_FAMILY_LEOPARD
#  define EFM32_DMA_CHAN_COUNT 12
# elif CONFIG_EFM32_FAMILY == EFM32_FAMILY_WONDER
#  define EFM32_DMA_CHAN_COUNT 12
# elif CONFIG_EFM32_FAMILY == EFM32_FAMILY_GIANT
#  define EFM32_DMA_CHAN_COUNT 12
# endif

# ifndef EFM32_DMA_CHAN_COUNT
#  error
# endif

#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1
# include "efr/dma.h"
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12
# include "efr/dma.h"
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14
# include "efr/dma.h"

#else
# error
#endif

