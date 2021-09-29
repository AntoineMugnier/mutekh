
#include "chips.h"

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
# if (CONFIG_EFM32_FAMILY == EFM32_FAMILY_LEOPARD) \
  || (CONFIG_EFM32_FAMILY == EFM32_FAMILY_WONDER) \
  || (CONFIG_EFM32_FAMILY == EFM32_FAMILY_GIANT)
# include "efm/leopard/clock.h"
#elif (CONFIG_EFM32_FAMILY == EFM32_FAMILY_GECKO)
# include "efm/gecko/clock.h"
#elif (CONFIG_EFM32_FAMILY == EFM32_FAMILY_TINY)
# include "efm/tiny/clock.h"
#elif (CONFIG_EFM32_FAMILY == EFM32_FAMILY_ZERO)
# include "efm/zero/clock.h"
#else
# error
#endif
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14
# include "efr/xg14/clock.h"
#else
# error
#endif

