
#include "chips.h"

# if (CONFIG_EFM32_FAMILY == EFM32_FAMILY_LEOPARD) \
  || (CONFIG_EFM32_FAMILY == EFM32_FAMILY_WONDER) \
  || (CONFIG_EFM32_FAMILY == EFM32_FAMILY_GIANT)
# include "efm/leopard/devaddr.h"
#elif (CONFIG_EFM32_FAMILY == EFM32_FAMILY_GECKO)
# include "efm/gecko/devaddr.h"
#elif (CONFIG_EFM32_FAMILY == EFM32_FAMILY_TINY)
# include "efm/tiny/devaddr.h"
#elif (CONFIG_EFM32_FAMILY == EFM32_FAMILY_ZERO)
# include "efm/zero/devaddr.h"
#elif (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1)
# include "efr/devaddr.h"
#elif (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
# include "efr/devaddr.h"
#elif (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
# include "efr/devaddr.h"
#else
# error not supported
#endif

