
#include "chips.h"

# if (CONFIG_EFM32_FAMILY == EFM32_FAMILY_LEOPARD) \
  || (CONFIG_EFM32_FAMILY == EFM32_FAMILY_WONDER) \
  || (CONFIG_EFM32_FAMILY == EFM32_FAMILY_GIANT)
# include "efm/leopard/irq.h"
#elif (CONFIG_EFM32_FAMILY == EFM32_FAMILY_GECKO)
# include "efm/gecko/irq.h"
#elif (CONFIG_EFM32_FAMILY == EFM32_FAMILY_TINY)
# include "efm/tiny/irq.h"
#elif (CONFIG_EFM32_FAMILY == EFM32_FAMILY_ZERO)
# include "efm/zero/irq.h"
#elif (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1)
# include "efr/xg1/irq.h"
#elif (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
# include "efr/xg12/irq.h"
#elif (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
# include "efr/xg14/irq.h"
#else
# error not supported
#endif

