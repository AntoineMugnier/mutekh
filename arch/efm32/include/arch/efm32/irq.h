
# if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO)
# include "efm/leopard/irq.h"
#elif defined(CONFIG_EFM32_GECKO)
# include "efm/gecko/irq.h"
#elif defined(CONFIG_EFM32_ZERO_GECKO)
# include "efm/zero/irq.h"
#else
# include "efr/irq.h"
#endif

