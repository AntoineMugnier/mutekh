
# if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO)
# include "efm/leopard/devaddr.h"
#elif defined(CONFIG_EFM32_GECKO)
# include "efm/gecko/devaddr.h"
#elif defined(CONFIG_EFM32_ZERO_GECKO)
# include "efm/zero/devaddr.h"
#else
# include "efr/devaddr.h"
#endif

