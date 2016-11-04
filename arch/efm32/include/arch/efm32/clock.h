
# if defined(CONFIG_EFM32_LEOPARD_GECKO) \
  || defined(CONFIG_EFM32_WONDER_GECKO) \
  || defined(CONFIG_EFM32_GIANT_GECKO)
# include "efm/leopard/clock.h"
#elif defined(CONFIG_EFM32_GECKO)
# include "efm/gecko/clock.h"
#elif defined(CONFIG_EFM32_ZERO_GECKO)
# include "efm/zero/clock.h"
#else
# error
#endif

