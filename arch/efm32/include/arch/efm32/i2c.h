
#include "chips.h"

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
# include "efm/i2c.h"
#elif (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) \
  ||  (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
# include "efr/i2c.h"
#else
# error not supported
#endif
