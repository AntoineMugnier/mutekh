
#include "../chips.h"

#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12)
# include "xg12/bufc.h"
#elif (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
# include "xg12/bufc.h"
#else
# error not supported
#endif
