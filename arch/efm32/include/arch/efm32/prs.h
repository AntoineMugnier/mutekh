
#include "chips.h"

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
# include "efm/prs.h"
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12
# include "efr/xg12/prs.h"
#else
# error not supported
#endif
