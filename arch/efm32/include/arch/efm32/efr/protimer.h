
#include "../chips.h"

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12
# include "xg12/protimer.h"
#else
# error not supported
#endif
