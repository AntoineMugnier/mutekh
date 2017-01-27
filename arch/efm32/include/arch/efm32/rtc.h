
#include "chips.h"

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
# include "efm/rtc.h"
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1
# include "efr/rtcc.h"
#else
# error not supported
#endif
