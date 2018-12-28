
#include "chips.h"

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
# include "efm/pin.h"
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1
# include "efr/xg1/pin.h"
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12
# include "efr/xg12/pin.h"
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14
# include "efr/xg12/pin.h"
#else
# error not supported
#endif
