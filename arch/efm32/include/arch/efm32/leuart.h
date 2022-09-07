
#include "chips.h"

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
# include "s0/leuart.h"
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
# include "s1/leuart.h"
#else
# error not supported
#endif
