
#include "chips.h"

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
# include "s0/i2c.h"
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
# include "s1/i2c.h"
#else
# error not supported
#endif
