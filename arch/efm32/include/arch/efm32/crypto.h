
#include "chips.h"

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
# include "s1/crypto.h"
#else
# error not supported
#endif
