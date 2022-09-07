
#include "chips.h"

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
# include "s0/aes.h"
#else
# error not supported
#endif
