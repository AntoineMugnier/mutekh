
#include "chips.h"

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
# include "s0/usart.h"
#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
# include "s1/usart.h"
#else
# error not supported
#endif
