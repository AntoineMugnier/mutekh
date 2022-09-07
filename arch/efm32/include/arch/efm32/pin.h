
#include "chips.h"

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
# include "s0/pin.h"

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
# if EFM32_CONFIG(CONFIG_EFM32_CFAMILY) == 0
#  include "s1/xg1/pin.h"
# elif EFM32_CONFIG(CONFIG_EFM32_CFAMILY) == 2
#  include "s1/xg12/pin.h"
# elif EFM32_CONFIG(CONFIG_EFM32_CFAMILY) == 4
#  include "s1/xg12/pin.h"      /* FIXME */
# else
#  error not supported
# endif
#else
# error not supported
#endif
