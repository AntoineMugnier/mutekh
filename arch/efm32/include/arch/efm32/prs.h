
#include "chips.h"

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0
# include "s0/prs.h"

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
# if EFM32_CONFIG(CONFIG_EFM32_CFAMILY) == 2
#  include "s1/xg12/prs.h"
# elif EFM32_CONFIG(CONFIG_EFM32_CFAMILY) == 4
#  include "s1/xg12/prs.h"      /* FIXME */
# else
#  error not supported
#else
# error not supported
#endif
