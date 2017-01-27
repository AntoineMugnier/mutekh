
#include "chips.h"

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1
# include "efm/crypto.h"
#else
# error not supported
#endif
