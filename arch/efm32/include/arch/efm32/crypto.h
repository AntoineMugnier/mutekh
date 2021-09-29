
#include "chips.h"

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1 || CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12 \
    || CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14
# include "efr/crypto.h"
#else
# error not supported
#endif
