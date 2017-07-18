
#include "chips.h"

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
# include "efm/usart.h"
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1
# include "efr/usart.h"
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12
# include "efr/usart.h"
#else
# error not supported
#endif
