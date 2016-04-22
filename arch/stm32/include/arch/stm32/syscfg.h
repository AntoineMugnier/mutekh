
#if CONFIG_STM32_FAMILY == 4
# include <arch/stm32/f4/syscfg.h>
#elif CONFIG_STM32_FAMILY == L4
# include <arch/stm32/l4/syscfg.h>
#else
# error unspecified architecture
#endif
