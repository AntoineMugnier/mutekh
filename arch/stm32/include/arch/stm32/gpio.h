
#if CONFIG_STM32_FAMILY == 1
# include <arch/stm32/f1/gpio.h>
#elif CONFIG_STM32_FAMILY == 4
# include <arch/stm32/f4/gpio.h>
#else
# error unspecified architecture
#endif
