
#if CONFIG_STM32_FAMILY == 1
# include <arch/stm32/f1/irq.h>
#elif CONFIG_STM32_FAMILY == 4 || CONFIG_STM32_FAMILY == L4
# include <arch/stm32/f4/irq.h>
#else
# error unspecified architecture
#endif
