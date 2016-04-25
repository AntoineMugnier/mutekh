
#if CONFIG_STM32_FAMILY == 1 || CONFIG_STM32_FAMILY == 4
# include <arch/stm32/f1/usart.h>
#elif CONFIG_STM32_FAMILY == L4
# include <arch/stm32/l4/usart.h>
#endif
