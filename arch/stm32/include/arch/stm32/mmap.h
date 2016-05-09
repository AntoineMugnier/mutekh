
#if CONFIG_STM32_FAMILY == 1
# include <arch/stm32/f1/mmap.h>
#elif CONFIG_STM32_FAMILY == 4
# include <arch/stm32/f4/mmap.h>
#elif CONFIG_STM32_FAMILY == L4
# include <arch/stm32/l4/mmap.h>
#else
# error unspecified architecture
#endif