
#include "chips.h"

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
# include "efm/msc.h"

# if (CONFIG_EFM32_FAMILY == EFM32_FAMILY_GECKO) || (CONFIG_EFM32_FAMILY == EFM32_FAMILY_TINY)
#  define EFM32_FLASH_PAGE_SIZE 9
# elif CONFIG_EFM32_FAMILY == EFM32_FAMILY_ZERO
#  define EFM32_FLASH_PAGE_SIZE 10
# elif CONFIG_EFM32_FAMILY == EFM32_FAMILY_LEOPARD
#  define EFM32_FLASH_PAGE_SIZE 11
# elif CONFIG_EFM32_FAMILY == EFM32_FAMILY_WONDER
#  define EFM32_FLASH_PAGE_SIZE 11
# elif CONFIG_EFM32_FAMILY == EFM32_FAMILY_GIANT
#  define EFM32_FLASH_PAGE_SIZE 12
# endif

#elif (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) || \
      (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) ||\
      (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
# include "efr/msc.h"

# if CONFIG_EFM32_FAMILY == EFM32_FAMILY_PEARL
#  define EFM32_FLASH_PAGE_SIZE 11
# elif CONFIG_EFM32_FAMILY == EFM32_FAMILY_JADE
#  define EFM32_FLASH_PAGE_SIZE 11
# elif CONFIG_EFM32_FAMILY == EFM32_FAMILY_FLEX
#  define EFM32_FLASH_PAGE_SIZE 11
# elif CONFIG_EFM32_FAMILY == EFM32_FAMILY_MIGHTY
#  define EFM32_FLASH_PAGE_SIZE 11
# elif CONFIG_EFM32_FAMILY == EFM32_FAMILY_BLUE
#  define EFM32_FLASH_PAGE_SIZE 11
# endif

#else
# error
#endif

#ifndef EFM32_FLASH_PAGE_SIZE
# error
#endif

#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
#  define EFM32_MSC_ADDR 0x400e0000
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
#  define EFM32_MSC_ADDR 0x400c0000
#else
#  error
#endif
