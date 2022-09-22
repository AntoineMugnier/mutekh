
#include <hexo/types.h>

/** see hwrand.S */
uint32_t efm32_hw_rand32(void);

#define HWRAND_CRC32_POLY 0xe12390be
#define HWRAND_CRC32_INIT 0xff
#define HWRAND_CRC32_ALL1 0xa30864d0
#define HWRAND_CRC32_ALL0 0xdf6e5483
