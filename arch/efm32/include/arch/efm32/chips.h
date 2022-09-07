
/**
  @multiple List of efm32 micro-controller families as defined by MutekH.
  see https://community.silabs.com/s/article/series-0-and-series-1-efm32-ezr32-efr32-device
  EFM32_MCU_* numbers are not specified by silabs.
*/

#define EFM32_MCU_EFM32G     1
#define EFM32_MCU_EFM32GG    2
#define EFM32_MCU_EFM32TG    3
#define EFM32_MCU_EFM32LG    4
#define EFM32_MCU_EFM32WG    5
#define EFM32_MCU_EFM32ZG    6
#define EFM32_MCU_EFM32HG    7
#define EFM32_MCU_EFR32MG1  11
#define EFM32_MCU_EFR32BG1  12
#define EFM32_MCU_EFR32FG1  13
#define EFM32_MCU_EFR32MG12 14
#define EFM32_MCU_EFR32BG12 15
#define EFM32_MCU_EFR32FG12 16
#define EFM32_MCU_EFR32MG13 17
#define EFM32_MCU_EFR32BG13 18
#define EFM32_MCU_EFR32FG13 19
#define EFM32_MCU_EFR32MG14 20
#define EFM32_MCU_EFR32BG14 21
#define EFM32_MCU_EFR32FG14 22
#define EFM32_MCU_EFM32PG1  23
#define EFM32_MCU_EFM32JG1  24
#define EFM32_MCU_EFM32PG12 25
#define EFM32_MCU_EFM32JG12 26
#define EFM32_MCU_EFM32PG13 27
#define EFM32_MCU_EFM32JG13 28
#define EFM32_MCU_EFM32GG11 29
#define EFM32_MCU_EFM32TG11 30
#define EFM32_MCU_EFM32GG12 31
#define EFM32_MCU_EFM32FG22 50
#define EFM32_MCU_EFM32FG23 51
#define EFM32_MCU_EFM32MG22 52
#define EFM32_MCU_EFM32MG23 53
#define EFM32_MCU_EFM32BG22 54
#define EFM32_MCU_EFM32BG23 55
#define EFM32_MCU_EFM32PG22 56
#define EFM32_MCU_EFM32PG23 57

/** @internal Generates a composite family value that embed various
    values related to identification of an efm32 chip. */
#define EFM32_CFAMILY(series, config, family_num, mcu) ((series << 20) | (config << 16) | (family_num << 8) | (mcu))

/** Extracts the series number as defined by silabs (0, 1 or 2) from
the composite family value. This is the first number in the family
name, unless zero. */
#define EFM32_SERIES(cfamily) (((cfamily) >> 20) & 0xf)

/** Extracts the config number as defined by silabs from the composite
family value. This is the second number in the family name, unless
zero. */
#define EFM32_CONFIG(cfamily) (((cfamily) >> 16) & 0xf)

/** Extracts the family number as defined by silabs from the
   composite family value. see EFM32PG22 ref manual for a list of
   values. */
#define EFM32_FAMILY(cfamily) (((cfamily) >> 8) & 0xff)

/** Extracts the MutekH efm32 family number from the composite family
    value. The silabs family number is not used because it is not
    designed to be unique across all series. */
#define EFM32_MCU(cfamily) ((cfamily) & 0xff)

/** @multiple Composite family value. Available in @ref #CONFIG_EFM32_CFAMILY */
#define EFM32_CFAMILY_EFM32G       EFM32_CFAMILY(0, 0, 71,  EFM32_MCU_EFM32G)  // EFM32 Gecko Device Family
#define EFM32_CFAMILY_EFM32GG      EFM32_CFAMILY(0, 0, 72,  EFM32_MCU_EFM32GG) // EFM32 Giant Gecko Device Family
#define EFM32_CFAMILY_EFM32TG      EFM32_CFAMILY(0, 0, 73,  EFM32_MCU_EFM32TG) // EFM32 Tiny Gecko Device Family
#define EFM32_CFAMILY_EFM32LG      EFM32_CFAMILY(0, 0, 74,  EFM32_MCU_EFM32LG) // EFM32 Leopard Gecko Device Family
#define EFM32_CFAMILY_EFM32WG      EFM32_CFAMILY(0, 0, 75,  EFM32_MCU_EFM32WG) // EFM32 Wonder Gecko Device Family
#define EFM32_CFAMILY_EFM32ZG      EFM32_CFAMILY(0, 0, 76,  EFM32_MCU_EFM32ZG) // EFM32 Zero Gecko Device Family
#define EFM32_CFAMILY_EFM32HG      EFM32_CFAMILY(0, 0, 77,  EFM32_MCU_EFM32HG) // EFM32 Happy Gecko Device Family
#define EFM32_CFAMILY_EZR32LG      EFM32_CFAMILY(0, 0, 120, EFM32_MCU_EFM32LG) // EZR32 Leopard Gecko Device Family
#define EFM32_CFAMILY_EZR32WG      EFM32_CFAMILY(0, 0, 121, EFM32_MCU_EFM32WG) // EZR32 Wonder Gecko Device Family
#define EFM32_CFAMILY_EZR32HG      EFM32_CFAMILY(0, 0, 122, EFM32_MCU_EFM32HG) // EZR32 Happy Gecko Device Family

#define EFM32_CFAMILY_EFR32MG1P    EFM32_CFAMILY(1, 0, 16,  EFM32_MCU_EFR32MG1)  // EFR32 Mighty Gecko Family Series 1 Device Config 1
#define EFM32_CFAMILY_EFR32MG1B    EFM32_CFAMILY(1, 0, 17,  EFM32_MCU_EFR32MG1)  // EFR32 Mighty Gecko Family Series 1 Device Config 1
#define EFM32_CFAMILY_EFR32MG1V    EFM32_CFAMILY(1, 0, 18,  EFM32_MCU_EFR32MG1)  // EFR32 Mighty Gecko Family Series 1 Device Config 1
#define EFM32_CFAMILY_EFR32BG1P    EFM32_CFAMILY(1, 0, 19,  EFM32_MCU_EFR32BG1)  // EFR32 Blue Gecko Family Series 1 Device Config 1
#define EFM32_CFAMILY_EFR32BG1B    EFM32_CFAMILY(1, 0, 20,  EFM32_MCU_EFR32BG1)  // EFR32 Blue Gecko Family Series 1 Device Config 1
#define EFM32_CFAMILY_EFR32BG1V    EFM32_CFAMILY(1, 0, 21,  EFM32_MCU_EFR32BG1)  // EFR32 Blue Gecko Family Series 1 Device Config 1
#define EFM32_CFAMILY_EFR32FG1P    EFM32_CFAMILY(1, 0, 25,  EFM32_MCU_EFR32FG1)  // EFR32 Flex Gecko Family Series 1 Device Config 1
#define EFM32_CFAMILY_EFR32FG1B    EFM32_CFAMILY(1, 0, 26,  EFM32_MCU_EFR32FG1)  // EFR32 Flex Gecko Family Series 1 Device Config 1
#define EFM32_CFAMILY_EFR32FG1V    EFM32_CFAMILY(1, 0, 27,  EFM32_MCU_EFR32FG1)  // EFR32 Flex Gecko Family Series 1 Device Config 1
#define EFM32_CFAMILY_EFR32MG12P   EFM32_CFAMILY(1, 2, 28,  EFM32_MCU_EFR32MG1)  // EFR32 Mighty Gecko Family Series 1 Device Config 2
//#define EFM32_CFAMILY_EFR32MG12B   EFM32_CFAMILY(1, 29,  EFM32_MCU_EFR32MG1B)  // EFR32 Mighty Gecko Family Series 1 Device Config 2
//#define EFM32_CFAMILY_EFR32MG12V   EFM32_CFAMILY(1, 30,  EFM32_MCU_EFR32MG1V)  // EFR32 Mighty Gecko Family Series 1 Device Config 2
#define EFM32_CFAMILY_EFR32BG12P   EFM32_CFAMILY(1, 2, 31,  EFM32_MCU_EFR32BG1)  // EFR32 Blue Gecko Family Series 1 Device Config 2
//#define EFM32_CFAMILY_EFR32BG12B   EFM32_CFAMILY(1, 32,  EFM32_MCU_EFR32BG1B)  // EFR32 Blue Gecko Family Series 1 Device Config 2
//#define EFM32_CFAMILY_EFR32BG12V   EFM32_CFAMILY(1, 33,  EFM32_MCU_EFR32BG1V)  // EFR32 Blue Gecko Family Series 1 Device Config 2
#define EFM32_CFAMILY_EFR32FG12P   EFM32_CFAMILY(1, 2, 37,  EFM32_MCU_EFR32FG1)  // EFR32 Flex Gecko Family Series 1 Device Config 2
//#define EFM32_CFAMILY_EFR32FG12B   EFM32_CFAMILY(1, 38,  EFM32_MCU_EFR32FG1B)  // EFR32 Flex Gecko Family Series 1 Device Config 2
//#define EFM32_CFAMILY_EFR32FG12V   EFM32_CFAMILY(1, 39,  EFM32_MCU_EFR32FG1V)  // EFR32 Flex Gecko Family Series 1 Device Config 2
#define EFM32_CFAMILY_EFR32MG13P   EFM32_CFAMILY(1, 3, 40,  EFM32_MCU_EFR32MG1)  // EFR32 Mighty Gecko Family Series 13 Device Config 3
//#define EFM32_CFAMILY_EFR32MG13B   EFM32_CFAMILY(1, 41,  EFM32_MCU_EFR32MG1B)  // EFR32 Mighty Gecko Family Series 13 Device Config 3
//#define EFM32_CFAMILY_EFR32MG13V   EFM32_CFAMILY(1, 42,  EFM32_MCU_EFR32MG1V)  // EFR32 Mighty Gecko Family Series 1 Device Config 3
#define EFM32_CFAMILY_EFR32BG13P   EFM32_CFAMILY(1, 3, 43,  EFM32_MCU_EFR32BG1)  // EFR32 Blue Gecko Family Series 1 Device Config 3
//#define EFM32_CFAMILY_EFR32BG13B   EFM32_CFAMILY(1, 44,  EFM32_MCU_EFR32BG1B)  // EFR32 Blue Gecko Family Series 1 Device Config 3
//#define EFM32_CFAMILY_EFR32BG13V   EFM32_CFAMILY(1, 45,  EFM32_MCU_EFR32BG1V)  // EFR32 Blue Gecko Family Series 1 Device Config 3
#define EFM32_CFAMILY_EFR32FG13P   EFM32_CFAMILY(1, 3, 49,  EFM32_MCU_EFR32FG1)  // EFR32 Flex Gecko Family Series 1 Device Config 3
//#define EFM32_CFAMILY_EFR32FG13B   EFM32_CFAMILY(1, 50,  EFM32_MCU_EFR32FG1B)  // EFR32 Flex Gecko Family Series 1 Device Config 3
//#define EFM32_CFAMILY_EFR32FG13V   EFM32_CFAMILY(1, 51,  EFM32_MCU_EFR32FG1V)  // EFR32 Flex Gecko Family Series 1 Device Config 3
#define EFM32_CFAMILY_EFR32MG14P   EFM32_CFAMILY(1, 4, 52,  EFM32_MCU_EFR32MG1)  // EFR32 Mighty Gecko Family Series 1 Device Config 4
//#define EFM32_CFAMILY_EFR32MG14B   EFM32_CFAMILY(1, 53,  EFM32_MCU_EFR32MG1B)  // EFR32 Mighty Gecko Family Series 1 Device Config 4
//#define EFM32_CFAMILY_EFR32MG14V   EFM32_CFAMILY(1, 54,  EFM32_MCU_EFR32MG1V)  // EFR32 Mighty Gecko Family Series 1 Device Config 4
#define EFM32_CFAMILY_EFR32BG14P   EFM32_CFAMILY(1, 4, 55,  EFM32_MCU_EFR32BG1)  // EFR32 Blue Gecko Family Series 1 Device Config 4
//#define EFM32_CFAMILY_EFR32BG14B   EFM32_CFAMILY(1, 56,  EFM32_MCU_EFR32BG1B)  // EFR32 Blue Gecko Family Series 1 Device Config 4
//#define EFM32_CFAMILY_EFR32BG14V   EFM32_CFAMILY(1, 57,  EFM32_MCU_EFR32BG1V)  // EFR32 Blue Gecko Family Series 1 Device Config 4
#define EFM32_CFAMILY_EFR32FG14P   EFM32_CFAMILY(1, 4, 61,  EFM32_MCU_EFR32FG1)  // EFR32 Flex Gecko Family Series 1 Device Config 4
//#define EFM32_CFAMILY_EFR32FG14B   EFM32_CFAMILY(1, 62,  EFM32_MCU_EFR32FG1B)  // EFR32 Flex Gecko Family Series 1 Device Config 4
//#define EFM32_CFAMILY_EFR32FG14V   EFM32_CFAMILY(1, 63,  EFM32_MCU_EFR32FG1V)  // EFR32 Flex Gecko Family Series 1 Device Config 4
#define EFM32_CFAMILY_EFM32PG1B    EFM32_CFAMILY(1, 0, 81,  EFM32_MCU_EFM32PG1)  // EFM32 Pearl Gecko Device Family Series 1 Device Config 1
#define EFM32_CFAMILY_EFM32JG1B    EFM32_CFAMILY(1, 0, 83,  EFM32_MCU_EFM32JG1)  // EFM32 Jade Gecko Device Family Series 1 Device Config 1
#define EFM32_CFAMILY_EFM32GG11B   EFM32_CFAMILY(1, 1, 100, EFM32_MCU_EFM32GG11) // EFM32 Giant Gecko Device Family Series 1 Device Config 1
#define EFM32_CFAMILY_EFM32GG12B   EFM32_CFAMILY(1, 1, 106, EFM32_MCU_EFM32GG12) // EFM32 Giant Gecko Device Family Series 1 Device Config 2
#define EFM32_CFAMILY_EFM32TG11B   EFM32_CFAMILY(1, 1, 103, EFM32_MCU_EFM32TG11) // EFM32 Giant Gecko Device Family Series 1 Device Config 1
#define EFM32_CFAMILY_EFM32PG12B   EFM32_CFAMILY(1, 2, 85,  EFM32_MCU_EFM32PG12)  // EFM32 Pearl Gecko Device Family Series 1 Device Config 2
#define EFM32_CFAMILY_EFM32JG12B   EFM32_CFAMILY(1, 2, 87,  EFM32_MCU_EFM32JG12)  // EFM32 Jade Gecko Device Family Series 1 Device Config 2
#define EFM32_CFAMILY_EFM32PG13B   EFM32_CFAMILY(1, 3, 89,  EFM32_MCU_EFM32PG13)  // EFM32 Pearl Gecko Device Family Series 1 Device Config 3
#define EFM32_CFAMILY_EFM32JG13B   EFM32_CFAMILY(1, 3, 91,  EFM32_MCU_EFM32JG13)  // EFM32 Jade Gecko Device Family Series 1 Device Config 3

#define EFM32_CFAMILY_EFM32FG22    EFM32_CFAMILY(2, 2, 0, EFM32_MCU_EFM32FG22) // Series 2 Flex Gecko
#define EFM32_CFAMILY_EFM32FG23    EFM32_CFAMILY(2, 3, 0, EFM32_MCU_EFM32FG23) // Series 2 Flex Gecko
#define EFM32_CFAMILY_EFM32MG22    EFM32_CFAMILY(2, 2, 1, EFM32_MCU_EFM32MG22) // Series 2 Mighty Gecko
#define EFM32_CFAMILY_EFM32MG23    EFM32_CFAMILY(2, 3, 1, EFM32_MCU_EFM32MG23) // Series 2 Mighty Gecko
#define EFM32_CFAMILY_EFM32BG22    EFM32_CFAMILY(2, 2, 2, EFM32_MCU_EFM32BG22) // Series 2 Blue Gecko
#define EFM32_CFAMILY_EFM32BG23    EFM32_CFAMILY(2, 3, 2, EFM32_MCU_EFM32BG23) // Series 2 Blue Gecko
#define EFM32_CFAMILY_EFM32PG22    EFM32_CFAMILY(2, 2, 5, EFM32_MCU_EFM32PG22) // Series 2 Pearl Gecko
#define EFM32_CFAMILY_EFM32PG23    EFM32_CFAMILY(2, 3, 5, EFM32_MCU_EFM32PG23) // Series 2 Pearl Gecko
