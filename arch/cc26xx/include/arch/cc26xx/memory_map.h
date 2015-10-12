#ifndef _CC26XX_MAP_H_
#define _CC26XX_MAP_H_

  #define CC26XX_FLASHMEM_BASE            0x00000000 // FLASHMEM
  #define CC26XX_BROM_BASE                0x10000000 // BROM
  #define CC26XX_GPRAM_BASE               0x11000000 // GPRAM
  #define CC26XX_SRAM_BASE                0x20000000 // SRAM
  #define CC26XX_RFC_RAM_BASE             0x21000000 // RFRAM
  #define CC26XX_SSI0_BASE                0x40000000 // SSI
  #define CC26XX_UART0_BASE               0x40001000 // UART
  #define CC26XX_I2C0_BASE                0x40002000 // I2C
  #define CC26XX_SSI1_BASE                0x40008000 // SSI
  #define CC26XX_GPT0_BASE                0x40010000 // GPT
  #define CC26XX_GPT1_BASE                0x40011000 // GPT
  #define CC26XX_GPT2_BASE                0x40012000 // GPT
  #define CC26XX_GPT3_BASE                0x40013000 // GPT
  #define CC26XX_UDMA0_BASE               0x40020000 // UDMA
  #define CC26XX_I2S0_BASE                0x40021000 // I2S
  #define CC26XX_GPIO_BASE                0x40022000 // GPIO
  #define CC26XX_CRYPTO_BASE              0x40024000 // CRYPTO
  #define CC26XX_TRNG_BASE                0x40028000 // TRNG
  #define CC26XX_FLASH_BASE               0x40030000 // FLASH
  #define CC26XX_VIMS_BASE                0x40034000 // VIMS
  #define CC26XX_RFC_PWR_BASE             0x40040000 // RFC_PWR
  #define CC26XX_RFC_DBELL_BASE           0x40041000 // RFC_DBELL
  #define CC26XX_RFC_RAT_BASE             0x40043000 // RFC_RAT
  #define CC26XX_RFC_FSCA_BASE            0x40044000 // RFC_FSCA
  #define CC26XX_WDT_BASE                 0x40080000 // WDT
  #define CC26XX_IOC_BASE                 0x40081000 // IOC
  #define CC26XX_PRCM_BASE                0x40082000 // PRCM
  #define CC26XX_EVENT_BASE               0x40083000 // EVENT
  #define CC26XX_SMPH_BASE                0x40084000 // SMPH
  #define CC26XX_ADI2_BASE                0x40086000 // ADI
  #define CC26XX_ADI3_BASE                0x40086200 // ADI
  #define CC26XX_AON_SYSCTL_BASE          0x40090000 // AON_SYSCTL
  #define CC26XX_AON_WUC_BASE             0x40091000 // AON_WUC
  #define CC26XX_AON_RTC_BASE             0x40092000 // AON_RTC
  #define CC26XX_AON_EVENT_BASE           0x40093000 // AON_EVENT
  #define CC26XX_AON_IOC_BASE             0x40094000 // AON_IOC
  #define CC26XX_AON_BATMON_BASE          0x40095000 // AON_BATMON
  #define CC26XX_AUX_AIODIO0_BASE         0x400C1000 // AUX_AIODIO
  #define CC26XX_AUX_AIODIO1_BASE         0x400C2000 // AUX_AIODIO
  #define CC26XX_AUX_TDCIF_BASE           0x400C4000 // AUX_TDC
  #define CC26XX_AUX_EVCTL_BASE           0x400C5000 // AUX_EVCTL
  #define CC26XX_AUX_WUC_BASE             0x400C6000 // AUX_WUC
  #define CC26XX_AUX_TIMER_BASE           0x400C7000 // AUX_TIMER
  #define CC26XX_AUX_SMPH_BASE            0x400C8000 // AUX_SMPH
  #define CC26XX_AUX_ANAIF_BASE           0x400C9000 // AUX_ANAIF
  #define CC26XX_AUX_DDI0_OSC_BASE        0x400CA000 // DDI
  #define CC26XX_AUX_ADI4_BASE            0x400CB000 // ADI
  #define CC26XX_AUX_RAM_BASE             0x400E0000 // AUX_RAM
  #define CC26XX_AUX_SCE_BASE             0x400E1000 // AUX_SCE
  #define CC26XX_FLASH_CFG_BASE           0x50000000 // CC26_DUMMY_COMP
  #define CC26XX_FCFG1_BASE               0x50001000 // FCFG1
  #define CC26XX_FCFG2_BASE               0x50002000 // FCFG2
  #ifndef CC26XX_CCFG_BASE
      #define CC26XX_CCFG_BASE                0x50003000 // CCFG
  #endif
  #define CC26XX_SSI0_NONBUF_BASE         0x60000000 // SSI CPU nonbuf base
  #define CC26XX_UART0_NONBUF_BASE        0x60001000 // UART CPU nonbuf base
  #define CC26XX_I2C0_NONBUF_BASE         0x60002000 // I2C CPU nonbuf base
  #define CC26XX_SSI1_NONBUF_BASE         0x60008000 // SSI CPU nonbuf base
  #define CC26XX_GPT0_NONBUF_BASE         0x60010000 // GPT CPU nonbuf base
  #define CC26XX_GPT1_NONBUF_BASE         0x60011000 // GPT CPU nonbuf base
  #define CC26XX_GPT2_NONBUF_BASE         0x60012000 // GPT CPU nonbuf base
  #define CC26XX_GPT3_NONBUF_BASE         0x60013000 // GPT CPU nonbuf base
  #define CC26XX_UDMA0_NONBUF_BASE        0x60020000 // UDMA CPU nonbuf base
  #define CC26XX_I2S0_NONBUF_BASE         0x60021000 // I2S CPU nonbuf base
  #define CC26XX_GPIO_NONBUF_BASE         0x60022000 // GPIO CPU nonbuf base
  #define CC26XX_CRYPTO_NONBUF_BASE       0x60024000 // CRYPTO CPU nonbuf base
  #define CC26XX_TRNG_NONBUF_BASE         0x60028000 // TRNG CPU nonbuf base
  #define CC26XX_FLASH_NONBUF_BASE        0x60030000 // FLASH CPU nonbuf base
  #define CC26XX_VIMS_NONBUF_BASE         0x60034000 // VIMS CPU nonbuf base
  #define CC26XX_RFC_PWR_NONBUF_BASE      0x60040000 // RFC_PWR CPU nonbuf base
  #define CC26XX_RFC_DBELL_NONBUF_BASE    0x60041000 // RFC_DBELL CPU nonbuf base
  #define CC26XX_RFC_RAT_NONBUF_BASE      0x60043000 // RFC_RAT CPU nonbuf base
  #define CC26XX_RFC_FSCA_NONBUF_BASE     0x60044000 // RFC_FSCA CPU nonbuf base
  #define CC26XX_WDT_NONBUF_BASE          0x60080000 // WDT CPU nonbuf base
  #define CC26XX_IOC_NONBUF_BASE          0x60081000 // IOC CPU nonbuf base
  #define CC26XX_PRCM_NONBUF_BASE         0x60082000 // PRCM CPU nonbuf base
  #define CC26XX_EVENT_NONBUF_BASE        0x60083000 // EVENT CPU nonbuf base
  #define CC26XX_SMPH_NONBUF_BASE         0x60084000 // SMPH CPU nonbuf base
  #define CC26XX_ADI2_NONBUF_BASE         0x60086000 // ADI CPU nonbuf base
  #define CC26XX_ADI3_NONBUF_BASE         0x60086200 // ADI CPU nonbuf base
  #define CC26XX_AON_SYSCTL_NONBUF_BASE   0x60090000 // AON_SYSCTL CPU nonbuf base
  #define CC26XX_AON_WUC_NONBUF_BASE      0x60091000 // AON_WUC CPU nonbuf base
  #define CC26XX_AON_RTC_NONBUF_BASE      0x60092000 // AON_RTC CPU nonbuf base
  #define CC26XX_AON_EVENT_NONBUF_BASE    0x60093000 // AON_EVENT CPU nonbuf base
  #define CC26XX_AON_IOC_NONBUF_BASE      0x60094000 // AON_IOC CPU nonbuf base
  #define CC26XX_AON_BATMON_NONBUF_BASE   0x60095000 // AON_BATMON CPU nonbuf base
  #define CC26XX_AUX_AIODIO0_NONBUF_BASE  0x600C1000 // AUX_AIODIO CPU nonbuf base
  #define CC26XX_AUX_AIODIO1_NONBUF_BASE  0x600C2000 // AUX_AIODIO CPU nonbuf base
  #define CC26XX_AUX_TDCIF_NONBUF_BASE    0x600C4000 // AUX_TDC CPU nonbuf base
  #define CC26XX_AUX_EVCTL_NONBUF_BASE    0x600C5000 // AUX_EVCTL CPU nonbuf base
  #define CC26XX_AUX_WUC_NONBUF_BASE      0x600C6000 // AUX_WUC CPU nonbuf base
  #define CC26XX_AUX_TIMER_NONBUF_BASE    0x600C7000 // AUX_TIMER CPU nonbuf base
  #define CC26XX_AUX_SMPH_NONBUF_BASE     0x600C8000 // AUX_SMPH CPU nonbuf base
  #define CC26XX_AUX_ANAIF_NONBUF_BASE    0x600C9000 // AUX_ANAIF CPU nonbuf base
  #define CC26XX_AUX_DDI0_OSC_NONBUF_BASE 0x600CA000 // DDI CPU nonbuf base
  #define CC26XX_AUX_ADI4_NONBUF_BASE     0x600CB000 // ADI CPU nonbuf base
  #define CC26XX_AUX_RAM_NONBUF_BASE      0x600E0000 // AUX_RAM CPU nonbuf base
  #define CC26XX_AUX_SCE_NONBUF_BASE      0x600E1000 // AUX_SCE CPU nonbuf base
  #define CC26XX_FLASHMEM_ALIAS_BASE      0xA0000000 // FLASHMEM Alias base
  #define CC26XX_CPU_ITM_BASE             0xE0000000 // CPU_ITM
  #define CC26XX_CPU_DWT_BASE             0xE0001000 // CPU_DWT
  #define CC26XX_CPU_FPB_BASE             0xE0002000 // CPU_FPB
  #define CC26XX_CPU_SCS_BASE             0xE000E000 // CPU_SCS
  #define CC26XX_CPU_TPIU_BASE            0xE0040000 // CPU_TPIU
  #define CC26XX_CPU_TIPROP_BASE          0xE00FE000 // CPU_TIPROP
  #define CC26XX_CPU_ROM_TABLE_BASE       0xE00FF000 // CPU_ROM_TABLE

#endif

