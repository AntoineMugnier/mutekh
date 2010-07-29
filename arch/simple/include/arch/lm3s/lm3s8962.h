//*****************************************************************************
//
// LM3S8962 Register Definitions
//
//*****************************************************************************

#ifndef __LM3S8962_H__
#define __LM3S8962_H__

typedef uint32_t LM3S_REG;

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR System Peripherals
// *****************************************************************************

typedef struct _LM3SS_WDT {
  LM3S_REG  WDT_LOADR; // Watchdog Timer Load Register
  LM3S_REG  WDT_VALUER; // Watchdog Timer Value Register
  LM3S_REG  WDT_CTLR; // Watchdog Timer Control Register
  LM3S_REG  WDT_ICRR; // Watchdog Timer Interrupt Clear Register
  LM3S_REG  WDT_RISR; // Watchdog Timer Raw Interrupt Status Register
  LM3S_REG  WDT_MISR; // Watchdog Timer Masked Interrupt Status Register
  LM3S_REG  Reserved0[256];
  LM3S_REG  WDT_TESTR; // Watchdog Timer Test Register
  LM3S_REG  Reserved1[505];
  LM3S_REG  WDT_LOCKR; // Watchdog Timer Lock Register
} LM3SS_WDT, *LM3SPS_WDT;

typedef struct _LM3SS_GPIO {
  LM3S_REG  GPIO_DATAR[256]; // GPIO Data Registers
  LM3S_REG  GPIO_DIRR;       // GPIO Direction Register
  LM3S_REG  GPIO_ISR;        // GPIO Interrupt Sense Register
  LM3S_REG  GPIO_IBER;       // GPIO Interrupt Both Edges Register
  LM3S_REG  GPIO_IEVR;       // GPIO Interrupt Event Register
  LM3S_REG  GPIO_IMR;        // GPIO Interrupt Mask Register
  LM3S_REG  GPIO_RISR;       // GPIO Raw Interrupt Status Register
  LM3S_REG  GPIO_MISR;       // GPIO Masked Interrupt Status Register
  LM3S_REG  GPIO_ICRR;       // GPIO Interrupt Clear Register
  LM3S_REG  GPIO_AFSELR;     // GPIO Alternate Function Select Register
  LM3S_REG  Reserved0[55];
  LM3S_REG  GPIO_DR2RR;      // GPIO 2-mA Drive Select Register
  LM3S_REG  GPIO_DR4RR;      // GPIO 4-mA Drive Select Register
  LM3S_REG  GPIO_DR8RR;      // GPIO 8-mA Drive Select Register
  LM3S_REG  GPIO_ODRR;       // GPIO Open Drain Select Register
  LM3S_REG  GPIO_PURR;       // GPIO Pull-Up Select Register
  LM3S_REG  GPIO_PDRR;       // GPIO Pull-Down Select Register
  LM3S_REG  GPIO_SLRR;       // GPIO Slew-Rate Control Select Register
  LM3S_REG  GPIO_DENR;       // GPIO Digital Enable Register
  LM3S_REG  GPIO_LOCKR;      // GPIO Lock Register
  LM3S_REG  GPIO_CRR;        // GPIO Commit Register
} LM3SS_GPIO, *LM3SPS_GPIO;

typedef struct _LM3SS_SYSCTL {
  LM3S_REG  SYSCTL_DIDR[2];    // System Control Device Identification Registers
  LM3S_REG  SYSCTL_DCR[6];     // System Control Device Capabilities Registers
  LM3S_REG  Reserved0[4];
  LM3S_REG  SYSCTL_PBORCTLR;   // System Control Brown-Out Reset Control Register
  LM3S_REG  SYSCTL_LDOPCTLR;   // System Control LDO Power Control Register
  LM3S_REG  Reserved1[2];
  LM3S_REG  SYSCTL_SRCRR[3];   // System Control Software Reset Control Registers 
  LM3S_REG  Reserved2;
  LM3S_REG  SYSCTL_RISR;       // System Control Raw Interrupt Status Register   
  LM3S_REG  SYSCTL_IMCR;       // System Control Interrupt Mask Control Register   
  LM3S_REG  SYSCTL_MISCR;      // System Control Masked Interrupt Status and Clear Register   
  LM3S_REG  SYSCTL_RESCR;      // System Control Reset Cause Register   
  LM3S_REG  SYSCTL_RCCR;       // System Control Run-Mode Clock Configuration Register
  LM3S_REG  SYSCTL_PLLCFGR;    // System Control XTAL to PLL Translation Register
  LM3S_REG  Reserved3[2];
  LM3S_REG  SYSCTL_RCC2R;      // System Control Run Mode Clock Configuration 2 Register
  LM3S_REG  Reserved4[35];
  LM3S_REG  SYSCTL_RCGCR[3];   // System Control Run Mode Clock Gating Control Registers
  LM3S_REG  Reserved5;
  LM3S_REG  SYSCTL_SCGCR[3];   // System Control Sleep Mode Clock Gating Control Registers
  LM3S_REG  Reserved6;
  LM3S_REG  SYSCTL_DCGCR[3];   // System Control Deep Sleep Mode Clock Gating Control Register
  LM3S_REG  Reserved7[6];
  LM3S_REG  SYSCTL_DSLPCLKCFG; // System Control Deep Sleep Clock Configuration Register
} LM3SS_SYSCTL, *LM3SPS_SYSCTL;

// *****************************************************************************
//               BASE ADDRESS DEFINITIONS FOR LM3S8962 devices
// *****************************************************************************
#define LM3S_BASE_WDT       ((LM3SPS_WDT)       0x40000000) // WDT Base Address 
#define LM3S_BASE_GPIOA     ((LM3SPS_GPIO)      0x40004000) // GPIOA Base Address
#define LM3S_BASE_GPIOB     ((LM3SPS_GPIO)      0x40005000) // GPIOB Base Address
#define LM3S_BASE_GPIOC     ((LM3SPS_GPIO)      0x40006000) // GPIOC Base Address
#define LM3S_BASE_GPIOD     ((LM3SPS_GPIO)      0x40007000) // GPIOD Base Address
#define LM3S_BASE_GPIOE     ((LM3SPS_GPIO)      0x40024000) // GPIOE Base Address
#define LM3S_BASE_GPIOF     ((LM3SPS_GPIO)      0x40025000) // GPIOF Base Address
#define LM3S_BASE_GPIOG     ((LM3SPS_GPIO)      0x40026000) // GPIOG Base Address
#define LM3S_BASE_SYSCTL    ((LM3SPS_SYSCTL)    0x400FE000) // SYSCTL Base Address

#endif
