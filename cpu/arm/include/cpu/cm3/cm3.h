//*****************************************************************************
//
// Cortex M3 System Controller Register Definitions
//
//*****************************************************************************

#ifndef __CM3_H__
#define __CM3_H__

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR System Peripherals
// *****************************************************************************

//*****************************************************************************
//
// Nested Vectored Interrupt Ctrl (NVIC)
//
//*****************************************************************************

typedef struct _CM3S_NVIC {
  uint32_t Reserved0;  
  uint32_t NVIC_ITCTL_TYPER;  // NVIC Interrupt Controller Type Register 
  uint32_t NVIC_ACTLR;        // NVIC Auxiliary Control Register 
  uint32_t Reserved1;
  uint32_t NVIC_ST_CTLR;      // NVIC SysTick Control and Status Register 
  uint32_t NVIC_ST_RLDR;      // NVIC SysTick Reload Value Register 
  uint32_t NVIC_ST_CURR;      // NVIC SysTick Current Value Register 
  uint32_t NVIC_ST_CALR;      // NVIC SysTick Calibration Value Register 
  uint32_t Reserved2[56];
  uint32_t NVIC_ITENR[8];     // NVIC Interrupt Set Enable Registers 
  uint32_t Reserved3[24];
  uint32_t NVIC_ITDISR[8];    // NVIC Interrupt Clear Enable Registers 
  uint32_t Reserved4[24];
  uint32_t NVIC_ITPENDR[8];   // NVIC Interrupt Set Pending Registers 
  uint32_t Reserved5[24];
  uint32_t NVIC_ITCLRPR[8];   // NVIC Interrupt Clear Pending Registers 
  uint32_t Reserved6[24];
  uint32_t NVIC_ITACTIVER[8]; // NVIC Interrupt Active Bit Registers 
  uint32_t Reserved7[56];
  uint32_t NVIC_ITPRIR[60];   // NVIC Interrupt Priority Registers 
  uint32_t Reserved8[516];
  uint32_t NVIC_CPUIDR;       // NVIC CPUID Base Register 
  uint32_t NVIC_ITCTLR;       // NVIC Interrupt Control State Register 
  uint32_t NVIC_VTOR;         // NVIC Vector Table Offset Register 
  uint32_t NVIC_AIRCR;        // NVIC Application Interrupt and Reset Control Register 
  uint32_t NVIC_SYSCTLR;      // NVIC System Control Register 
  uint32_t NVIC_CFGCTLR;      // NVIC Config Control Register 
  uint32_t NVIC_SYS_PRIR[3];  // NVIC System Handler Priority Registers 
  uint32_t NVIC_SHND_CTLR;    // NVIC System Handler Control and State Register 
  uint32_t NVIC_FLT_STR;      // NVIC Configurable Fault Status Register 
  uint32_t NVIC_HFLT_STR;     // NVIC Hard Fault Status Register 
  uint32_t NVIC_DBG_STR;      // NVIC Debug Fault Status Register 
  uint32_t NVIC_MM_ADR;       // NVIC Memory Manage Fault Address Register 
  uint32_t NVIC_FLT_ADR;      // NVIC Bus Fault Address Register 
  uint32_t NVIC_AFLT_ADR;     // NVIC Auxiliary Fault Address Register (impdep)
  uint32_t Reserved9[20];
  uint32_t MPU_TYPER;         // MPU Type Register
  uint32_t MPU_CTLR;          // MPU Control Register
  uint32_t MPU_REG_NBRR;      // MPU Region Number Register
  uint32_t MPU_REG_BASE_ADR;  // MPU Region Base Address Register
  uint32_t MPU_REG_ATTRR;     // MPU Region Attribute and Size Register
  uint32_t DGB_CTLR;          // Debug Control Register
  uint32_t DBG_XFERR;         //
  uint32_t DBG_DATAR;         //
  uint32_t DGB_INTR;          //
  uint32_t Reserved10[64];
  uint32_t NVIC_SW_TRIR;      // NVIC Sowftawre Triggerable Interrupt
} CM3S_NVIC, *CM3PS_NVIC;


// *****************************************************************************
//               BASE ADDRESS DEFINITIONS
// *****************************************************************************
#define CM3_BASE_NVIC       ((CM3PS_NVIC) 	0xE000E000) // NVIC Base Address

#endif
