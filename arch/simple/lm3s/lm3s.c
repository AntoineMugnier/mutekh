#include <hexo/local.h>
#include <hexo/types.h>
#include <hexo/interrupt.h>

#include <arch/lm3s/lm3s8962.h>
#include <cpu/cm3/cm3.h>

void arch_specific_init()
{
  size_t i;

  // Enable System Fault Handlers
  CM3_BASE_NVIC->NVIC_SHND_CTLR = 0x00070000;

  // Enable clock for used peripherals
  // --> Port A enable
  LM3S_BASE_SYSCTL->SYSCTL_RCGCR[2] = 0x00000001 |
    LM3S_BASE_SYSCTL->SYSCTL_RCGCR[2];
  // --> Port F enable
  LM3S_BASE_SYSCTL->SYSCTL_RCGCR[2] = 0x00000020 |
    LM3S_BASE_SYSCTL->SYSCTL_RCGCR[2];
  // --> UART0 enable
  LM3S_BASE_SYSCTL->SYSCTL_RCGCR[1] = 0x00000001 |
    LM3S_BASE_SYSCTL->SYSCTL_RCGCR[1];

  // Delay a little
  for(i=0; i<10; i++)
    asm volatile("nop");

  // Configure GPIOF => LED on bit 0
  // --> PF0 is an output
  LM3S_BASE_GPIOF->GPIO_DIRR = 0x00000001 | LM3S_BASE_GPIOF->GPIO_DIRR;
  // --> PF0 is a digital pin
  LM3S_BASE_GPIOF->GPIO_DENR = 0x00000001 | LM3S_BASE_GPIOF->GPIO_DENR;
  // --> PF1 is an input
  LM3S_BASE_GPIOF->GPIO_DIRR = ~0x00000002 & LM3S_BASE_GPIOF->GPIO_DIRR;
  // --> PF1 is a digital pin
  LM3S_BASE_GPIOF->GPIO_DENR = 0x00000002 | LM3S_BASE_GPIOF->GPIO_DENR;
  // --> PF1 has a pull-up
  LM3S_BASE_GPIOF->GPIO_PURR = 0x00000002 | LM3S_BASE_GPIOF->GPIO_PURR;
  // --> PF1 generate interrupt
  LM3S_BASE_GPIOF->GPIO_IMR = 0x00000002 | LM3S_BASE_GPIOF->GPIO_IMR;
  // --> PA0 and PA1 are digital pins
  LM3S_BASE_GPIOA->GPIO_DENR = 0x00000003 | LM3S_BASE_GPIOA->GPIO_DENR;
  // --> PA0 and PA1 are controlled by uart0 module
  LM3S_BASE_GPIOA->GPIO_AFSELR = 0x00000003 | LM3S_BASE_GPIOA->GPIO_AFSELR; // Set afsel

}
