
#include <pthread.h>
#include <mutek/printk.h>

#include <arch/stm32f4xx_regs.h>

void main()
{
  if (cpu_isbootstrap())
  {
    printk("value if I2C1 SR1 register = %lx\n",
           STM32F4xx_REG_VALUE(I2C, 1, SR1));
  }

  pthread_yield();
}

