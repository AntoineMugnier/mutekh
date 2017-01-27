/*
    This file is part of MutekH.
    
    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU Lesser General Public License as
    published by the Free Software Foundation; version 2.1 of the
    License.
    
    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.
    
    You should have received a copy of the GNU Lesser General Public
    License along with MutekH; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
    02110-1301 USA.

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013

*/

#include <mutek/startup.h>

#include <string.h>

#include <hexo/iospace.h>
#include <arch/efm32/gpio.h>
#include <arch/efm32/cmu.h>
#include <arch/efm32/devaddr.h>

#if defined(CONFIG_EFM32_BOOT_BUTTON)

void efm32_boot_button_wait()
{
  uint32_t b, x;

  b = EFM32_CMU_ADDR;

#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1

  cpu_mem_write_32(b + EFM32_CMU_OSCENCMD_ADDR, EFM32_CMU_OSCENCMD_HFRCOEN);
  while (!(cpu_mem_read_32(b + EFM32_CMU_STATUS_ADDR) & EFM32_CMU_STATUS_HFRCORDY))
    ;
  cpu_mem_write_32(b + EFM32_CMU_HFCLKSEL_ADDR, EFM32_CMU_HFCLKSEL_HF(HFRCO));
  /* Enable clock for GPIO */
  x = cpu_mem_read_32(b + EFM32_CMU_HFBUSCLKEN0_ADDR);
  cpu_mem_write_32(b + EFM32_CMU_HFBUSCLKEN0_ADDR, x | EFM32_CMU_HFBUSCLKEN0_GPIO);

#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM

  /* Enable clock for HF peripherals */
  x = cpu_mem_read_32(b + EFM32_CMU_HFPERCLKDIV_ADDR);
  x |= EFM32_CMU_HFPERCLKDIV_HFPERCLKEN;
  cpu_mem_write_32(b + EFM32_CMU_HFPERCLKDIV_ADDR, x);

  /* Enable clock for GPIO */
  x = cpu_mem_read_32(b + EFM32_CMU_HFPERCLKEN0_ADDR);
  x |= EFM32_CMU_HFPERCLKEN0_GPIO;
  cpu_mem_write_32(b + EFM32_CMU_HFPERCLKEN0_ADDR, x);

#else
# error
#endif

  b = EFM32_GPIO_ADDR;

  /* wait for button to be released */
  uint32_t bank = CONFIG_EFM32_BOOT_BUTTON_PIN / 16;
  uint32_t h = (CONFIG_EFM32_BOOT_BUTTON_PIN >> 1) & 4;

  x = cpu_mem_read_32(b + EFM32_GPIO_MODEL_ADDR(bank) + h);
  EFM32_GPIO_MODEL_MODE_SET(CONFIG_EFM32_BOOT_BUTTON_PIN % 8, x, INPUT);
  cpu_mem_write_32(b + EFM32_GPIO_MODEL_ADDR(bank) + h, x);

  while (!(cpu_mem_read_32(b + EFM32_GPIO_DIN_ADDR(bank))
           & EFM32_GPIO_DIN_DIN(CONFIG_EFM32_BOOT_BUTTON_PIN % 16)))
    ;
}

#endif

#ifndef CONFIG_DEVICE_CLOCK

/* clock management disabled, enable all clocks at startup */
void efm32_clock_enable()
{
  uint32_t b, x;

  b = EFM32_CMU_ADDR;
  
#if CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1
  cpu_mem_write_32(b + EFM32_CMU_OSCENCMD_ADDR, EFM32_CMU_OSCENCMD_HFRCOEN);
  while (!(cpu_mem_read_32(b + EFM32_CMU_STATUS_ADDR) & EFM32_CMU_STATUS_HFRCORDY))
    ;
  cpu_mem_write_32(b + EFM32_CMU_HFCLKSEL_ADDR, EFM32_CMU_HFCLKSEL_HF(HFRCO));

  x = cpu_mem_read_32(b + EFM32_CMU_CTRL_ADDR);
  x = EFM32_CMU_CTRL_HFPERCLKEN |
      EFM32_CMU_CTRL_HFRADIOCLKEN;
  cpu_mem_write_32(b + EFM32_CMU_CTRL_ADDR, x);
  
  cpu_mem_write_32(b + EFM32_CMU_HFBUSCLKEN0_ADDR, EFM32_CMU_HFBUSCLKEN0_MASK);
  cpu_mem_write_32(b + EFM32_CMU_LFBCLKSEL_ADDR, EFM32_CMU_LFBCLKSEL_LFB(LFRCO));
  cpu_mem_write_32(b + EFM32_CMU_LFACLKSEL_ADDR, EFM32_CMU_LFACLKSEL_LFA(LFRCO));

#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM

  x = cpu_mem_read_32(b + EFM32_CMU_HFPERCLKDIV_ADDR);
  x |= EFM32_CMU_HFPERCLKDIV_HFPERCLKEN;
  cpu_mem_write_32(b + EFM32_CMU_HFPERCLKDIV_ADDR, x);

  cpu_mem_write_32(b + EFM32_CMU_HFCORECLKEN0_ADDR, EFM32_CMU_HFCORECLKEN0_MASK);

#endif

  cpu_mem_write_32(b + EFM32_CMU_OSCENCMD_ADDR, EFM32_CMU_OSCENCMD_LFRCOEN);
  while (!(cpu_mem_read_32(b + EFM32_CMU_STATUS_ADDR) & EFM32_CMU_STATUS_LFRCORDY))
    ;

  cpu_mem_write_32(b + EFM32_CMU_HFPERCLKEN0_ADDR, EFM32_CMU_HFPERCLKEN0_MASK);
  cpu_mem_write_32(b + EFM32_CMU_LFACLKEN0_ADDR, EFM32_CMU_LFACLKEN0_MASK);
  cpu_mem_write_32(b + EFM32_CMU_LFBCLKEN0_ADDR, EFM32_CMU_LFBCLKEN0_MASK);
}

#endif

