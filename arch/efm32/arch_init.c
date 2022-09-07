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
#include <hexo/endian.h>
#include <arch/efm32/gpio.h>
#include <arch/efm32/cmu.h>
#include <arch/efm32/devaddr.h>
#include <arch/efm32/pin.h>

#ifndef CONFIG_DEVICE_CLOCK

/* clock management disabled, enable all clocks at startup */
void efm32_clock_enable()
{
  uint32_t b, x;

  b = EFM32_CMU_ADDR;

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1

  x = cpu_mem_read_32(b + EFM32_CMU_CTRL_ADDR);
  x |= EFM32_CMU_CTRL_HFPERCLKEN;
  cpu_mem_write_32(b + EFM32_CMU_CTRL_ADDR, x);
  
  cpu_mem_write_32(b + EFM32_CMU_HFBUSCLKEN0_ADDR, EFM32_CMU_HFBUSCLKEN0_MASK);

#elif EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0

  x = cpu_mem_read_32(b + EFM32_CMU_HFPERCLKDIV_ADDR);
  x |= EFM32_CMU_HFPERCLKDIV_HFPERCLKEN;
  cpu_mem_write_32(b + EFM32_CMU_HFPERCLKDIV_ADDR, x);

  cpu_mem_write_32(b + EFM32_CMU_HFCORECLKEN0_ADDR, EFM32_CMU_HFCORECLKEN0_MASK);

#endif

  cpu_mem_write_32(b + EFM32_CMU_HFPERCLKEN0_ADDR, EFM32_CMU_HFPERCLKEN0_MASK);
  cpu_mem_write_32(b + EFM32_CMU_LFACLKEN0_ADDR, EFM32_CMU_LFACLKEN0_MASK);
  cpu_mem_write_32(b + EFM32_CMU_LFBCLKEN0_ADDR, EFM32_CMU_LFBCLKEN0_MASK);

#if EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 1
  cpu_mem_write_32(b + EFM32_CMU_LFECLKEN0_ADDR, EFM32_CMU_LFECLKEN0_MASK);

  cpu_mem_write_32(b + EFM32_CMU_LFACLKSEL_ADDR, EFM32_CMU_LFACLKSEL_LFA(LFRCO));
  cpu_mem_write_32(b + EFM32_CMU_LFBCLKSEL_ADDR, EFM32_CMU_LFBCLKSEL_LFB(LFRCO));
  cpu_mem_write_32(b + EFM32_CMU_LFECLKSEL_ADDR, EFM32_CMU_LFECLKSEL_LFE(LFRCO));
#endif

  /* Enable LFRCO */
  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_OSCENCMD_ADDR, EFM32_CMU_OSCENCMD_LFRCOEN);
  while (!(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_STATUS_ADDR) & EFM32_CMU_STATUS_LFRCORDY))
    ;

}
#endif

#if defined(CONFIG_CPU_ARM32M_TRACE)
static void gpio_out_en(uint8_t port)
{
  uint32_t x, b = EFM32_GPIO_ADDR;

  /* TX route */
  uint32_t bank = port / 16;
  uint32_t pin = port % 8;
  uint32_t h = (port >> 1) & 4;

  x = cpu_mem_read_32(b + EFM32_GPIO_MODEL_ADDR(bank) + h);
  EFM32_GPIO_MODEL_MODE_SET(pin, x, PUSHPULL);
  cpu_mem_write_32(b + EFM32_GPIO_MODEL_ADDR(bank) + h, x);
}

void efm32_trace_init(void)
{
# if (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32LG) \
  || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32WG) \
  || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32GG) \
  || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32ZG)
  uint32_t x, b = EFM32_CMU_ADDR;

  cpu_mem_write_32(b + EFM32_CMU_LOCK_ADDR, EFM32_CMU_LOCK_LOCKKEY_UNLOCK);

  x = cpu_mem_read_32(b + EFM32_CMU_HFPERCLKDIV_ADDR);
  x |= EFM32_CMU_HFPERCLKDIV_HFPERCLKEN;
  cpu_mem_write_32(b + EFM32_CMU_HFPERCLKDIV_ADDR, x);

  x = cpu_mem_read_32(b + EFM32_CMU_HFPERCLKEN0_ADDR);
  x |= EFM32_CMU_HFPERCLKEN0_GPIO;
  cpu_mem_write_32(b + EFM32_CMU_HFPERCLKEN0_ADDR, x);

  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_OSCENCMD_ADDR, EFM32_CMU_OSCENCMD_AUXHFRCODIS);

  uint32_t band = 0;
  switch (CONFIG_EFM32_TRACE_AUXHFCO_FREQ)
    {
    case 1000000:
      band = EFM32_CMU_AUXHFRCOCTRL_BAND_1MHZ;
      break;
    case 7000000:
      band = EFM32_CMU_AUXHFRCOCTRL_BAND_7MHZ;
      break;
    case 11000000:
      band = EFM32_CMU_AUXHFRCOCTRL_BAND_11MHZ;
      break;
    case 14000000:
      band = EFM32_CMU_AUXHFRCOCTRL_BAND_14MHZ;
      break;
    case 21000000:
    default:
      band = EFM32_CMU_AUXHFRCOCTRL_BAND_21MHZ;
      break;
#  if (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32LG)      \
   || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32WG)       \
   || (EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32GG)
    case 28000000:
      band = EFM32_CMU_AUXHFRCOCTRL_BAND_28MHZ;
      break;
#  endif
    }

  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_AUXHFRCOCTRL_ADDR,
                   (band << EFM32_CMU_AUXHFRCOCTRL_BAND_SHIFT) |
                   cpu_mem_read_8(/* device information page */ 0xfe081d4 + (band ^ 3)));

  cpu_mem_write_32(EFM32_CMU_ADDR + EFM32_CMU_OSCENCMD_ADDR, EFM32_CMU_OSCENCMD_AUXHFRCOEN);
  while (!(cpu_mem_read_32(EFM32_CMU_ADDR + EFM32_CMU_STATUS_ADDR) & EFM32_CMU_STATUS_AUXHFRCORDY))
    ;

  uint32_t gpio_route = endian_le32(cpu_mem_read_32(EFM32_GPIO_ADDR + EFM32_GPIO_ROUTE_ADDR));

#  if CONFIG_CPU_ARM32M_TRACE_PARALLEL == 0
  gpio_route |= EFM32_GPIO_ROUTE_SWOPEN;
  gpio_out_en(EFM32_PF2);
#  elif CONFIG_CPU_ARM32M_TRACE_PARALLEL == 1
  gpio_route |= EFM32_GPIO_ROUTE_TD0PEN;
  gpio_route |= EFM32_GPIO_ROUTE_TCLKPEN;
  gpio_out_en(EFM32_PD7);
  gpio_out_en(EFM32_PD6);
#  elif CONFIG_CPU_ARM32M_TRACE_PARALLEL == 2
  gpio_route |= EFM32_GPIO_ROUTE_TD0PEN;
  gpio_route |= EFM32_GPIO_ROUTE_TD1PEN;
  gpio_route |= EFM32_GPIO_ROUTE_TCLKPEN;
  gpio_out_en(EFM32_PD7);
  gpio_out_en(EFM32_PD6);
  gpio_out_en(EFM32_PD3);
#  elif CONFIG_CPU_ARM32M_TRACE_PARALLEL == 4
  gpio_route |= EFM32_GPIO_ROUTE_TD0PEN;
  gpio_route |= EFM32_GPIO_ROUTE_TD1PEN;
  gpio_route |= EFM32_GPIO_ROUTE_TD2PEN;
  gpio_route |= EFM32_GPIO_ROUTE_TD3PEN;
  gpio_route |= EFM32_GPIO_ROUTE_TCLKPEN;
  gpio_out_en(EFM32_PD7);
  gpio_out_en(EFM32_PD6);
  gpio_out_en(EFM32_PD3);
  gpio_out_en(EFM32_PD4);
  gpio_out_en(EFM32_PD5);
#  else
#   error Unsupported trace configuration
#  endif

  cpu_mem_write_32(EFM32_GPIO_ADDR + EFM32_GPIO_ROUTE_ADDR, endian_le32(gpio_route));
# else
#  error Unsupported chip
# endif
}
#endif
