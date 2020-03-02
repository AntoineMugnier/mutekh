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

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/printk.h>
#include <mutek/startup.h>

#include <arch/efm32/leuart.h>
#include <arch/efm32/gpio.h>
#include <arch/efm32/cmu.h>
#include <arch/efm32/devaddr.h>

static inline void printk_out_char(char c)
{
  uint32_t b = CONFIG_MUTEK_PRINTK_ADDR;

  while (!(cpu_mem_read_32(b + EFM32_LEUART_STATUS_ADDR)
           & EFM32_LEUART_STATUS_TXBL))
    ;

  cpu_mem_write_32(b + EFM32_LEUART_TXDATA_ADDR, c);
}

static PRINTK_HANDLER(efm32_leuart_printk_out)
{
  size_t i;

  for (i = 0; i < len; i++)
  {
    if (str[i] == '\n')
      printk_out_char('\r');
    printk_out_char(str[i]);
  }
}

void efm32_leuart_printk_init()
{
  /* configure CMU */
  uint32_t cmu = EFM32_CMU_ADDR;

  uint32_t x = endian_le32(cpu_mem_read_32(cmu + EFM32_CMU_STATUS_ADDR));

  if ((x & EFM32_CMU_STATUS_LFXOENS) && 
      (x & EFM32_CMU_STATUS_LFXORDY))
    /* Select LFXO for CLKLFB */
    {
#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14) 
      x = endian_le32(EFM32_CMU_LFBCLKSEL_LFB(LFXO));
      cpu_mem_write_32(cmu + EFM32_CMU_LFBCLKSEL_ADDR, x);
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
      x = cpu_mem_read_32(cmu + EFM32_CMU_LFCLKSEL_ADDR);
      EFM32_CMU_LFCLKSEL_LFB_SET(x, LFXO);
      cpu_mem_write_32(cmu + EFM32_CMU_LFCLKSEL_ADDR, x);
#else
# error
#endif
    }
  else
    /* Select LFRCO for CLKLFB */
    {
      /* Enable LFRCO */
      cpu_mem_write_32(cmu + EFM32_CMU_OSCENCMD_ADDR, EFM32_CMU_OSCENCMD_LFRCOEN);
      /* Wait LFRCO for stabilizing */
      while (!(cpu_mem_read_32(cmu + EFM32_CMU_STATUS_ADDR) & EFM32_CMU_STATUS_LFRCORDY))
            ;
#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14) 
      x = endian_le32(EFM32_CMU_LFBCLKSEL_LFB(LFRCO));
      cpu_mem_write_32(cmu + EFM32_CMU_LFBCLKSEL_ADDR, x);
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
      x = cpu_mem_read_32(cmu + EFM32_CMU_LFCLKSEL_ADDR);
      EFM32_CMU_LFCLKSEL_LFB_SET(x, LFRCO);
      cpu_mem_write_32(cmu + EFM32_CMU_LFCLKSEL_ADDR, x);
#else
# error
#endif
    }

  uint32_t lfbclken;

#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14) 
  /* Enable clock for GPIO and LE interface */
  x = cpu_mem_read_32(cmu + EFM32_CMU_HFBUSCLKEN0_ADDR);
  x |= EFM32_CMU_HFBUSCLKEN0_GPIO | EFM32_CMU_HFBUSCLKEN0_LE;
  cpu_mem_write_32(cmu + EFM32_CMU_HFBUSCLKEN0_ADDR, endian_le32(x));

  if (CONFIG_MUTEK_PRINTK_ADDR != 0x4004a000)
    return;

  lfbclken = EFM32_CMU_LFBCLKEN0_LEUART0;

#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
  /* Enable HF peripherals clock */
  x = cpu_mem_read_32(cmu + EFM32_CMU_HFPERCLKDIV_ADDR);
  x |= EFM32_CMU_HFPERCLKDIV_HFPERCLKEN;
  cpu_mem_write_32(cmu + EFM32_CMU_HFPERCLKDIV_ADDR, endian_le32(x));
  /* Enable LE clock */
  x = cpu_mem_read_32(cmu + EFM32_CMU_HFCORECLKEN0_ADDR);
  x |= EFM32_CMU_HFCORECLKEN0_LE;
  cpu_mem_write_32(cmu + EFM32_CMU_HFCORECLKEN0_ADDR, endian_le32(x));
  /* Enable GPIO clock */
  x = cpu_mem_read_32(cmu + EFM32_CMU_HFPERCLKEN0_ADDR);
  x |= EFM32_CMU_HFPERCLKEN0_GPIO;
  cpu_mem_write_32(cmu + EFM32_CMU_HFPERCLKEN0_ADDR, endian_le32(x));

  switch (CONFIG_MUTEK_PRINTK_ADDR)
    {
  #ifdef EFM32_CMU_LFBCLKEN0_LEUART0
    case 0x40084000:            /* leuart0 */
      lfbclken = EFM32_CMU_LFBCLKEN0_LEUART0;
      break;
  #endif
  #ifdef EFM32_CMU_LFBCLKEN0_LEUART1
    case 0x40084400:            /* leuart1 */
      lfbclken = EFM32_CMU_LFBCLKEN0_LEUART1;
      break;
  #endif
    default:
      return;
    }
#else
# error
#endif

  /* Enable clock for LEUART0 */
  x = cpu_mem_read_32(cmu + EFM32_CMU_LFBCLKEN0_ADDR);
  x |= lfbclken;
  cpu_mem_write_32(cmu + EFM32_CMU_LFBCLKEN0_ADDR, x);

  /* configure GPIO to route LEUART signals */
  uint32_t gpio = EFM32_GPIO_ADDR;

  /* TX route */
  uint32_t bank = CONFIG_DRIVER_EFM32_LEUART_PRINTK_PIN / 16;
  uint32_t pin = CONFIG_DRIVER_EFM32_LEUART_PRINTK_PIN % 8;
  uint32_t h = (CONFIG_DRIVER_EFM32_LEUART_PRINTK_PIN >> 1) & 4;
  x = cpu_mem_read_32(gpio + EFM32_GPIO_MODEL_ADDR(bank) + h);
  EFM32_GPIO_MODEL_MODE_SET(pin, x, PUSHPULL);
  cpu_mem_write_32(gpio + EFM32_GPIO_MODEL_ADDR(bank) + h, x);

  uint32_t leuart = CONFIG_MUTEK_PRINTK_ADDR;

  /* Check that there is no on-going synchronization */
  while (cpu_mem_read_32(leuart + EFM32_LEUART_SYNCBUSY_ADDR)
         & EFM32_LEUART_SYNCBUSY_MASK)
    ;

  /* Freeze Registers */
  cpu_mem_write_32(leuart + EFM32_LEUART_FREEZE_ADDR, EFM32_LEUART_FREEZE_MASK);

  /* 8N1 */
  cpu_mem_write_32(leuart + EFM32_LEUART_CTRL_ADDR, 0);

  /* Baudrate */
  //#define LEUART_CLOCK (38400000/2/8)
#define LEUART_CLOCK CONFIG_DRIVER_EFM32_LEUART_CLOCK
#define RATE (256ULL * LEUART_CLOCK / CONFIG_DRIVER_EFM32_LEUART_RATE - 256)

#if RATE > EFM32_LEUART_CLKDIV_MASK || RATE < 0
# warning EFM32 leuart printk data rate out of range when used with a 32khz clock.
  /* However, the data rate may be fine if the leuart input clock is
     later changed by the clock management when the
     CONFIG_DRIVER_EFM32_LEUART_CHAR driver is enabled. */
  cpu_mem_write_32(leuart + EFM32_LEUART_CLKDIV_ADDR, EFM32_LEUART_CLKDIV_MASK);
#else
  cpu_mem_write_32(leuart + EFM32_LEUART_CLKDIV_ADDR, (uint32_t)RATE);
#endif

  /* LEUART routes */
#if (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG1) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG12) ||\
    (CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFR_XG14)
  cpu_mem_write_32(leuart + EFM32_LEUART_ROUTEPEN_ADDR, EFM32_LEUART_ROUTEPEN_TXPEN);
  x = cpu_mem_read_32(leuart + EFM32_LEUART_ROUTELOC0_ADDR);
  EFM32_LEUART_ROUTELOC0_TXLOC_SETVAL(x, CONFIG_DRIVER_EFM32_LEUART_PRINTK_LOC);
  cpu_mem_write_32(leuart + EFM32_LEUART_ROUTELOC0_ADDR, x);
#elif CONFIG_EFM32_ARCHREV == EFM32_ARCHREV_EFM
  x = EFM32_LEUART_ROUTE_TXPEN;
  EFM32_LEUART_ROUTE_LOCATION_SETVAL(x, CONFIG_DRIVER_EFM32_LEUART_PRINTK_LOC);
  cpu_mem_write_32(leuart + EFM32_LEUART_ROUTE_ADDR, x);
#else
# error
#endif

  /* Unfreeze Registers */
  cpu_mem_write_32(leuart + EFM32_LEUART_FREEZE_ADDR, 0);

  /* Enable TX */
  cpu_mem_write_32(leuart + EFM32_LEUART_CMD_ADDR, EFM32_LEUART_CMD_TXEN);

  static struct printk_backend_s backend;
  printk_register(&backend, efm32_leuart_printk_out);
}

