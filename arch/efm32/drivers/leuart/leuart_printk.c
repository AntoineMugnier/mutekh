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

#include <arch/efm32/leuart.h>
#include <arch/efm32/gpio.h>
#include <arch/efm32/cmu.h>
#include <arch/efm32/devaddr.h>

#define LEUART_CLOCK            32768
#define LEUART_RATE             9600

static void early_console_out_char(char c)
{
  uint32_t b = CONFIG_MUTEK_PRINTK_ADDR;

  while (!(cpu_mem_read_32(b + EFM32_LEUART_STATUS_ADDR)
           & EFM32_LEUART_STATUS_TXBL))
    ;

  cpu_mem_write_32(b + EFM32_LEUART_TXDATA_ADDR, c);
}

static PRINTF_OUTPUT_FUNC(early_console_out)
{
  uint_fast8_t i;

  for (i = 0; i < len; i++)
  {
    if (str[i] == '\n')
      early_console_out_char('\r');
    early_console_out_char(str[i]);
  }
}

void efm32_leuart_printk_init()
{
  uint32_t lfbclken;

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

  uint32_t b, x;

  /* configure CMU */
  b = EFM32_CMU_ADDR;

  /* Enable clock for LE interface */
  cpu_mem_write_32(b + EFM32_CMU_HFCORECLKEN0_ADDR, EFM32_CMU_HFCORECLKEN0_LE);

  /* Enable LFRCO */
  cpu_mem_write_32(b + EFM32_CMU_OSCENCMD_ADDR, EFM32_CMU_OSCENCMD_LFRCOEN);

  /* Wait LFRCO for stabilizing */
  while (!(cpu_mem_read_32(b + EFM32_CMU_STATUS_ADDR) & EFM32_CMU_STATUS_LFRCORDY))
        ;

  /* Select LFRCO for CLKLFB */
  x = cpu_mem_read_32(b + EFM32_CMU_LFCLKSEL_ADDR);
  EFM32_CMU_LFCLKSEL_LFB_SET(x, LFRCO);
  cpu_mem_write_32(b + EFM32_CMU_LFCLKSEL_ADDR, x);

  /* Enable clock for LEUART0 */
  x = cpu_mem_read_32(b + EFM32_CMU_LFBCLKEN0_ADDR);
  x |= lfbclken;
  cpu_mem_write_32(b + EFM32_CMU_LFBCLKEN0_ADDR, x);

  /* Enable clock for HF peripherals */
  x = cpu_mem_read_32(b + EFM32_CMU_HFPERCLKDIV_ADDR);
  x |= EFM32_CMU_HFPERCLKDIV_HFPERCLKEN;
  cpu_mem_write_32(b + EFM32_CMU_HFPERCLKDIV_ADDR, x);

  /* Enable clock for GPIO */
  x = cpu_mem_read_32(b + EFM32_CMU_HFPERCLKEN0_ADDR);
  x |= EFM32_CMU_HFPERCLKEN0_GPIO;
  cpu_mem_write_32(b + EFM32_CMU_HFPERCLKEN0_ADDR, x);

  /* configure GPIO to route LEUART signals */
  b = EFM32_GPIO_ADDR;

  /* TX route */
  uint32_t bank = CONFIG_DRIVER_EFM32_LEUART_PRINTK_PIN / 16;
  uint32_t pin = CONFIG_DRIVER_EFM32_LEUART_PRINTK_PIN % 8;
  uint32_t h = (CONFIG_DRIVER_EFM32_LEUART_PRINTK_PIN >> 1) & 4;
  x = cpu_mem_read_32(b + EFM32_GPIO_MODEL_ADDR(bank) + h);
  EFM32_GPIO_MODEL_MODE_SET(pin, x, PUSHPULL);
  cpu_mem_write_32(b + EFM32_GPIO_MODEL_ADDR(bank) + h, x);

#if defined(CONFIG_EFM32_STK3200) && \
  CONFIG_MUTEK_PRINTK_ADDR == 0x40084000 && \
  CONFIG_DRIVER_EFM32_LEUART_PRINTK_PIN == 52 && \
  defined(CONFIG_EFM32_STK_BC_EN)

  /* set EFM_BC_EN (PA9) high on stk3200 */
  x = cpu_mem_read_32(b + EFM32_GPIO_MODEH_ADDR(0));
  EFM32_GPIO_MODEH_MODE_SET(1, x, PUSHPULL);
  cpu_mem_write_32(b + EFM32_GPIO_MODEH_ADDR(0), x);

  cpu_mem_write_32(b + EFM32_GPIO_DOUTSET_ADDR(0), EFM32_GPIO_DOUTSET_DOUTSET(9));
#endif

  b = CONFIG_MUTEK_PRINTK_ADDR;

  /* Check that there is no on-going synchronization */
  while (cpu_mem_read_32(b + EFM32_LEUART_SYNCBUSY_ADDR)
         & EFM32_LEUART_SYNCBUSY_MASK)
    ;

  /* Freeze Registers */
  x = cpu_mem_read_32(b + EFM32_LEUART_FREEZE_ADDR);
  EFM32_LEUART_FREEZE_REGFREEZE_SET(x, FREEZE);
  cpu_mem_write_32(b + EFM32_LEUART_FREEZE_ADDR, x);

  /* 8N1 */
  x = cpu_mem_read_32(b + EFM32_LEUART_CTRL_ADDR);
  EFM32_LEUART_CTRL_DATABITS_SET(x, EIGHT);
  EFM32_LEUART_CTRL_PARITY_SET(x, NONE);
  EFM32_LEUART_CTRL_STOPBITS_SET(x, ONE);
  cpu_mem_write_32(b + EFM32_LEUART_CTRL_ADDR, x);

  /* Baudrate */
  x = cpu_mem_read_32(b + EFM32_LEUART_CLKDIV_ADDR);
  EFM32_LEUART_CLKDIV_DIV_SET(x, 32 * LEUART_CLOCK / LEUART_RATE - 32);
  cpu_mem_write_32(b + EFM32_LEUART_CLKDIV_ADDR, x);

  /* LEUART routes */
  x = EFM32_LEUART_ROUTE_TXPEN;
  EFM32_LEUART_ROUTE_LOCATION_SETVAL(x, CONFIG_DRIVER_EFM32_LEUART_PRINTK_LOC);
  cpu_mem_write_32(b + EFM32_LEUART_ROUTE_ADDR, x);

  /* Unfreeze Registers */
  x = cpu_mem_read_32(b + EFM32_LEUART_FREEZE_ADDR);
  EFM32_LEUART_FREEZE_REGFREEZE_SET(x, UPDATE);
  cpu_mem_write_32(b + EFM32_LEUART_FREEZE_ADDR, x);

  /* Enable TX */
  cpu_mem_write_32(b + EFM32_LEUART_CMD_ADDR, EFM32_LEUART_CMD_TXEN);

  printk_set_output(early_console_out, NULL);
}

