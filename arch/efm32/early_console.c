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
#include <hexo/cpu.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>
#include <mutek/printk.h>

#include <arch/efm32_gpio.h>
#include <arch/efm32_leuart.h>
#include <arch/efm32_cmu.h>

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

void efm32_early_console_leuart_init()
{
  uint32_t b;
  uint32_t x;

#if defined(CONFIG_EFM32_STK3200) \
 || defined(CONFIG_EFM32_STK3600) \
 || defined(CONFIG_EFM32_STK3700) \
 || defined(CONFIG_EFM32_STK3800) \
 || defined(CONFIG_EFM32_G8XX_STK)

  /* configure CMU for LEUART */
  b = 0x400c8000;

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
  x |= EFM32_CMU_LFBCLKEN0_LEUART0;
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
  b = 0x40006000;

  x = cpu_mem_read_32(b + EFM32_GPIO_MODEL_ADDR(3));

# if 0
  /* RX routed on D.5 */ 
  EFM32_GPIO_MODEL_MODE_SET(5, x, INPUT);
# endif

  /* TX routed on D.4 */ 
  EFM32_GPIO_MODEL_MODE_SET(4, x, PUSHPULL);
  cpu_mem_write_32(b + EFM32_GPIO_MODEL_ADDR(3), x);

# if defined(CONFIG_EFM32_STK3200)
  /* wait for user button 0 (PC8) to be released */
  x = cpu_mem_read_32(b + EFM32_GPIO_MODEH_ADDR(2));
  EFM32_GPIO_MODEH_MODE_SET(0, x, INPUT);
  cpu_mem_write_32(b + EFM32_GPIO_MODEH_ADDR(2), x);

  while (!(cpu_mem_read_32(b + EFM32_GPIO_DIN_ADDR(2)) & EFM32_GPIO_DIN_DIN(8)))
    ;

# elif defined(CONFIG_EFM32_STK3600) \
  || defined(CONFIG_EFM32_STK3700)  \
  || defined(CONFIG_EFM32_STK3800)  \
  || defined(CONFIG_EFM32_G8XX_STK)

  /* wait for user button 0 (PB9) to be released */
  x = cpu_mem_read_32(b + EFM32_GPIO_MODEH_ADDR(1));
  EFM32_GPIO_MODEH_MODE_SET(1, x, INPUT);
  cpu_mem_write_32(b + EFM32_GPIO_MODEH_ADDR(1), x);

  while (!(cpu_mem_read_32(b + EFM32_GPIO_DIN_ADDR(1)) & EFM32_GPIO_DIN_DIN(9)))
    ;
# endif

  b = CONFIG_MUTEK_PRINTK_ADDR;

  /* Check that there is no on-going synchronization */
  while (cpu_mem_read_32(b + EFM32_LEUART_SYNCBUSY_ADDR)
         & EFM32_LEUART_SYNCBUSY_MASK)
    ;

  /* Freeze Registers */
  x = cpu_mem_read_32(b + EFM32_LEUART_FREEZE_ADDR);
  EFM32_LEUART_FREEZE_REGFREEZE_SET(x, FREEZE);
  cpu_mem_write_32(b + EFM32_LEUART_FREEZE_ADDR, x);

  /* 8 data bits , 2 stop bits , no parity */
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
  x = cpu_mem_read_32(b + EFM32_LEUART_ROUTE_ADDR);
  x = EFM32_LEUART_ROUTE_RXPEN | EFM32_LEUART_ROUTE_TXPEN;
  EFM32_LEUART_ROUTE_LOCATION_SET(x, LOC0);
  cpu_mem_write_32(b + EFM32_LEUART_ROUTE_ADDR, x);
  
  /* Unfreeze Registers */
  x = cpu_mem_read_32(b + EFM32_LEUART_FREEZE_ADDR);
  EFM32_LEUART_FREEZE_REGFREEZE_SET(x, UPDATE);
  cpu_mem_write_32(b + EFM32_LEUART_FREEZE_ADDR, x);

  /* Enable TX and RX */
  cpu_mem_write_32(b + EFM32_LEUART_CMD_ADDR, EFM32_LEUART_CMD_TXEN |
                   EFM32_LEUART_CMD_RXEN);

  printk_set_output(early_console_out, NULL);

#else
# error No support for early console on this efm32 board
#endif
}

