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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2015
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/printk.h>
#include <mutek/startup.h>

#include <arch/efm32/usart.h>
#include <arch/efm32/gpio.h>
#include <arch/efm32/cmu.h>
#include <arch/efm32/devaddr.h>

#define USART_CLOCK            14000000

static inline void printk_out_char(char c)
{
  uint32_t b = CONFIG_MUTEK_PRINTK_ADDR;

  while (!(cpu_mem_read_32(b + EFM32_USART_STATUS_ADDR)
           & EFM32_USART_STATUS_TXBL))
    ;

  cpu_mem_write_32(b + EFM32_USART_TXDATA_ADDR, c);
}

static PRINTK_HANDLER(efm32_usart_printk_out)
{
  uint_fast8_t i;

  for (i = 0; i < len; i++)
  {
    if (str[i] == '\n')
      printk_out_char('\r');
    printk_out_char(str[i]);
  }
}

void efm32_usart_printk_init()
{
  uint32_t hfperclken;

  switch (CONFIG_MUTEK_PRINTK_ADDR)
    {
#ifdef EFM32_CMU_HFPERCLKEN0_UART0
    case 0x4000e000:            /* uart0 */
      hfperclken = EFM32_CMU_HFPERCLKEN0_UART0;
      break;
#endif
#ifdef EFM32_CMU_HFPERCLKEN0_UART1
    case 0x4000e400:            /* uart1 */
      hfperclken = EFM32_CMU_HFPERCLKEN0_UART1;
      break;
#endif
#ifdef EFM32_CMU_HFPERCLKEN0_USART0
    case 0x4000c000:            /* usart0 */
      hfperclken = EFM32_CMU_HFPERCLKEN0_USART0;
      break;
#endif
#ifdef EFM32_CMU_HFPERCLKEN0_USART1
    case 0x4000c400:            /* usart1 */
      hfperclken = EFM32_CMU_HFPERCLKEN0_USART1;
      break;
#endif
#ifdef EFM32_CMU_HFPERCLKEN0_USART2
    case 0x4000c800:            /* usart2 */
      hfperclken = EFM32_CMU_HFPERCLKEN0_USART2;
      break;
#endif
    default:
      return;
    }

  uint32_t b, x;

  /* configure CMU */
  b = EFM32_CMU_ADDR;

  /* Enable clock for HF peripherals */
  x = cpu_mem_read_32(b + EFM32_CMU_HFPERCLKDIV_ADDR);
  x |= EFM32_CMU_HFPERCLKDIV_HFPERCLKEN;
  cpu_mem_write_32(b + EFM32_CMU_HFPERCLKDIV_ADDR, x);

  /* Enable clock for GPIO */
  x = cpu_mem_read_32(b + EFM32_CMU_HFPERCLKEN0_ADDR);
  x |= EFM32_CMU_HFPERCLKEN0_GPIO | hfperclken;
  cpu_mem_write_32(b + EFM32_CMU_HFPERCLKEN0_ADDR, x);

  /* configure GPIO to route USART signals */
  b = EFM32_GPIO_ADDR;

  /* TX route */
  uint32_t bank = CONFIG_DRIVER_EFM32_USART_PRINTK_PIN / 16;
  uint32_t pin = CONFIG_DRIVER_EFM32_USART_PRINTK_PIN % 8;
  uint32_t h = (CONFIG_DRIVER_EFM32_USART_PRINTK_PIN >> 1) & 4;
  x = cpu_mem_read_32(b + EFM32_GPIO_MODEL_ADDR(bank) + h);
  EFM32_GPIO_MODEL_MODE_SET(pin, x, PUSHPULL);
  cpu_mem_write_32(b + EFM32_GPIO_MODEL_ADDR(bank) + h, x);

#if (defined(CONFIG_EFM32_LEOPARD_GECKO) ||             \
     defined(CONFIG_EFM32_GIANT_GECKO) ||             \
     defined(CONFIG_EFM32_WONDER_GECKO)) &&            \
  defined(CONFIG_EFM32_STK_BC_EN) &&              \
  CONFIG_MUTEK_PRINTK_ADDR == 0x4000e000 &&       \
  CONFIG_DRIVER_EFM32_USART_PRINTK_PIN == 64

  /* set EFM_BC_EN (PF7) high on stk3[678]00 */
  x = cpu_mem_read_32(b + EFM32_GPIO_MODEL_ADDR(5));
  EFM32_GPIO_MODEL_MODE_SET(7, x, PUSHPULL);
  cpu_mem_write_32(b + EFM32_GPIO_MODEL_ADDR(5), x);

  cpu_mem_write_32(b + EFM32_GPIO_DOUTSET_ADDR(5), EFM32_GPIO_DOUTSET_DOUTSET(7));

#elif defined(CONFIG_EFM32_GECKO) &&           \
  defined(CONFIG_EFM32_STK_BC_EN) &&              \
  CONFIG_MUTEK_PRINTK_ADDR == 0x4000e000 &&       \
  CONFIG_DRIVER_EFM32_USART_PRINTK_PIN == 64

  /* set EFM_BC_EN (PD13) high on g8xx_stk */
  x = cpu_mem_read_32(b + EFM32_GPIO_MODEH_ADDR(3));
  EFM32_GPIO_MODEH_MODE_SET(5, x, PUSHPULL);
  cpu_mem_write_32(b + EFM32_GPIO_MODEH_ADDR(3), x);

  cpu_mem_write_32(b + EFM32_GPIO_DOUTSET_ADDR(3), EFM32_GPIO_DOUTSET_DOUTSET(13));
#endif

  b = CONFIG_MUTEK_PRINTK_ADDR;

  cpu_mem_write_32(b + EFM32_USART_CMD_ADDR,
                   EFM32_USART_CMD_CLEARRX | EFM32_USART_CMD_CLEARTX);

  cpu_mem_write_32(b + EFM32_USART_CTRL_ADDR,
                   EFM32_USART_CTRL_SYNC(ASYNC) | EFM32_USART_CTRL_OVS(X4));

  cpu_mem_write_32(b + EFM32_USART_FRAME_ADDR,
                   EFM32_USART_FRAME_DATABITS(EIGHT) |
                   EFM32_USART_FRAME_PARITY(NONE) |
                   EFM32_USART_FRAME_STOPBITS(ONE));

  uint32_t div = (256ULL * USART_CLOCK) / (4 * CONFIG_DRIVER_EFM32_USART_RATE) - 256;
  cpu_mem_write_32(b + EFM32_USART_CLKDIV_ADDR, div);

  /* USART routes */
  x = EFM32_USART_ROUTE_TXPEN;
  EFM32_USART_ROUTE_LOCATION_SETVAL(x, CONFIG_DRIVER_EFM32_USART_PRINTK_LOC);
  cpu_mem_write_32(b + EFM32_USART_ROUTE_ADDR, x);

  /* Enable TX */
  cpu_mem_write_32(b + EFM32_USART_CMD_ADDR,
                   EFM32_USART_CMD_TXEN);

  static struct printk_backend_s backend;
  printk_register(&backend, efm32_usart_printk_out);
}

