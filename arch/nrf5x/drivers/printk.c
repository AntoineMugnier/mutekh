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
    License along with this program.  If not, see
    <http://www.gnu.org/licenses/>.

    Copyright
        Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>
#include <mutek/startup.h>

#include <mutek/printk.h>
#include <arch/nrf5x/uart.h>
#include <arch/nrf5x/gpio.h>

#include "printk.h"

static inline void nrf5x_printk_out_char(uintptr_t addr, char c)
{
  uint32_t timeout = 0x100;

  nrf_event_clear(addr, NRF_UART_TXDRDY);

  nrf_reg_set(addr, NRF_UART_TXD, c);

  while (!nrf_event_check(addr, NRF_UART_TXDRDY) && --timeout)
    ;
}

void nrf5x_printk_out_nodrv(uintptr_t addr, const char *str, size_t len)
{
  size_t i;

  nrf_task_trigger(addr, NRF_UART_STARTTX);
  nrf_event_clear(addr, NRF_UART_TXDRDY);

  for (i = 0; i < len; i++)
    {
      if (str[i] == '\n')
        nrf5x_printk_out_char(addr, '\r');
      nrf5x_printk_out_char(addr, str[i]);
    }

  nrf_task_trigger(addr, NRF_UART_STOPTX);
}

static PRINTK_HANDLER(nrf5x_printk)
{
  nrf5x_printk_out_nodrv(CONFIG_MUTEK_PRINTK_ADDR, str, len);
}

void nrf5x_printk_init()
{
  const uintptr_t gpio = NRF5X_GPIO_ADDR;

  nrf_reg_set(CONFIG_MUTEK_PRINTK_ADDR, NRF_UART_PSELTXD,
              CONFIG_DRIVER_NRF5X_PRINTK_PIN);

  nrf_reg_set(gpio,
              NRF_GPIO_PIN_CNF(CONFIG_DRIVER_NRF5X_PRINTK_PIN),
              NRF_GPIO_PIN_CNF_DIR_OUTPUT
              | NRF_GPIO_PIN_CNF_INPUT_DISCONNECT
              | NRF_GPIO_PIN_CNF_DRIVE_S0S1);

  nrf_reg_set(CONFIG_MUTEK_PRINTK_ADDR, NRF_UART_BAUDRATE,
              NRF_UART_BAUDRATE_(CONFIG_DRIVER_NRF5X_PRINTK_RATE));
  nrf_reg_set(CONFIG_MUTEK_PRINTK_ADDR,
              NRF_UART_CONFIG,
              NRF_UART_CONFIG_PARITY_DISABLED
              | NRF_UART_CONFIG_CTSRTS_DISABLED);
  nrf_reg_set(CONFIG_MUTEK_PRINTK_ADDR,
              NRF_UART_ENABLE,
              NRF_UART_ENABLE_ENABLED);

  struct printk_backend_s backend;
  printk_register(&backend, nrf5x_printk);
}
