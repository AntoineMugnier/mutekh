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

    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014
    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2012
*/

#include <hexo/types.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>

#include <mutek/printk.h>

#include <cpp/device/helpers.h>
#include <arch/stm32_usart.h>
#include <arch/stm32_gpio.h>
#include <arch/stm32_rcc.h>
#include <arch/stm32_memory_map.h>


extern uint32_t stm32f4xx_clock_freq_apb1;

static inline void stm32_usart_tx_wait_ready()
{
  reg_t status;
  do
    {
      status = DEVICE_REG_VALUE(USART, 2, SR);
    }
  while ((status & STM32_USART_SR_TXE) == 0);
}

static PRINTF_OUTPUT_FUNC(early_console_out)
{
  uint_fast8_t i;

  for (i = 0; i < len; i++)
    {
      /* wait for the previous byte to be sent. */
      stm32_usart_tx_wait_ready();

      /* write the byte to the data register of the USART. */
      if (str[i] == '\n')
      {
        DEVICE_REG_UPDATE(USART, 2, DR, '\r');
        stm32_usart_tx_wait_ready();
      }
      DEVICE_REG_UPDATE(USART, 2, DR, str[i]);
    }
}

void stm32_usart_printk_init()
{
  reg_t cr1 = 0, cr2 = 0, cfg;

  /* enable clock on USART bus. */
  DEVICE_REG_FIELD_SET(RCC, , AHB1ENR, GPIOAEN);
  DEVICE_REG_FIELD_SET(RCC, , APB1ENR, USART2EN);

  /* configure PA2/PA3 as TX/RX. */
  cfg = DEVICE_REG_VALUE(GPIO, A, MODER);
  DEVICE_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 2, ALT, cfg);
  DEVICE_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 3, ALT, cfg);
  DEVICE_REG_UPDATE(GPIO, A, MODER, cfg);

  cfg = DEVICE_REG_VALUE(GPIO, A, AFRL);
  DEVICE_REG_FIELD_IDX_UPDATE_VAR(GPIO, AFRL, AF, 2, 7, cfg);
  DEVICE_REG_FIELD_IDX_UPDATE_VAR(GPIO, AFRL, AF, 3, 7, cfg);
  DEVICE_REG_UPDATE(GPIO, A, AFRL, cfg);

  /* wait for the last byte to be send just in case. */
  stm32_usart_tx_wait_ready();

  /* deactivate the USART and reset configuration. */
  DEVICE_REG_UPDATE(USART, 2, CR1, 0);
  DEVICE_REG_UPDATE(USART, 2, CR2, 0);
  DEVICE_REG_UPDATE(USART, 2, CR3, 0);

  /* configure baud rate tp 9600 Kbps. */
  DEVICE_REG_UPDATE(USART, 2, BRR,
    (int)(
      (float)stm32f4xx_clock_freq_apb1 /
        CONFIG_DRIVER_STM32_USART_PRINTK_BAUDRATE
      + 0.5
    )
  );

  /* oversampling x16. */
  DEVICE_REG_FIELD_UPDATE_VAR(USART, CR1, OVER8, 16, cr1);

  /* configure USART 8 bits, no parity, 1 stop bit. */
  DEVICE_REG_FIELD_UPDATE_VAR(USART, CR1, M, 8_BITS, cr1);
  DEVICE_REG_FIELD_UPDATE_VAR(USART, CR1, PCE, NONE, cr1);
  DEVICE_REG_FIELD_UPDATE_VAR(USART, CR2, STOP, 1_BIT, cr2);

  /* enable TX. */
  DEVICE_REG_FIELD_SET_VAR(USART, CR1, TE, cr1);

  /* propagate the configuration. */
  DEVICE_REG_UPDATE(USART, 2, CR1, cr1);
  DEVICE_REG_UPDATE(USART, 2, CR2, cr2);

  /* enable USART2. */
  DEVICE_REG_FIELD_SET_VAR(USART, CR1, UE, cr1);
  DEVICE_REG_UPDATE(USART, 2, CR1, cr1);

  printk_set_output(early_console_out, NULL);
}

