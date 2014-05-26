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
#include <hexo/cpu.h>
#include <hexo/iospace.h>
#include <hexo/endian.h>
#include <mutek/printk.h>

#ifdef CONFIG_STM32_EARLY_CONSOLE_UART

#define _STM32F4xx_USART2_ADDR(offset) \
    (STM32F4xx_USART2_ADDR + (offset)) \
/**/

#ifndef CONFIG_STM32_EARLY_CONSOLE_UART_ADDR
# error missing address of early console uart controller
#else
# define STM32F4xx_USART2_ADDR   CONFIG_STM32_EARLY_CONSOLE_UART_ADDR
#endif

#define STM32F4xx_USART2_SR     0x00
#define STM32F4xx_USART2_DR     0x04
#define STM32F4xx_USART2_BRR    0x08
#define STM32F4xx_USART2_CR1    0x0c
#define STM32F4xx_USART2_CR2    0x10
#define STM32F4xx_USART2_CR3    0x14
#define STM32F4xx_USART2_GTPR   0x18

extern uint32_t stm32f4xx_clock_freq_ahb1;
extern uint32_t stm32f4xx_clock_freq_apb1;
extern uint32_t stm32f4xx_clock_freq_apb2;

/* Define the baudrate of the USART according to the bus clock. */
#define STM32F4xx_USART2_BRR_VALUE(bps)                         \
    ( (int)(stm32f4xx_clock_freq_apb1 / (bps) + 0.5) & 0xffff ) \
/**/

/* Enable oversampling (x16). */
#define STM32F4xx_USART2_CR1_OVER_16        0
#define STM32F4xx_USART2_CR1_OVER_8         (1 << 15)

/* Disable/enable USART. */
#define STM32F4xx_USART2_CR1_ENABLE         (1 << 13)

/* Word length (0=8 bits, 1=9 bits). */
#define STM32F4xx_USART2_CR1_8_BITS         0
#define STM32F4xx_USART2_CR1_9_BITS         (1 << 12)

/* Parity (0=None, 1=Enabled). */
#define STM32F4xx_USART2_CR1_PARITY_NONE    0
#define STM32F4xx_USART2_CR1_PARITY_EN      (1 << 10)

/* TX enable. */
#define STM32F4xx_USART2_CR1_TXEN           (1 << 3)

/* RX enable. */
#define STM32F4xx_USART2_CR1_RXEN           (1 << 2)

/* Stop bits. */
#define STM32F4xx_USART2_CR2_STOP_1_BIT     0
#define STM32F4xx_USART2_CR2_STOP_05_BIT    (1 << 12)
#define STM32F4xx_USART2_CR2_STOP_2_BIT     (1 << 13)

/* Status register. */
#define STM32F4xx_USART2_SR_TX_DONE         (1 << 7)

static inline void stm32_usart_tx_wait_ready()
{
  reg_t status;
  do
    {
      status = cpu_mem_read_32(
        _STM32F4xx_USART2_ADDR(STM32F4xx_USART2_SR));
    }
  while ((status & STM32F4xx_USART2_SR_TX_DONE) == 0);
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
        cpu_mem_write_32(_STM32F4xx_USART2_ADDR(STM32F4xx_USART2_DR), '\r');
        stm32_usart_tx_wait_ready();
      }
      cpu_mem_write_32(_STM32F4xx_USART2_ADDR(STM32F4xx_USART2_DR), str[i]);
    }
}

void stm32_early_console_init()
{
  reg_t cr1 = 0, cr2 = 0, cfg;

#if !defined(CONFIG_STM32_EARLY_CONSOLE_UART)
# error
#endif

  /* enable clock on USART bus. */
  cpu_mem_write_32(0x40023830, (1 << 0)); /* enable AHB1 GPIO port A. */
  cpu_mem_write_32(0x40023840, (1 << 17)); /* enable APB1 USART2. */

  /* configure PA2/PA3 as TX/RX. */
  cfg = STM32F4xx_REG_VALUE(GPIO, A, MODER);
  STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 2, ALT, cfg);
  STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, MODER, MODE, 3, ALT, cfg);
  STM32F4xx_REG_UPDATE(GPIO, A, MODER, cfg);

  cfg = STM32F4xx_REG_VALUE(GPIO, A, AFRL);
  STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, AFRL, AF, 2, 7, cfg);
  STM32F4xx_REG_FIELD_IDX_UPDATE_VAR(GPIO, AFRL, AF, 3, 7, cfg);
  STM32F4xx_REG_UPDATE(GPIO, A, AFRL, cfg);

  /* wait for the last byte to be send just in case. */
  stm32_usart_tx_wait_ready();

  /* configure baud rate tp 9600 Kbps. */
  cpu_mem_write_32(
    _STM32F4xx_USART2_ADDR(STM32F4xx_USART2_BRR),
    STM32F4xx_USART2_BRR_VALUE(CONFIG_STM32_EARLY_CONSOLE_UART_BAUDRATE)
  );

  /* oversampling x16. */
  STM32F4xx_REG_FIELD_UPDATE_VAR(USART, CR1, OVER8, 16, cr1);

  /* configure USART 8 bits, no parity, 1 stop bit. */
  STM32F4xx_REG_FIELD_UPDATE_VAR(USART, CR1, M, 8_BITS, cr1);
  STM32F4xx_REG_FIELD_UPDATE_VAR(USART, CR1, PCE, NONE, cr1);
  STM32F4xx_REG_FIELD_UPDATE_VAR(USART, CR2, STOP, 1_BIT, cr2);

  /* enable TX. */
  STM32F4xx_REG_FIELD_SET_VAR(USART, CR1, TE, cr1);

  /* propagate the configuration. */
  cpu_mem_write_32(_STM32F4xx_USART2_ADDR(STM32F4xx_USART2_CR1), cr1);
  cpu_mem_write_32(_STM32F4xx_USART2_ADDR(STM32F4xx_USART2_CR2), cr2);

  /* enable USART2. */
  STM32F4xx_REG_FIELD_SET_VAR(USART, CR1, UE, cr1);
  STM32F4xx_REG_UPDATE(USART, 2, CR1, cr1);

  printk_set_output(early_console_out, NULL);
}

#endif

