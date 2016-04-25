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
#include <mutek/startup.h>

#include <arch/stm32/f1/usart.h>

#include <arch/stm32/f1/mmap.h>
#include <arch/stm32/f1/gpio.h>
#include <arch/stm32/f1/rcc.h>
#include <arch/stm32/f1/remap.h>


#define STM32_GPIO_BANK_WIDTH 16

void stm32_usart_printk_init(void);

static inline void stm32_usart_tx_wait_ready()
{
  uintptr_t const a = CONFIG_MUTEK_PRINTK_ADDR + STM32_USART_SR_ADDR;

  reg_t status;
  do
    {
      status = endian_le32(cpu_mem_read_32(a));
    }
  while (!(status & STM32_USART_SR_TXE));
}

static PRINTF_OUTPUT_FUNC(printk_out)
{
  uintptr_t const a = CONFIG_MUTEK_PRINTK_ADDR + STM32_USART_DR_ADDR;

  uint_fast8_t i;

  for (i = 0; i < len; i++)
    {
      /* wait for the previous byte to be sent. */
      stm32_usart_tx_wait_ready();

      /* write the byte to the data register of the USART. */
      if (str[i] == '\n')
      {
        cpu_mem_write_32(a, endian_le32('\r'));
        stm32_usart_tx_wait_ready();
      }
      cpu_mem_write_32(a, endian_le32(str[i]));
    }
}

void stm32_usart_printk_init(void)
{
  uint32_t cr1 = 0, cr2 = 0, x;

  /* set gpio for TX */
  uint32_t a = (CONFIG_DRIVER_STM32_USART_PRINTK_PIN & (STM32_GPIO_BANK_WIDTH/2)
     ? STM32_GPIO_CRH_ADDR(CONFIG_DRIVER_STM32_USART_PRINTK_PIN / STM32_GPIO_BANK_WIDTH)
     : STM32_GPIO_CRL_ADDR(CONFIG_DRIVER_STM32_USART_PRINTK_PIN / STM32_GPIO_BANK_WIDTH));

  x = endian_le32(cpu_mem_read_32(STM32_GPIO_ADDR + a));
  STM32_GPIO_CRL_MODE_SET(
    CONFIG_DRIVER_STM32_USART_PRINTK_PIN % (STM32_GPIO_BANK_WIDTH/2), x, OUTPUT_50MHZ);
  STM32_GPIO_CRL_CNF_SET(
    CONFIG_DRIVER_STM32_USART_PRINTK_PIN % (STM32_GPIO_BANK_WIDTH/2), x, ALT_PUSH_PULL);
  cpu_mem_write_32(STM32_GPIO_ADDR + a, endian_le32(x));

  /* set gpio alternate function. */
  x = endian_le32(cpu_mem_read_32(STM32_AFIO_ADDR + STM32_AFIO_MAPR_ADDR));
  switch (CONFIG_MUTEK_PRINTK_ADDR)
    {
    case STM32_USART1_ADDR:
      STM32_AFIO_MAPR_USART1_REMAP_SET(x, CONFIG_DRIVER_STM32_USART_PRINTK_AF);
      break;
    case STM32_USART2_ADDR:
      STM32_AFIO_MAPR_USART2_REMAP_SET(x, CONFIG_DRIVER_STM32_USART_PRINTK_AF);
      break;
    case STM32_USART3_ADDR:
      STM32_AFIO_MAPR_USART3_REMAP_SET(x, CONFIG_DRIVER_STM32_USART_PRINTK_AF);
      break;
    }
  cpu_mem_write_32(STM32_AFIO_ADDR + STM32_AFIO_MAPR_ADDR, endian_le32(x));

  /* wait for the last byte to be send just in case. */
  //stm32_usart_tx_wait_ready();

  /* deactivate the USART and reset configuration. */
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + STM32_USART_CR1_ADDR, 0);
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + STM32_USART_CR2_ADDR, 0);
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + STM32_USART_CR3_ADDR, 0);

  /* enable TX. */
  STM32_USART_CR1_TE_SET(cr1, 1);

  /* configure USART 8 bits, no parity, 1 stop bit. */
  STM32_USART_CR1_M_SET(cr1, 0);
  STM32_USART_CR1_PCE_SET(cr1, 0);
  STM32_USART_CR2_STOP_SET(cr2, 0);

  /* configure baud rate */
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + STM32_USART_BRR_ADDR,
    endian_le32(CONFIG_DRIVER_STM32_USART_PRINTK_CLK_FREQ /
      CONFIG_DRIVER_STM32_USART_PRINTK_BAUDRATE));

  /* propagate the configuration. */
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + STM32_USART_CR1_ADDR, endian_le32(cr1));
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + STM32_USART_CR2_ADDR, endian_le32(cr2));

  /* enable USART. */
  STM32_USART_CR1_UE_SET(cr1, 1);
  cpu_mem_write_32(CONFIG_MUTEK_PRINTK_ADDR + STM32_USART_CR1_ADDR, endian_le32(cr1));

  printk_set_output(printk_out, NULL);
}

