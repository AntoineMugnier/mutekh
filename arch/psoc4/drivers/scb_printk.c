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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2015
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/printk.h>
#include <psoc4/scb.h>
#include <psoc4/hsiom.h>
#include <psoc4/memory_map.h>
#include <psoc4/pin.h>

#if CONFIG_MUTEK_PRINTK_ADDR == SCB0_CTRL_ADDR
# define PRINTK_SCB SCB0
# define PRINTK_SCB_NO 0
#elif CONFIG_MUTEK_PRINTK_ADDR == SCB1_CTRL_ADDR
# define PRINTK_SCB SCB1
# define PRINTK_SCB_NO 1
#else
# error Unknown peripheral address for CONFIG_MUTEK_PRINTK_ADDR
#endif

void scb_printk_out_char(void *addr, char c)
{
  const uintptr_t scb = (uintptr_t)addr;

  while (!(cpu_mem_read_32(scb + SCB_INTR_TX_ADDR) & SCB_INTR_TX_NOT_FULL))
    ;
  cpu_mem_write_32(scb + SCB_TX_FIFO_WR_ADDR, c);
}

PRINTF_OUTPUT_FUNC(scb_printk_out_nodrv)
{
  size_t i;

  for (i = 0; i < len; i++) {
    if (str[i] == '\n')
      scb_printk_out_char(ctx, '\r');
    scb_printk_out_char(ctx, str[i]);
  }
}

void scb_printk_init()
{
  const uintptr_t scb = CONFIG_MUTEK_PRINTK_ADDR;
  uint32_t tmp;

  cpu_mem_write_32(scb + SCB_CTRL_ADDR, 0
                   | SCB_CTRL_MODE(UART)
                   | SCB_CTRL_BYTE_MODE(8_BIT_ELEMENTS)
                   | SCB_CTRL_ENABLED
                   );

  cpu_mem_write_32(scb + SCB_UART_CTRL_ADDR, 0
                   | SCB_UART_CTRL_MODE(STD)
                   );

  cpu_mem_write_32(scb + SCB_UART_TX_CTRL_ADDR, 0
                   | SCB_UART_TX_CTRL_STOP_BITS(1)
                   );

  cpu_mem_write_32(scb + SCB_UART_RX_CTRL_ADDR, 0
                   | SCB_UART_RX_CTRL_STOP_BITS(1)
                   );

  cpu_mem_write_32(scb + SCB_UART_FLOW_CTRL_ADDR, 0
                   );

  cpu_mem_write_32(scb + SCB_TX_CTRL_ADDR, 0
                   | SCB_TX_CTRL_DATA_WIDTH(8)
                   );

  cpu_mem_write_32(scb + SCB_TX_FIFO_CTRL_ADDR, 0
                   | SCB_TX_FIFO_CTRL_CLEAR
                   );

  cpu_mem_write_32(scb + SCB_TX_FIFO_CTRL_ADDR, 0
                   );

  cpu_mem_write_32(scb + SCB_RX_CTRL_ADDR, 0
                   | SCB_RX_CTRL_DATA_WIDTH(8)
                   );

  cpu_mem_write_32(scb + SCB_RX_FIFO_CTRL_ADDR, 0
                   | SCB_RX_FIFO_CTRL_CLEAR
                   );

  cpu_mem_write_32(scb + SCB_RX_FIFO_CTRL_ADDR, 0
                   );

  cpu_mem_write_32(scb + SCB_INTR_TX_MASK_ADDR, 0
                   );

  cpu_mem_write_32(scb + SCB_INTR_RX_MASK_ADDR, 0
                   );

  tmp = cpu_mem_read_32(HSIOM_PORT_SEL_ADDR(CONFIG_DRIVER_PSOC4_PRINTK_PORT));
  PSOC4_PMUX_SET(tmp,
                   CONFIG_DRIVER_PSOC4_PRINTK_PORT,
                   CONFIG_DRIVER_PSOC4_PRINTK_PIN,
                   PRINTK_SCB,
                   UART_TX);
  cpu_mem_write_32(HSIOM_PORT_SEL_ADDR(CONFIG_DRIVER_PSOC4_PRINTK_PORT), tmp);

  printk_set_output(scb_printk_out_nodrv, (void*)scb);
}
