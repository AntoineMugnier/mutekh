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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>
#include <hexo/interrupt.h>

#include <mutek/printk.h>
#include <mutek/startup.h>

#include <arch/psoc4/scb.h>
#include <arch/psoc4/hsiom.h>
#include <arch/psoc4/hsiom_port.h>
#include <arch/psoc4/gpio_port.h>
#include <arch/psoc4/peri.h>
#include <arch/psoc4/variant.h>

#if CONFIG_MUTEK_PRINTK_ADDR == PSOC4_SCB0_ADDR
# define PRINTK_PCLK_CHAN PSOC4_CLOCK_SCB0
#elif CONFIG_MUTEK_PRINTK_ADDR == PSOC4_SCB1_ADDR
# define PRINTK_PCLK_CHAN PSOC4_CLOCK_SCB1
#else
# error Unknown peripheral address for CONFIG_MUTEK_PRINTK_ADDR
#endif

static
void psoc4_printk_out_char(void *addr, char c)
{
  const uintptr_t scb = (uintptr_t)addr;

  while (SCB_TX_FIFO_STATUS_USED_GET(cpu_mem_read_32(scb + SCB_TX_FIFO_STATUS_ADDR)))
    ;
  cpu_mem_write_32(scb + SCB_TX_FIFO_WR_ADDR, c);
}

static
PRINTF_OUTPUT_FUNC(psoc4_printk_out_nodrv)
{
  size_t i;

  for (i = 0; i < len; i++) {
    if (str[i] == '\n')
      psoc4_printk_out_char(ctx, '\r');
    psoc4_printk_out_char(ctx, str[i]);
  }
}

void psoc4_printk_init(void)
{
  const uintptr_t scb = CONFIG_MUTEK_PRINTK_ADDR;
  const uint8_t port = PSOC4_IO_PORT(CONFIG_DRIVER_PSOC4_PRINTK_PIN);
  const uint8_t pin = PSOC4_IO_PIN(CONFIG_DRIVER_PSOC4_PRINTK_PIN);
  const uintptr_t gpio = PSOC4_GPIO_PRT_ADDR(port);
  const uintptr_t hsiom = PSOC4_HSIOM_PRT_ADDR(port);
  uint32_t tmp;

  // HF Clock is fixed to 24MHz, we need at least 8MHz for OVS
  // Divide peri clock by 3, get a 8MHz clock.
  cpu_mem_write_32(PSOC4_PERI_ADDR + PERI_DIV_16_CTL_ADDR(0), 0
                   | PERI_DIV_16_CTL_DIV_INT(2)
                   );

  // Enable Div16(0)
  cpu_mem_write_32(PSOC4_PERI_ADDR + PERI_DIV_CMD_ADDR, 0
                   | PERI_DIV_CMD_SEL_TYPE(16)
                   | PERI_DIV_CMD_SEL_DIV(0)
                   | PERI_DIV_CMD_PA_SEL_TYPE(24_5)
                   | PERI_DIV_CMD_PA_SEL_DIV(63)
                   | PERI_DIV_CMD_ENABLE
                   );

  // Wait for it
  while (!(cpu_mem_read_32(PSOC4_PERI_ADDR + PERI_DIV_16_CTL_ADDR(0))
           & PERI_DIV_16_CTL_EN))
    ;

  // Enable UART mode, 8x oversampling
  cpu_mem_write_32(scb + SCB_CTRL_ADDR, 0
                   | SCB_CTRL_MODE(UART)
                   | SCB_CTRL_BYTE_MODE
                   | SCB_CTRL_OVS(7)
                   );

  // Switch SCB_CLK on PClk0
  cpu_mem_write_32(PSOC4_PERI_ADDR + PERI_PCLK_CTL_ADDR(PRINTK_PCLK_CHAN), 0
                   | PERI_PCLK_CTL_SEL_TYPE(16)
                   | PERI_PCLK_CTL_SEL_DIV(0)
                   );

  // No IRQs
  cpu_mem_write_32(scb + SCB_INTR_TX_MASK_ADDR, 0
                   );

  cpu_mem_write_32(scb + SCB_INTR_RX_MASK_ADDR, 0
                   );

  // STD Submode
  cpu_mem_write_32(scb + SCB_UART_CTRL_ADDR, 0
                   | SCB_UART_CTRL_MODE(STD)
                   );

  // 1 stop bit
  cpu_mem_write_32(scb + SCB_UART_TX_CTRL_ADDR, 0
                   | SCB_UART_TX_CTRL_STOP_BITS(1)
                   );

  // 1 stop bit, 10-bit break
  cpu_mem_write_32(scb + SCB_UART_RX_CTRL_ADDR, 0
                   | SCB_UART_RX_CTRL_STOP_BITS(1)
                   | SCB_UART_RX_CTRL_BREAK_WIDTH(10*2-1)
                   );

  // No flow control
  cpu_mem_write_32(scb + SCB_UART_FLOW_CTRL_ADDR, 0
                   );

  // 8-bit data
  cpu_mem_write_32(scb + SCB_TX_CTRL_ADDR, 0
                   | SCB_TX_CTRL_DATA_WIDTH(7)
                   );

  // Clear fifos
  cpu_mem_write_32(scb + SCB_TX_FIFO_CTRL_ADDR, 0
                   | SCB_TX_FIFO_CTRL_CLEAR
                   );
  cpu_mem_write_32(scb + SCB_TX_FIFO_CTRL_ADDR, 0
                   );

  // 8-bit data
  cpu_mem_write_32(scb + SCB_RX_CTRL_ADDR, 0
                   | SCB_RX_CTRL_DATA_WIDTH(7)
                   );

  // Clear fifos
  cpu_mem_write_32(scb + SCB_RX_FIFO_CTRL_ADDR, 0
                   | SCB_RX_FIFO_CTRL_CLEAR
                   );
  cpu_mem_write_32(scb + SCB_RX_FIFO_CTRL_ADDR, 0
                   );

  // Enable UART mode, 8x oversampling
  cpu_mem_write_32(scb + SCB_CTRL_ADDR, 0
                   | SCB_CTRL_MODE(UART)
                   | SCB_CTRL_BYTE_MODE
                   | SCB_CTRL_ENABLED
                   | SCB_CTRL_OVS(7)
                   );

  // Route UART to GPIO pin
  tmp = cpu_mem_read_32(hsiom + HSIOM_PORT_SEL_ADDR);
  HSIOM_PORT_SEL_IO_SEL_SET(pin, tmp, ACT_1);
  cpu_mem_write_32(hsiom + HSIOM_PORT_SEL_ADDR, tmp);

  // Set pin mode
  tmp = cpu_mem_read_32(gpio + GPIO_PORT_PC_ADDR);
  GPIO_PORT_PC_DM_SET(pin, tmp, 0_1);
  cpu_mem_write_32(gpio + GPIO_PORT_PC_ADDR, tmp);

  printk_set_output(psoc4_printk_out_nodrv, (void*)scb);
}
