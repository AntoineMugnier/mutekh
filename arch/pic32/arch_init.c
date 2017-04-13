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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2015

*/

#include <mutek/startup.h>

#include <string.h>

#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/memory_allocator.h>
#include <arch/pic32/clk.h>
#include <arch/pic32/pin.h>
#include <arch/pic32/devaddr.h>
#include <arch/pic32/gpio.h>
#include <mutek/printk.h>
#include <mutek/startup.h>

void pic32_mem_init(void)
{
  default_region = memory_allocator_init(NULL, (void*)CONFIG_STARTUP_HEAP_ADDR,
                                         (void*)(CONFIG_STARTUP_HEAP_ADDR +
                                                 CONFIG_STARTUP_HEAP_SIZE));
}

void pic32_clk_init(void)
{

  /* Unlock sequence */
  cpu_mem_write_32(PIC32_SYSKEY_ADDR, 0x00000000);
  cpu_mem_write_32(PIC32_SYSKEY_ADDR, 0xAA996655);
  cpu_mem_write_32(PIC32_SYSKEY_ADDR, 0x556699AA);


  uint32_t x = cpu_mem_read_32(PIC32_CMU_ADDR + PIC32_CLOCK_CTRL_ADDR);

  if (x & PIC32_CLOCK_CTRL_CLKLOCK)
    printk("Warning: Clock configuration can not be modified\n");

  x = PIC32_CLOCK_CTRL_NOSC(FRC) |
      PIC32_CLOCK_CTRL_FRCDIV(DIV1) |
      PIC32_CLOCK_CTRL_OSWEN;

  cpu_mem_write_32(PIC32_CMU_ADDR + PIC32_CLOCK_CTRL_ADDR, endian_le32(x));

  while(1)
    {
      x = cpu_mem_read_32(PIC32_CMU_ADDR + PIC32_CLOCK_CTRL_ADDR);
      if (!(x & PIC32_CLOCK_CTRL_OSWEN))
        break;
    }

  /* Configure SPLL @ 200 MHz from FRC @ 8 MHz */
  x = PIC32_CLOCK_SPLLCTRL_PLLCLK |
      PIC32_CLOCK_SPLLCTRL_PLLMULT(49) |
      PIC32_CLOCK_SPLLCTRL_PLLODIV(1) |
      PIC32_CLOCK_SPLLCTRL_PLLDIV(0) |
      PIC32_CLOCK_SPLLCTRL_RANGE(1);

  cpu_mem_write_32(PIC32_CMU_ADDR + PIC32_CLOCK_SPLLCTRL_ADDR, endian_le32(x));

  x = PIC32_CLOCK_CTRL_NOSC(SPLL) |
      PIC32_CLOCK_CTRL_OSWEN;

  cpu_mem_write_32(PIC32_CMU_ADDR + PIC32_CLOCK_CTRL_ADDR, endian_le32(x));

  while(1)
    {
      x = cpu_mem_read_32(PIC32_CMU_ADDR + PIC32_CLOCK_CTRL_ADDR);
      if (!(x & PIC32_CLOCK_CTRL_OSWEN))
        break;
    }

  /* Enable Peripheral bus clock @ 200 MHz for PB7 and @100 MHz for others clocks */

  for (uint8_t i = 0; i<8; i++)
    {
      do{
          x = cpu_mem_read_32(PIC32_CMU_ADDR + PIC32_CLOCK_PBDIV_ADDR(i));
        }while(!(x & PIC32_CLOCK_PBDIV_DIBRDY));

      if (i == 6)
        cpu_mem_write_32(PIC32_CMU_ADDR + PIC32_CLOCK_PBDIV_ADDR(i), PIC32_CLOCK_PBDIV_ON);
      else
        cpu_mem_write_32(PIC32_CMU_ADDR + PIC32_CLOCK_PBDIV_ADDR(i), PIC32_CLOCK_PBDIV_ON | PIC32_CLOCK_PBDIV_DIV(1));
    }

  /* Lock */
  cpu_mem_write_32(PIC32_SYSKEY_ADDR, 0x33333333);
}
