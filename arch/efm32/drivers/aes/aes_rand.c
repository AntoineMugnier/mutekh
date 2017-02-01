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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2015

*/

#include "aes.h"

void efm32_aes_random(uint32_t state[8], uint32_t *out)
{
  /*
   ______              ______
  /      \            /      \
  |      |            |      |
  |      v            v      |
  |  +----------|---------+  |
  |  |   256 bits state   |  |
  |  +----------|---------+  |
  |      |            |      |
 XOR<----+----.  .----+---->XOR
  ^      |     \/     |      ^
  |      v     /\     v      |
  |     AES<--'  '-->AES     |
  |      |            |      |
  \______/            |______/
                      |
                      v
                     Out

   */

  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_CTRL_ADDR, EFM32_AES_CTRL_DATASTART);

  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_KEYL_ADDR(0), state[4]);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_KEYL_ADDR(1), state[5]);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_KEYL_ADDR(2), state[6]);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_KEYL_ADDR(3), state[7]);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR,    state[0]);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR,    state[1]);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR,    state[2]);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR,    state[3]);
  efm32_aes_wait();

  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_KEYL_ADDR(0), state[0]);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_KEYL_ADDR(1), state[1]);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_KEYL_ADDR(2), state[2]);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_KEYL_ADDR(3), state[3]);

  state[0] ^= cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
  state[1] ^= cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
  state[2] ^= cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
  state[3] ^= cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);

  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, state[4]);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, state[5]);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, state[6]);
  cpu_mem_write_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR, state[7]);
  efm32_aes_wait();

  uint32_t x0 = cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
  uint32_t x1 = cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
  uint32_t x2 = cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);
  uint32_t x3 = cpu_mem_read_32(EFM32_AES_ADDR + EFM32_AES_DATA_ADDR);

  state[4] ^= x0;
  state[5] ^= x1;
  state[6] ^= x2;
  state[7] ^= x3;

  if (out != NULL)
    {
      out[0] = x0;
      out[1] = x1;
      out[2] = x2;
      out[3] = x3;
    }
}

