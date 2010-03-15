/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

#include <string.h>
#include <mutek/printk.h>

PRINTF_OUTPUT_FUNC(early_console_vga)
{
  uint16_t *ptr = (void*)0xb8000;
  static uint_fast16_t cursor = 0;
  static const size_t width = 80;
  static const size_t height = 25;

  while (len--)
    {
      char c = *str++;

      if (c == '\n') {
        if (cursor >= (height - 1) * width) {
          memmove(ptr, ptr + width, (height - 1) * width * 2);
          cursor = (height - 1) * width;
        } else {
          cursor = (cursor / width + 1) * width;
        }
        memset(ptr + cursor, 0, width * 2);
      } else {
        ptr[cursor++] = c | 0x0200;
      }

      if (cursor >= height * width)
        cursor = 0;
    }

  /* force memory write */
  asm volatile("":::"memory");
}

