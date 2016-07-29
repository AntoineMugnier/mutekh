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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2016
*/

#include <hexo/endian.h>
#include <inet/protocol/checksum.h>

uint16_t inet_checksum(const void *data, size_t size)
{
  const uint8_t *byte = data;
  uint32_t sum = 0;
  bool_t unaligned = (uintptr_t)data & 1;

  if (unaligned) {
    sum += endian_be16(byte[0]);
    byte++;
    size--;
  }

  const uint16_t *half = (const uint16_t *)byte;

  for (size_t i = 0; i < size / 2; i++)
    sum += half[i];

  if (size & 1)
    sum += endian_be16(byte[size - 1] << 8);

  if (unaligned)
    return endian_be16(sum + (sum >> 16));

  return sum + (sum >> 16);
}
