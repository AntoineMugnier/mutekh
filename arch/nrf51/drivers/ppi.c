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
#include <hexo/error.h>

#include <arch/ppi.h>

static uint32_t nrf51_ppi_free_map = 0xffff;

error_t nrf51_ppi_alloc(uint8_t *id, uint8_t count)
{
  uint8_t cur = 0;
  uint8_t i = 0;

  while (nrf51_ppi_free_map && i < count) {
    cur = __builtin_ctz(nrf51_ppi_free_map);
    id[i++] = cur;
    nrf51_ppi_free_map &= ~(1 << cur);
  }

  if (i == count)
    return 0;

  nrf51_ppi_free(id, i);

  return -ENOMEM;
}

void nrf51_ppi_free(const uint8_t *id, uint8_t count)
{
  for (uint8_t i = 0; i < count; ++i)
    nrf51_ppi_free_map |= 1 << id[i];
}
