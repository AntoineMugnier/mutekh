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

#ifndef IMO_H_
#define IMO_H_

#include <hexo/types.h>

uint_fast8_t clk_imo_mhz(uint_fast8_t freq_trim2);
uint_fast8_t clk_imo_trim2(uint_fast8_t freq_mhz);
void psoc4_flash_set_freq(uint_fast8_t freq_trim2);
void psoc4_imo_mhz_set(uint_fast8_t freq_mhz);

#endif
