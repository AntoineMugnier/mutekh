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
#include <hexo/iospace.h>
#include <hexo/interrupt.h>
#include <hexo/bit.h>

#include <mutek/startup.h>

#include <arch/psoc4/srss.h>
#include <arch/psoc4/variant.h>

#include "imo.h"

#define SRSS PSOC4_SRSS_ADDR

void psoc4_clock_setup(void)
{
  cpu_mem_write_32(SRSS + SRSS_CLK_SELECT_ADDR, 0
                   | SRSS_CLK_SELECT_DIRECT_SEL(IMO)
                   );

  psoc4_imo_mhz_set(24);

  cpu_mem_write_32(SRSS + SRSS_CLK_ILO_CONFIG_ADDR, 0
                   | SRSS_CLK_ILO_CONFIG_ENABLE
                   | SRSS_CLK_ILO_CONFIG_SATBIAS(SATURATED)
                   | SRSS_CLK_ILO_CONFIG_TURBO_EN
                   );
}
