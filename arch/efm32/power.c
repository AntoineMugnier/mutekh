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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2015

*/

#include <hexo/power.h>
#include <hexo/iospace.h>
#include <cpu/arm32m/v7m.h>

error_t power_reboot()
{
  cpu_mem_write_32(ARMV7M_AIRCR_ADDR, ARMV7M_AIRCR_VECTKEY(KEY)
                   | ARMV7M_AIRCR_SYSRESETREQ);
  while (1)
    asm volatile("dmb");
}

error_t power_shutdown()
{
  return ENOTSUP;
}

enum power_reset_cause_e power_reset_cause(void)
{
  return POWER_RESET_CAUSE_UNKNOWN;
}
