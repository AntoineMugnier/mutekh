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
    Copyright (c) 2017 Nicolas Pouillon <nipo@ssji.net>
*/

#include <hexo/power.h>
#include <hexo/iospace.h>
#include <arch/pic32/devaddr.h>
#include <arch/pic32/reset.h>
#include <arch/pic32/system.h>

error_t power_reboot()
{
  pic32_system_unlock();

  cpu_mem_write_32(PIC32_RSWRST_ADDR, 1);
  cpu_mem_read_32(PIC32_RSWRST_ADDR);

  return -ENOTSUP;
}

error_t power_shutdown()
{
  return -ENOTSUP;
}

enum power_reset_cause_e power_reset_cause(void)
{
  static enum power_reset_cause_e cause = POWER_RESET_CAUSE_UNKNOWN;

  if (cause == POWER_RESET_CAUSE_UNKNOWN) {
    uint32_t rcon = cpu_mem_read_32(PIC32_RCON_ADDR);

    cpu_mem_write_32(PIC32_RCON_ADDR, 0);
  
    if (rcon & PIC32_RCON_POR)
      cause = POWER_RESET_CAUSE_POWERUP;

    if (rcon & (PIC32_RCON_EXTR | PIC32_RCON_BOR))
      cause = POWER_RESET_CAUSE_HARD;

    if (rcon & (PIC32_RCON_WDTO | PIC32_RCON_DMTO))
      cause = POWER_RESET_CAUSE_WATCHDOG;

    if (rcon & PIC32_RCON_SWR)
      cause = POWER_RESET_CAUSE_SOFT;

    if (rcon & PIC32_RCON_SLEEP)
      cause = POWER_RESET_CAUSE_WAKEUP;
  }

  return cause;
}
