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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2011

*/

#include <mutek/instrumentation.h>
#include <hexo/power.h>
#include <hexo/iospace.h>

error_t power_reboot()
{
  return ENOTSUP;
}

error_t power_shutdown()
{
  cpu_interrupt_disable();
  instrumentation_check_disable(INSTRUMENTATION_CHECK_ALL);

  /* FIXME vci simhelper hack */
  cpu_mem_write_32(0xd3200004, 0);

  return 0;
}

enum power_reset_cause_e power_reset_cause(void)
{
  return POWER_RESET_CAUSE_UNKNOWN;
}
