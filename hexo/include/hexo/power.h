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

/**
 * @file
 * @module {Core::Hardware abstraction layer}
 * @short Power management API
 */

#ifndef HEXO_POWER_H_
#define HEXO_POWER_H_

#include <hexo/error.h>
#include <hexo/enum.h>

/** Hard reboot the platform. May return ENOTSUP. */
error_t power_reboot(void);

/** Turn system power off. May return ENOTSUP. */
error_t power_shutdown(void);

enum power_reset_cause_e
{
  POWER_RESET_CAUSE_UNKNOWN,
  POWER_RESET_CAUSE_POWERUP,
  POWER_RESET_CAUSE_HARD,
  POWER_RESET_CAUSE_SOFT,
  POWER_RESET_CAUSE_WATCHDOG,
  POWER_RESET_CAUSE_WAKEUP,
  POWER_RESET_CAUSE_FAULT,
};

ENUM_DESCRIPTOR(power_reset_cause_e, strip:POWER_RESET_CAUSE_, upper);

/** Get chip reset cause. */
enum power_reset_cause_e power_reset_cause(void);

#endif

