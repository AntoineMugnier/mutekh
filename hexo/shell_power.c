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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2016
*/

#include <hexo/iospace.h>

#include <mutek/shell.h>
#include <stdlib.h>

#include <hexo/power.h>
#include <enums.h>

static TERMUI_CON_COMMAND_PROTOTYPE(shell_power_cause)
{
  enum power_reset_cause_e cause = power_reset_cause();

  termui_con_printf(con, "Reset cause: %N\n", cause, ENUM_DESC_POWER_RESET_CAUSE_E);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_power_shutdown)
{
  power_shutdown();

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_power_reboot)
{
  power_reboot();

  return 0;
}

static TERMUI_CON_GROUP_DECL(shell_power_subgroup) =
{
  TERMUI_CON_ENTRY(shell_power_cause, "reset_cause")
  TERMUI_CON_ENTRY(shell_power_shutdown, "shutdown")
  TERMUI_CON_ENTRY(shell_power_reboot, "reboot")
  TERMUI_CON_LIST_END
};

MUTEK_SHELL_ROOT_GROUP(shell_power_subgroup, "power")
