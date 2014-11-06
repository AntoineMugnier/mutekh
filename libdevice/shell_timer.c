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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include <device/shell.h>
#include <device/device.h>
#include <device/driver.h>

#include <device/class/timer.h>

enum timer_opts_e
{
  TIMER_OPT_DEV = 0x01,
  TIMER_OPT_DELAY = 0x02,
};

struct termui_optctx_dev_timer_opts
{
  struct device_timer_s timer;
  dev_timer_delay_t delay;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(timer_opts_cleanup)
{
  struct termui_optctx_dev_timer_opts *c = ctx;

  if (device_check_accessor(&c->timer))
      device_put_accessor(&c->timer);
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_timer_value)
{
  struct termui_optctx_dev_timer_opts *c = ctx;
  dev_timer_value_t value;

  if (DEVICE_OP(&c->timer, get_value, &value))
    return -EINVAL;

  termui_con_printf(con, "%llu\n", value);
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_timer_start)
{
  struct termui_optctx_dev_timer_opts *c = ctx;

  if (DEVICE_OP(&c->timer, start_stop, 1))
    return -EINVAL;
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_timer_stop)
{
  struct termui_optctx_dev_timer_opts *c = ctx;

  if (DEVICE_OP(&c->timer, start_stop, 0))
    return -EINVAL;
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_timer_sleep)
{
  struct termui_optctx_dev_timer_opts *c = ctx;

  struct dev_timer_rq_s rq = {
    .delay = c->delay
  };

  if (dev_timer_sleep(&c->timer, &rq))
    return -EINVAL;
  return 0;
}

static TERMUI_CON_OPT_DECL(dev_timer_opts) =
{
  /* option 0, mask is 0x1 */
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--timer-dev", TIMER_OPT_DEV,
                                    struct termui_optctx_dev_timer_opts, timer, DRIVER_CLASS_TIMER,
                                    TERMUI_CON_OPT_CONSTRAINTS(TIMER_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_INTEGER_ENTRY("-D", "--delay", TIMER_OPT_DELAY, struct termui_optctx_dev_timer_opts, delay, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(TIMER_OPT_DELAY, 0))

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_timer_group) =
{
  TERMUI_CON_ENTRY(dev_shell_timer_value, "value",
		   TERMUI_CON_OPTS_CTX(dev_timer_opts, TIMER_OPT_DEV, 0, timer_opts_cleanup)
                   )
  TERMUI_CON_ENTRY(dev_shell_timer_start, "start",
		   TERMUI_CON_OPTS_CTX(dev_timer_opts, TIMER_OPT_DEV, 0, timer_opts_cleanup)
                   )
  TERMUI_CON_ENTRY(dev_shell_timer_stop, "stop",
		   TERMUI_CON_OPTS_CTX(dev_timer_opts, TIMER_OPT_DEV, 0, timer_opts_cleanup)
                   )
  TERMUI_CON_ENTRY(dev_shell_timer_sleep, "sleep",
		   TERMUI_CON_OPTS_CTX(dev_timer_opts, TIMER_OPT_DEV | TIMER_OPT_DELAY, 0, timer_opts_cleanup)
                   )

  TERMUI_CON_LIST_END
};

