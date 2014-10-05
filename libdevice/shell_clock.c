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

#include <device/class/clock.h>

enum mem_opts_e
{
  CLOCK_OPT_DEV    = 0x01,
};

struct termui_optctx_dev_clock_opts
{
  struct device_clock_s clock;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(clock_opts_cleanup)
{
  struct termui_optctx_dev_clock_opts *c = ctx;

  if (device_check_accessor(&c->clock))
      device_put_accessor(&c->clock);
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_clock_config)
{
  struct termui_optctx_dev_clock_opts *c = ctx;
  int_fast8_t i = atoi(argv[0]);

  if (dev_clock_config(&c->clock, i))
    return -EINVAL;
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_clock_freqs)
{
  struct termui_optctx_dev_clock_opts *c = ctx;
  dev_clock_node_id_t i;

  for (i = 0; ; i++)
    {
      struct dev_clock_node_info_s info;
      enum dev_clock_node_info_e mask =
        DEV_CLOCK_INFO_FREQ | DEV_CLOCK_INFO_NAME |
        DEV_CLOCK_INFO_PARENT | DEV_CLOCK_INFO_RUNNING;

      error_t err = DEVICE_OP(&c->clock, node_info, i, &mask, &info);
      if (err == -EINVAL)
        break;

      termui_con_printf(con, "  node %-3u : %16s @ %llu/%llu Hz, parent %u, %s\n",
        i, info.name, (uint64_t)info.freq.num,
        (uint64_t)info.freq.denom, info.parent_id, info.running ? "running" : "");
    }

  return 0;
}

static TERMUI_CON_OPT_DECL(dev_clock_opts) =
{
  /* option 0, mask is 0x1 */
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--clock-dev", CLOCK_OPT_DEV,
                                    struct termui_optctx_dev_clock_opts, clock, DRIVER_CLASS_CLOCK,
                                    TERMUI_CON_OPT_CONSTRAINTS(CLOCK_OPT_DEV, 0)
                                    )

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_clock_group) =
{
  TERMUI_CON_ENTRY(dev_shell_clock_config, "config",
		   TERMUI_CON_OPTS_CTX(dev_clock_opts, CLOCK_OPT_DEV, 0, clock_opts_cleanup)
                   TERMUI_CON_ARGS(1, 1)
                   )

  TERMUI_CON_ENTRY(dev_shell_clock_freqs, "freqs",
		   TERMUI_CON_OPTS_CTX(dev_clock_opts, CLOCK_OPT_DEV, 0, clock_opts_cleanup)
                   TERMUI_CON_ARGS(0, 0)
                   )

  TERMUI_CON_LIST_END
};

