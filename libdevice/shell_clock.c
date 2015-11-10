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
  CLOCK_OPT_NODE   = 0x02,
  CLOCK_OPT_PARENT = 0x04,
  CLOCK_OPT_FREQ   = 0x08,
  CLOCK_OPT_RATIO  = 0x10,
  CLOCK_OPT_NOCOMMIT = 0x20,
};

struct termui_optctx_dev_clock_opts
{
  struct device_clock_s clock;
  dev_clock_node_id_t   node;
  union {
    struct dev_freq_s     freq;
    struct {
      dev_clock_node_id_t   parent;
      struct dev_freq_ratio_s ratio;
    };
  };
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

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_clock_route)
{
  struct termui_optctx_dev_clock_opts *c = ctx;

  if (DEVICE_OP(&c->clock, config_route, c->node,
                c->parent, &c->ratio))
    return -EINVAL;

  if (!(used & CLOCK_OPT_NOCOMMIT) && DEVICE_OP(&c->clock, commit))
    return -EINVAL;

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_clock_oscillator)
{
  struct termui_optctx_dev_clock_opts *c = ctx;

  struct dev_freq_accuracy_s acc = {};

  if (DEVICE_OP(&c->clock, config_oscillator, c->node, &c->freq, &acc))
    return -EINVAL;

  if (!(used & CLOCK_OPT_NOCOMMIT) && DEVICE_OP(&c->clock, commit))
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

      if (!(mask & DEV_CLOCK_INFO_NAME))
        info.name = "?";
      termui_con_printf(con, "  node %-3u : %16s", i, info.name);

      if (mask & DEV_CLOCK_INFO_FREQ)
        {
          termui_con_printf(con, " @ %llu", (uint64_t)info.freq.num);
          if (info.freq.denom != 1)
            termui_con_printf(con, "/%llu", (uint64_t)info.freq.denom);
          termui_con_printf(con, " Hz");
        }

      if (mask & DEV_CLOCK_INFO_ACCURACY)
        termui_con_printf(con, ", %u ppb",
                          dev_freq_acc_ppb(&info.acc));
      if (mask & DEV_CLOCK_INFO_PARENT)
        termui_con_printf(con, ", parent %u", info.parent_id);
      if (mask & DEV_CLOCK_INFO_SRC)
        termui_con_printf(con, ", src_ep %p", info.src);
      if (mask & DEV_CLOCK_INFO_SINK)
        termui_con_printf(con, ", sink_ep %p", info.sink);
      if (info.running)
        termui_con_printf(con, ", Running");
      termui_con_printf(con, "\n");
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

  TERMUI_CON_OPT_INTEGER_ENTRY("-n", "--node", CLOCK_OPT_NODE,
                               struct termui_optctx_dev_clock_opts, node, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(CLOCK_OPT_NODE, 0)
                               )

  TERMUI_CON_OPT_INTEGER_ENTRY("-p", "--parent", CLOCK_OPT_PARENT,
                               struct termui_optctx_dev_clock_opts, parent, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(CLOCK_OPT_PARENT, 0)
                               )

  TERMUI_CON_OPT_FREQ_ENTRY("-f", "--freq", CLOCK_OPT_FREQ,
                            struct termui_optctx_dev_clock_opts, freq,
                            TERMUI_CON_OPT_CONSTRAINTS(CLOCK_OPT_FREQ, 0)
                            )

  TERMUI_CON_OPT_FREQ_RATIO_ENTRY("-r", "--ratio", CLOCK_OPT_RATIO,
                                  struct termui_optctx_dev_clock_opts, ratio,
                                  TERMUI_CON_OPT_CONSTRAINTS(CLOCK_OPT_RATIO, 0)
                                  )

  TERMUI_CON_OPT_ENTRY("-N", "--no-commit", CLOCK_OPT_NOCOMMIT,
                       TERMUI_CON_OPT_CONSTRAINTS(CLOCK_OPT_NOCOMMIT, 0)
                       )

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_clock_group) =
{
  TERMUI_CON_ENTRY(dev_shell_clock_config, "config",
		   TERMUI_CON_OPTS_CTX(dev_clock_opts, CLOCK_OPT_DEV, 0, clock_opts_cleanup)
                   TERMUI_CON_ARGS(1, 1)
                   )

  TERMUI_CON_ENTRY(dev_shell_clock_route, "route",
		   TERMUI_CON_OPTS_CTX(dev_clock_opts, CLOCK_OPT_DEV | CLOCK_OPT_NODE | CLOCK_OPT_RATIO | CLOCK_OPT_PARENT,
                                       CLOCK_OPT_NOCOMMIT, clock_opts_cleanup)
                   )

  TERMUI_CON_ENTRY(dev_shell_clock_oscillator, "oscillator",
		   TERMUI_CON_OPTS_CTX(dev_clock_opts, CLOCK_OPT_DEV | CLOCK_OPT_NODE | CLOCK_OPT_FREQ,
                                       CLOCK_OPT_NOCOMMIT, clock_opts_cleanup)
                   )

  TERMUI_CON_ENTRY(dev_shell_clock_freqs, "freqs",
		   TERMUI_CON_OPTS_CTX(dev_clock_opts, CLOCK_OPT_DEV, 0, clock_opts_cleanup)
                   TERMUI_CON_ARGS(0, 0)
                   )

  TERMUI_CON_LIST_END
};

