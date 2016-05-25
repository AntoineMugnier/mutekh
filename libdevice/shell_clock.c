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
#include <inttypes.h>

#include <device/class/cmu.h>

enum mem_opts_e
{
  CLOCK_OPT_DEV    = 0x01,
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  CLOCK_OPT_CFG    = 0x02,
  CLOCK_OPT_NODE   = 0x04,
  CLOCK_OPT_PARENT = 0x08,
  CLOCK_OPT_FREQ   = 0x10,
  CLOCK_OPT_RATIO  = 0x20,
  CLOCK_OPT_NOCOMMIT = 0x40,
#endif
};

struct termui_optctx_dev_clock_opts
{
  struct device_cmu_s cmu;
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  dev_cmu_config_id_t cfg;
  dev_cmu_node_id_t   node;
  union {
    struct dev_freq_s     freq;
    struct {
      dev_cmu_node_id_t   parent;
      struct dev_freq_ratio_s ratio;
    };
  };
#endif
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(clock_opts_cleanup)
{
  struct termui_optctx_dev_clock_opts *c = ctx;

  if (device_check_accessor(&c->cmu.base))
      device_put_accessor(&c->cmu.base);
}

static void dev_shell_clock_configs(struct termui_console_s *con,
                                    struct termui_optctx_dev_clock_opts *c)
{
  uint_fast8_t i = 0;

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  for (i = 0; i < sizeof(dev_cmu_config_mask_t) * 8; i++)
#endif
    {
      bool_t head = 0;

      DEVICE_RES_FOREACH(c->cmu.dev, r, {
          switch (r->type)
            {
            case DEV_RES_CMU_MUX: {
              if (!((r->u.cmu_mux.config >> i) & 1))
                break;

              if (!head)
                termui_con_printf(con, "\nConfig %u:\n", i);
              head = 1;

              struct dev_cmu_node_info_s info;
              enum dev_cmu_node_info_e m = DEV_CMU_INFO_NAME;
              const char *nname = "unknown";
              const char *pname = "unknown";
              if (!dev_cmu_node_info_get(&c->cmu, r->u.cmu_mux.node, &m, &info) && m)
                nname = info.name;
              if (!dev_cmu_node_info_get(&c->cmu, r->u.cmu_mux.parent, &m, &info) && m)
                pname = info.name;
              termui_con_printf(con, "  mux: node %u `%s': parent %u `%s', ratio %"PRIu64"/%"PRIu64"\n",
                                r->u.cmu_mux.node, nname, r->u.cmu_mux.parent, pname,
                                (uint64_t)r->u.cmu_mux.num, (uint64_t)r->u.cmu_mux.denom);
              break;
            }

            case DEV_RES_CMU_OSC: {
              if (!((r->u.cmu_osc.config >> i) & 1))
                break;

              if (!head)
                termui_con_printf(con, "\nConfig %u:\n", i);
              head = 1;

              struct dev_cmu_node_info_s info;
              enum dev_cmu_node_info_e m = DEV_CMU_INFO_NAME;
              const char *nname = "unknown";
              if (!dev_cmu_node_info_get(&c->cmu, r->u.cmu_osc.node, &m, &info) && m)
                nname = info.name;
              uint64_t integral  = r->u.cmu_osc.num / r->u.cmu_osc.denom;
              uint32_t frac      = 1000 * (r->u.cmu_osc.num % r->u.cmu_osc.denom) /
                                   r->u.cmu_osc.denom;

              termui_con_printf(con, "  osc: node %u `%s' @ %"PRIu64".%03"PRIu32" Hz",
                                (uint_fast8_t)r->u.cmu_osc.node, nname, (uint64_t)integral, (uint32_t)frac);

              if (r->u.cmu_osc.acc_e || r->u.cmu_osc.acc_m)
                {
                  uint32_t ppb = dev_acc_ppb(r->u.cmu_osc.acc_m,
                                             r->u.cmu_osc.acc_e);

                  if (ppb > 10000000)
                    termui_con_printf(con, ", %u %%", (ppb + 5000000) / 10000000);
                  else if (ppb > 1000)
                    termui_con_printf(con, ", %u ppm", (ppb + 500) / 1000);
                  else
                    termui_con_printf(con, ", %u ppb", ppb);
                }

              termui_con_printf(con, "\n");

              break;
            }

            default:
              break;
            }
        });
    }
}


static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_clock_config)
{
  struct termui_optctx_dev_clock_opts *c = ctx;

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  if (used & CLOCK_OPT_CFG)
    {
      if (dev_cmu_configure(&c->cmu, c->cfg))
        return -EINVAL;
      return 0;
    }
#endif

  dev_shell_clock_configs(con, c);
  return 0;
}

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_clock_mux)
{
  struct termui_optctx_dev_clock_opts *c = ctx;

  error_t err;
  LOCK_SPIN_IRQ(&c->cmu.dev->lock);
  err = DEVICE_OP(&c->cmu, config_mux, c->node, c->parent, &c->ratio);
  LOCK_RELEASE_IRQ(&c->cmu.dev->lock);
  if (err)
    return -EINVAL;

  if (!(used & CLOCK_OPT_NOCOMMIT) && DEVICE_OP(&c->cmu, commit))
    return -EINVAL;

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_clock_osc)
{
  struct termui_optctx_dev_clock_opts *c = ctx;

  error_t err;
  LOCK_SPIN_IRQ(&c->cmu.dev->lock);
  err = DEVICE_OP(&c->cmu, config_osc, c->node, &c->freq);
  LOCK_RELEASE_IRQ(&c->cmu.dev->lock);
  if (err)
    return -EINVAL;

  if (!(used & CLOCK_OPT_NOCOMMIT) && DEVICE_OP(&c->cmu, commit))
    return -EINVAL;

  return 0;
}
#endif

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_clock_nodes)
{
  struct termui_optctx_dev_clock_opts *c = ctx;
  dev_cmu_node_id_t i;

  for (i = 0; ; i++)
    {
      struct dev_cmu_node_info_s info;
      enum dev_cmu_node_info_e mask =
        DEV_CMU_INFO_FREQ | DEV_CMU_INFO_SCALE |
        DEV_CMU_INFO_NAME | DEV_CMU_INFO_ACCURACY |
        DEV_CMU_INFO_SRC | DEV_CMU_INFO_SINK |
        DEV_CMU_INFO_PARENT | DEV_CMU_INFO_RUNNING;

      error_t err = DEVICE_OP(&c->cmu, node_info, i, &mask, &info);
      if (err == -EINVAL)
        break;

      if (!(mask & DEV_CMU_INFO_NAME))
        info.name = "?";
      termui_con_printf(con, "  node %-3u : %16s", i, info.name);

      if (mask & DEV_CMU_INFO_FREQ)
        {
          termui_con_printf(con, " @ %llu", (uint64_t)info.freq.num);
          if (info.freq.denom != 1)
            termui_con_printf(con, "/%llu", (uint64_t)info.freq.denom);
          termui_con_printf(con, " Hz");
        }

      if (mask & DEV_CMU_INFO_SCALE && info.scale.denom != info.scale.num)
        {
          termui_con_printf(con, " x %llu", (uint64_t)info.scale.num);
          if (info.scale.denom != 1)
            termui_con_printf(con, "/%llu", (uint64_t)info.scale.denom);
        }

      if (mask & DEV_CMU_INFO_ACCURACY)
        {
          uint32_t ppb = dev_freq_acc_ppb(&info.freq);

          if (ppb > 10000000)
            termui_con_printf(con, ", %u %%", (ppb + 5000000) / 10000000);
          else if (ppb > 1000)
            termui_con_printf(con, ", %u ppm", (ppb + 500) / 1000);
          else if (ppb)
            termui_con_printf(con, ", %u ppb", ppb);
        }

      if (mask & DEV_CMU_INFO_PARENT)
        termui_con_printf(con, ", parent %u", info.parent_id);
      if (mask & DEV_CMU_INFO_SRC)
        termui_con_printf(con, ", src_ep %p", info.src);
      if (mask & DEV_CMU_INFO_SINK)
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
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--cmu-dev", CLOCK_OPT_DEV,
                                    struct termui_optctx_dev_clock_opts, cmu, DRIVER_CLASS_CMU,
                                    TERMUI_CON_OPT_CONSTRAINTS(CLOCK_OPT_DEV, 0)
                                    )

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  TERMUI_CON_OPT_INTEGER_ENTRY("-c", "--cfg-id", CLOCK_OPT_CFG,
                               struct termui_optctx_dev_clock_opts, cfg, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(CLOCK_OPT_CFG, 0)
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
#endif

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_clock_group) =
{
  TERMUI_CON_ENTRY(dev_shell_clock_config, "config",
		   TERMUI_CON_OPTS_CTX(dev_clock_opts, CLOCK_OPT_DEV, 0
#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
                                       | CLOCK_OPT_CFG
#endif
                                       , clock_opts_cleanup)
                   )

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  TERMUI_CON_ENTRY(dev_shell_clock_mux, "mux",
		   TERMUI_CON_OPTS_CTX(dev_clock_opts, CLOCK_OPT_DEV | CLOCK_OPT_NODE | CLOCK_OPT_RATIO | CLOCK_OPT_PARENT,
                                       CLOCK_OPT_NOCOMMIT, clock_opts_cleanup)
                   )

  TERMUI_CON_ENTRY(dev_shell_clock_osc, "osc",
		   TERMUI_CON_OPTS_CTX(dev_clock_opts, CLOCK_OPT_DEV | CLOCK_OPT_NODE | CLOCK_OPT_FREQ,
                                       CLOCK_OPT_NOCOMMIT, clock_opts_cleanup)
                   )
#endif

  TERMUI_CON_ENTRY(dev_shell_clock_nodes, "nodes",
		   TERMUI_CON_OPTS_CTX(dev_clock_opts, CLOCK_OPT_DEV, 0, clock_opts_cleanup)
                   )

  TERMUI_CON_LIST_END
};

