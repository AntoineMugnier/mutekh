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
#include <inttypes.h>

enum timer_opts_e
{
  TIMER_OPT_DEV    = 0x01,
  TIMER_OPT_TDELAY = 0x02,
  TIMER_OPT_MDELAY = 0x04,
  TIMER_OPT_RES    = 0x08,
};

struct termui_optctx_dev_timer_opts
{
  struct device_timer_s timer;
  union {
    dev_timer_delay_t delay;
    dev_timer_res_t res;
  };
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(timer_opts_cleanup)
{
  struct termui_optctx_dev_timer_opts *c = ctx;

  if (device_check_accessor(&c->timer.base))
      device_put_accessor(&c->timer.base);
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_timer_value)
{
  struct termui_optctx_dev_timer_opts *c = ctx;
  dev_timer_value_t value;

  if (DEVICE_OP(&c->timer, get_value, &value, 0))
    return -EINVAL;

  termui_con_printf(con, "%llu\n", value);
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_timer_wait)
{
  struct termui_optctx_dev_timer_opts *c = ctx;

  struct dev_timer_rq_s rq;
  rq.deadline = 0;

  if (used & TIMER_OPT_MDELAY)
    {
      if (dev_timer_init_sec(&c->timer, &rq.delay, &rq.rev, c->delay, 1000))
        return -EINVAL;
      termui_con_printf(con, "tick : %"PRItimerDelay"\n", rq.delay);
    }
  else
    {
      rq.delay = c->delay;
      rq.rev = 0;
    }

  if (dev_timer_wait_rq(&c->timer, &rq))
    return -EINVAL;

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_timer_config)
{
  struct termui_optctx_dev_timer_opts *c = ctx;
  error_t err;

  dev_timer_res_t res = 0;

  if (used & TIMER_OPT_RES)
    res = c->res;

  struct dev_timer_config_s cfg;
  err = DEVICE_OP(&c->timer, config, &cfg, res);

  if (err && err != -ERANGE)
    {
      termui_con_printf(con, "err : %i\n", err);
    }
  else
    {
      if (DEV_FREQ_IS_VALID(cfg.freq))
        termui_con_printf(con,
                          "freq: %llu/%llu\n",
                          (uint64_t)cfg.freq.num,
                          (uint64_t)cfg.freq.denom);
      if (DEV_FREQ_ACC_IS_VALID(cfg.freq))
        termui_con_printf(con,
                          "acc : %u ppb\n",
                          dev_freq_acc_ppb(&cfg.freq));
      termui_con_printf(con,
                        "rev : %"PRItimerRev"\n"
                        "res : %"PRItimerRes"\n"
                        "max : %"PRItimerValue"\n"
                        "cap : ",
                        cfg.rev, cfg.res, cfg.max);
      termui_con_print_enum(con, dev_timer_capabilities_e, cfg.cap);
      termui_con_printf(con, "\n");
    }

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_timer_convert)
{
  struct termui_optctx_dev_timer_opts *c = ctx;

  dev_timer_cfgrev_t rev;

  if (used & TIMER_OPT_MDELAY)
    {
      dev_timer_delay_t delay;
      if (dev_timer_init_sec(&c->timer, &delay, &rev, c->delay, 1000))
        return -EINVAL;
      termui_con_printf(con, "tick : %"PRItimerDelay"\n", delay);
    }
  else
    {
      uint64_t stime;
      if (dev_timer_get_sec(&c->timer, &stime, &rev, c->delay, 1000))
        return -EINVAL;
      termui_con_printf(con, "msec : %"PRIu64"\n", stime);
    }

  return 0;
}

static TERMUI_CON_OPT_DECL(dev_timer_opts) =
{
  /* option 0, mask is 0x1 */
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--timer-dev", TIMER_OPT_DEV,
                                    struct termui_optctx_dev_timer_opts, timer, DRIVER_CLASS_TIMER,
                                    TERMUI_CON_OPT_CONSTRAINTS(TIMER_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_INTEGER_ENTRY("-D", "--delay", TIMER_OPT_TDELAY, struct termui_optctx_dev_timer_opts, delay, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(TIMER_OPT_MDELAY | TIMER_OPT_TDELAY, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-m", "--msec", TIMER_OPT_MDELAY, struct termui_optctx_dev_timer_opts, delay, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(TIMER_OPT_TDELAY | TIMER_OPT_MDELAY, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-r", "--resolution", TIMER_OPT_RES, struct termui_optctx_dev_timer_opts, delay, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(TIMER_OPT_RES, 0))

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_timer_group) =
{
  TERMUI_CON_ENTRY(dev_shell_timer_value, "value",
		   TERMUI_CON_OPTS_CTX(dev_timer_opts, TIMER_OPT_DEV, 0, timer_opts_cleanup)
                   )
  TERMUI_CON_ENTRY(dev_shell_timer_wait, "wait",
		   TERMUI_CON_OPTS_CTX(dev_timer_opts,
                                       TIMER_OPT_DEV | TIMER_OPT_TDELAY | TIMER_OPT_MDELAY,
                                       0, timer_opts_cleanup)
                   )

  TERMUI_CON_ENTRY(dev_shell_timer_config, "config",
		   TERMUI_CON_OPTS_CTX(dev_timer_opts,
                   TIMER_OPT_DEV, TIMER_OPT_RES, timer_opts_cleanup)
                   )

  TERMUI_CON_ENTRY(dev_shell_timer_convert, "convert",
		   TERMUI_CON_OPTS_CTX(dev_timer_opts,
                                       TIMER_OPT_DEV | TIMER_OPT_TDELAY | TIMER_OPT_MDELAY,
                                       0, timer_opts_cleanup)
                   )

  TERMUI_CON_LIST_END
};

