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

#include <device/driver.h>
#include <device/resources.h>
#include <device/shell.h>
#include <device/class/valio.h>
#include <device/valio/motion_sensor.h>

#include <mutek/mem_alloc.h>
#include <mutek/console.h>
#include <mutek/shell.h>
#include <hexo/enum.h>

enum ms_opts_e
{
  MS_OPT_DEV         = 0x01,
  MS_OPT_THRESHOLD   = 0x02,
  MS_OPT_PERIOD      = 0x04,
  MS_OPT_WAKEUP_TIME = 0x08,
  MS_OPT_SLEEP_TIME  = 0x10,
};

struct termui_optctx_ms_opts_s
{
  struct device_valio_s accessor;
  struct valio_ms_config_s config;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(ms_opts_cleanup)
{
  struct termui_optctx_ms_opts_s *c = ctx;

  if (device_check_accessor(&c->accessor.base))
    device_put_accessor(&c->accessor.base);
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_ms_autocal)
{
  struct termui_optctx_ms_opts_s *c = ctx;
  struct valio_ms_state_s former = { .active = 0, };
  struct valio_ms_state_s latest = { .active = 0, };
  error_t err;

  while (latest.active || !former.active) {
    former = latest;

    err = dev_valio_wait_op(DEVICE_VALIO_WAIT_EVENT, &c->accessor, VALIO_MS_STATE, &latest);
    if (err) {
      termui_con_printf(con, "Wait with error: %d\n", err);
      return 0;
    }
  }

  err = dev_valio_wait_op(DEVICE_VALIO_READ, &c->accessor, VALIO_MS_CALIB, &latest.data);
  if (err) {
    termui_con_printf(con, "Calibration read failed with error: %d\n", err);
    return 0;
  }

  latest.data.axis[2] += 1000;

  for (size_t i = 0; i < 9; ++i)
    former.data.axis[i] = latest.data.axis[i] - former.data.axis[i];

  err = dev_valio_wait_op(DEVICE_VALIO_WRITE, &c->accessor, VALIO_MS_CALIB, &former.data);
  if (err) {
    termui_con_printf(con, "Calibration read failed with error: %d\n", err);
    return 0;
  }

  termui_con_printf(con, "Calibrated %d,%d,%dmg %d,%d,%ddps %d,%d,%duT\n",
         (int16_t)former.data.axis[0],
         (int16_t)former.data.axis[1],
         (int16_t)former.data.axis[2],
         (int16_t)former.data.axis[3] / 64,
         (int16_t)former.data.axis[4] / 64,
         (int16_t)former.data.axis[5] / 64,
         (int16_t)former.data.axis[6],
         (int16_t)former.data.axis[7],
         (int16_t)former.data.axis[8]);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_ms_read)
{
  struct termui_optctx_ms_opts_s *c = ctx;
  struct valio_ms_state_s state = {};
  error_t err;

  err = dev_valio_wait_op(DEVICE_VALIO_READ, &c->accessor, VALIO_MS_STATE, &state);
  if (err) {
    termui_con_printf(con, "Request failed with error: %d\n", err);
    return 0;
  }

  termui_con_printf(con, "Motion %s  %d,%d,%dmg %d,%d,ddps %d,%d,%duT\n",
         state.active ? "activ" : "sleep",
         (int16_t)state.data.axis[0],
         (int16_t)state.data.axis[1],
         (int16_t)state.data.axis[2],
         (int16_t)state.data.axis[3] / 64,
         (int16_t)state.data.axis[4] / 64,
         (int16_t)state.data.axis[5] / 64,
         (int16_t)state.data.axis[6],
         (int16_t)state.data.axis[7],
         (int16_t)state.data.axis[8]);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_ms_stream)
{
  struct termui_optctx_ms_opts_s *c = ctx;
  struct valio_ms_state_s state = {};
  error_t err;

  do {
    err = dev_valio_wait_op(DEVICE_VALIO_WAIT_EVENT, &c->accessor, VALIO_MS_STATE, &state);
    if (err) {
      termui_con_printf(con, "Request failed with error: %d\n", err);
      return 0;
    }

    termui_con_printf(con, "Motion %s  %d,%d,%dmg %d,%d,%ddps %d,%d,%duT\n",
           state.active ? "activ" : "sleep",
           (int16_t)state.data.axis[0],
           (int16_t)state.data.axis[1],
           (int16_t)state.data.axis[2],
           (int16_t)state.data.axis[3] / 64,
           (int16_t)state.data.axis[4] / 64,
           (int16_t)state.data.axis[5] / 64,
           (int16_t)state.data.axis[6],
           (int16_t)state.data.axis[7],
           (int16_t)state.data.axis[8]);
  } while (state.active);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_ms_configure)
{
  struct termui_optctx_ms_opts_s *c = ctx;
  struct valio_ms_config_s config;
  error_t err;

  err = dev_valio_wait_op(DEVICE_VALIO_READ, &c->accessor, VALIO_MS_CONFIG, &config);
  if (err) {
    termui_con_printf(con, "Configuration read failed with error: %d\n", err);
    return 0;
  }

  if (used & MS_OPT_THRESHOLD)
    config.threshold = c->config.threshold;
  if (used & MS_OPT_PERIOD)
    config.period = c->config.period;
  if (used & MS_OPT_WAKEUP_TIME)
    config.wakeup_time = c->config.wakeup_time;
  if (used & MS_OPT_SLEEP_TIME)
    config.sleep_time = c->config.sleep_time;

  err = dev_valio_wait_op(DEVICE_VALIO_WRITE, &c->accessor, VALIO_MS_CONFIG, &config);
  if (err) {
    termui_con_printf(con, "Calibration read failed with error: %d\n", err);
    return 0;
  }

  return 0;
}

static TERMUI_CON_OPT_DECL(ms_opts_s) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--dev", MS_OPT_DEV,
                                    struct termui_optctx_ms_opts_s, accessor, DRIVER_CLASS_VALIO,
                                    TERMUI_CON_OPT_CONSTRAINTS(MS_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_INTEGER_ENTRY("-m", "--motion-threshold", MS_OPT_THRESHOLD,
                               struct termui_optctx_ms_opts_s, config.threshold, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(MS_OPT_THRESHOLD, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-p", "--period", MS_OPT_PERIOD,
                               struct termui_optctx_ms_opts_s, config.period, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(MS_OPT_PERIOD, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-w", "--wakeup-time", MS_OPT_WAKEUP_TIME,
                               struct termui_optctx_ms_opts_s, config.wakeup_time, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(MS_OPT_WAKEUP_TIME, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-s", "--sleep-time", MS_OPT_SLEEP_TIME,
                               struct termui_optctx_ms_opts_s, config.sleep_time, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(MS_OPT_SLEEP_TIME, 0))

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_ms_group) =
{
  TERMUI_CON_ENTRY(shell_ms_read, "read",
    TERMUI_CON_OPTS_CTX(ms_opts_s,
                        MS_OPT_DEV,
                        MS_OPT_DEV,
                        ms_opts_cleanup)
  )

  TERMUI_CON_ENTRY(shell_ms_autocal, "autocal",
    TERMUI_CON_OPTS_CTX(ms_opts_s,
                        MS_OPT_DEV,
                        MS_OPT_DEV,
                        ms_opts_cleanup)
  )

  TERMUI_CON_ENTRY(shell_ms_stream, "stream",
    TERMUI_CON_OPTS_CTX(ms_opts_s,
                        MS_OPT_DEV,
                        MS_OPT_DEV,
                        ms_opts_cleanup)
  )

  TERMUI_CON_ENTRY(shell_ms_configure, "configure",
    TERMUI_CON_OPTS_CTX(ms_opts_s,
                        MS_OPT_DEV,
                        MS_OPT_DEV | MS_OPT_THRESHOLD | MS_OPT_PERIOD | MS_OPT_WAKEUP_TIME | MS_OPT_SLEEP_TIME,
                        ms_opts_cleanup)
  )

  TERMUI_CON_LIST_END
};
