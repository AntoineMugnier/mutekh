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

    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014

*/

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include <device/shell.h>
#include <device/device.h>
#include <device/driver.h>

#include <device/class/pwm.h>

enum pwm_opt_e
{
  PWM_OPT_DEV  = 0x1,
  PWM_OPT_FREQ = 0x2,
  PWM_OPT_DUTY = 0x4,
  PWM_OPT_POL  = 0x8
};

struct termui_optctx_dev_pwm_opts
{
  /* pwm device. */
  struct device_pwm_s         pwm;

  union
  {
    /* FIXME: should add selectable option in libtermui. */
    struct {
      /* pwm frequency. */
      struct dev_freq_s       freq;

      /* pwm channel duty cycle. */
      struct dev_freq_ratio_s duty;

      /* pwm channel polarity. */
      enum dev_pwm_polarity_e pol;
    };
    struct dev_pwm_config_s   cfg;
  };

  uint_fast8_t                mask;
};

static
TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(pwm_opts_cleanup)
{
  struct termui_optctx_dev_pwm_opts *data = ctx;

  if (device_check_accessor(&data->pwm))
    device_put_accessor(&data->pwm);
}

static
TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_pwm_config)
{
  struct termui_optctx_dev_pwm_opts *data = ctx;

  data->mask = 0;

  if (used & PWM_OPT_FREQ)
    data->mask |= DEV_PWM_MASK_FREQ;

  if (used & PWM_OPT_DUTY)
    data->mask |= DEV_PWM_MASK_DUTY;

  if (used & PWM_OPT_POL)
    data->mask |= DEV_PWM_MASK_POL;

#if defined(CONFIG_MUTEK_SCHEDULER)
  error_t err = dev_pwm_wait_config(&data->pwm, &data->cfg, data->mask);
#else
  error_t err = dev_pwm_spin_config(&data->pwm, &data->cfg, data->mask);
#endif

  if (err)
    termui_con_printf(con, "error: failed to apply pwm configuration.\n");

  return err;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_pwm_start)
{
  struct termui_optctx_dev_pwm_opts *data = ctx;

  if (device_start(&data->pwm))
    return -EINVAL;
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_pwm_stop)
{
  struct termui_optctx_dev_pwm_opts *data = ctx;

  if (device_stop(&data->pwm))
    return -EINVAL;
  return 0;
}

static TERMUI_CON_OPT_DECL(dev_pwm_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--pwm-dev", PWM_OPT_DEV,
    struct termui_optctx_dev_pwm_opts, pwm, DRIVER_CLASS_PWM,
    TERMUI_CON_OPT_CONSTRAINTS(PWM_OPT_DEV, 0)
    TERMUI_CON_OPT_HELP("This option selects a pwm device", NULL)
  )

  TERMUI_CON_OPT_FREQ_ENTRY("-f", "--freq", PWM_OPT_FREQ,
    struct termui_optctx_dev_pwm_opts, freq,
    TERMUI_CON_OPT_CONSTRAINTS(PWM_OPT_FREQ, PWM_OPT_DEV)
    TERMUI_CON_OPT_HELP("This option defines the frequency of the pwm signal",
                        NULL)
  )

  TERMUI_CON_OPT_FREQ_RATIO_ENTRY("-c", "--duty", PWM_OPT_DUTY,
    struct termui_optctx_dev_pwm_opts, duty,
    TERMUI_CON_OPT_CONSTRAINTS(PWM_OPT_DUTY, PWM_OPT_DEV)
    TERMUI_CON_OPT_HELP(
        "This option defines the duty cycle (ratio) of the pwm signal",
        NULL)
  )

  TERMUI_CON_OPT_ENUM_ENTRY("-p", "--polarity", PWM_OPT_POL,
    struct termui_optctx_dev_pwm_opts, pol, dev_pwm_polarity_e,
    TERMUI_CON_OPT_CONSTRAINTS(PWM_OPT_POL, PWM_OPT_DEV)
    TERMUI_CON_OPT_HELP("This option defines the polarity of the pwm signal",
                        NULL)
  )

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_pwm_group) =
{
  TERMUI_CON_ENTRY(dev_shell_pwm_config, "config",
    TERMUI_CON_OPTS_CTX(dev_pwm_opts,
                        PWM_OPT_DEV,
                        PWM_OPT_FREQ | PWM_OPT_DUTY | PWM_OPT_POL,
                        pwm_opts_cleanup)
  )

  TERMUI_CON_ENTRY(dev_shell_pwm_start, "start",
    TERMUI_CON_OPTS_CTX(dev_pwm_opts, PWM_OPT_DEV, 0, pwm_opts_cleanup)
  )

  TERMUI_CON_ENTRY(dev_shell_pwm_stop, "stop",
    TERMUI_CON_OPTS_CTX(dev_pwm_opts, PWM_OPT_DEV, 0, pwm_opts_cleanup)
  )

  TERMUI_CON_LIST_END
};

