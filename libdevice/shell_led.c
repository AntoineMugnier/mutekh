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

    Copyright (c) 2021 Nicolas Pouillon <nipo@ssji.net>
*/

#include <device/valio/led.h>
#include <device/class/valio.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/shell.h>

#include <mutek/mem_alloc.h>
#include <mutek/console.h>
#include <mutek/shell.h>
#include <hexo/enum.h>

enum led_opts_e
{
  LED_OPT_DEV  = 0x01,
  LED_OPT_R = 0x02,
  LED_OPT_G = 0x04,
  LED_OPT_B  = 0x08,
};

struct termui_optctx_led_opts_s
{
  struct device_valio_s accessor;
  uint8_t r, g, b;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(led_opts_cleanup)
{
  struct termui_optctx_led_opts_s *c = ctx;

  if (device_check_accessor(&c->accessor.base))
    device_put_accessor(&c->accessor.base);
}

static TERMUI_CON_COMMAND_PROTOTYPE(shell_led_set)
{
  struct termui_optctx_led_opts_s *c = ctx;
  error_t err = 0;
  struct valio_led_luminosity_s v;
  v.lum[0] = c->r;
  v.lum[1] = c->g;
  v.lum[2] = c->b;

  err = dev_valio_wait_op(DEVICE_VALIO_WRITE, &c->accessor, VALIO_LED, &v);
  if (err)
    termui_con_printf(con, "Request failed with error: %d\n", err);

  return err;
}

static TERMUI_CON_OPT_DECL(led_opts_s) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--dev", LED_OPT_DEV,
                                    struct termui_optctx_led_opts_s, accessor, DRIVER_CLASS_VALIO,
                                    TERMUI_CON_OPT_CONSTRAINTS(LED_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-r", "--red",
                                     LED_OPT_R,
                                     struct termui_optctx_led_opts_s,
                                     r, 1, 0, 255,
                                     TERMUI_CON_OPT_CONSTRAINTS(LED_OPT_R, 0)
                                     )

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-g", "--green",
                                     LED_OPT_G,
                                     struct termui_optctx_led_opts_s,
                                     g, 1, 0, 255,
                                     TERMUI_CON_OPT_CONSTRAINTS(LED_OPT_G, 0)
                                     )

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-b", "--blue",
                                     LED_OPT_B,
                                     struct termui_optctx_led_opts_s,
                                     b, 1, 0, 255,
                                     TERMUI_CON_OPT_CONSTRAINTS(LED_OPT_B, 0)
                                     )

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_led_group) =
{
  TERMUI_CON_ENTRY(shell_led_set, "set",
    TERMUI_CON_OPTS_CTX(led_opts_s,
                        LED_OPT_DEV,
                        LED_OPT_R | LED_OPT_G | LED_OPT_B,
                        led_opts_cleanup)
  )

  TERMUI_CON_LIST_END
};
