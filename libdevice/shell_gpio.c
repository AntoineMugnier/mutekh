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

#include <device/class/gpio.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/shell.h>

#include <mutek/console.h>
#include <mutek/shell.h>
#include <hexo/enum.h>

enum gpio_opts_e
{
  GPIO_OPT_DEV          = 0x01,
  GPIO_OPT_MODE         = 0x02,
  GPIO_OPT_IO           = 0x04,
  GPIO_OPT_IORANGE      = 0x08,
  GPIO_OPT_SET          = 0x10,
  GPIO_OPT_CLEAR        = 0x20,
  GPIO_OPT_TOGGLE       = 0x40,
  GPIO_OPT_MASK         = 0x80,
  GPIO_OPT_MASK_CLR     = 0x100,
};

struct termui_optctx_dev_gpio_opts
{
  struct device_gpio_s gpio;
  enum dev_pin_driving_e mode;
  gpio_id_t io[2];
  struct termui_con_string_s mask;
  struct termui_con_string_s mask_clr;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(gpio_opts_cleanup)
{
  struct termui_optctx_dev_gpio_opts *c = ctx;

  if (device_check_accessor(&c->gpio.base))
      device_put_accessor(&c->gpio.base);
}

static TERMUI_CON_COMMAND_PROTOTYPE(gpio_mode_cmd)
{
  struct termui_optctx_dev_gpio_opts *c = ctx;

  if (used & GPIO_OPT_IORANGE)
    {
      if (c->io[0] > c->io[1])
        return -EINVAL;
    }
  else
    {
      c->io[1] = c->io[0];
    }

  size_t l = (((c->io[1] - c->io[0]) | 7) + 1) / 8;
  uint8_t mask[l + 8];

  if (used & GPIO_OPT_MASK)
    {
      if (c->mask.len < l)
        return -EINVAL;
      memcpy(mask, c->mask.str, l);
    }
  else
    memset(mask, 0xff, l);

  struct dev_gpio_rq_s rq = {
    .type = DEV_GPIO_MODE,
    .io_first = c->io[0],
    .io_last = c->io[1],
  };

  rq.mode.mask = mask;
  rq.mode.mode = c->mode;

  error_t err = dev_gpio_wait_rq(&c->gpio, &rq);
  if (err)
    termui_con_printf(con, "request error %i", err);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(gpio_out_cmd)
{
  struct termui_optctx_dev_gpio_opts *c = ctx;

  if (used & GPIO_OPT_IORANGE)
    {
      if (c->io[0] > c->io[1])
        return -EINVAL;
    }
  else
    {
      c->io[1] = c->io[0];
    }

  size_t l = (((c->io[1] - c->io[0]) | 7) + 1) / 8;
  uint8_t mask_set[l + 8];
  uint8_t mask_clr[l + 8];

  if (used & GPIO_OPT_SET)
    {
      memset(mask_set, 0xff, l);
      memset(mask_clr, 0xff, l);
    }
  else if (used & GPIO_OPT_CLEAR)
    {
      memset(mask_set, 0x00, l);
      memset(mask_clr, 0x00, l);
    }
  else if (used & GPIO_OPT_TOGGLE)
    {
      memset(mask_set, 0xff, l);
      memset(mask_clr, 0x00, l);
    }
  else
    {
      if (c->mask.len < l)
        return -EINVAL;
      memcpy(mask_set, c->mask.str, c->mask.len);
      if (used & GPIO_OPT_MASK_CLR)
        {
          if (c->mask.len != c->mask_clr.len)
            return -EINVAL;
          memcpy(mask_clr, c->mask_clr.str, c->mask_clr.len);
        }
      else
        memcpy(mask_clr, mask_set, l);
    }

  struct dev_gpio_rq_s rq = {
    .type = DEV_GPIO_SET_OUTPUT,
    .io_first = c->io[0],
    .io_last = c->io[1],
  };

  rq.output.set_mask = mask_set;
  rq.output.clear_mask = mask_clr;

  error_t err = dev_gpio_wait_rq(&c->gpio, &rq);
  if (err)
    termui_con_printf(con, "request error %i", err);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(gpio_in_cmd)
{
  struct termui_optctx_dev_gpio_opts *c = ctx;

  if (used & GPIO_OPT_IORANGE)
    {
      if (c->io[0] > c->io[1])
        return -EINVAL;
    }
  else
    {
      c->io[1] = c->io[0];
    }

  size_t l = (((c->io[1] - c->io[0]) | 7) + 1) / 8;
  uint8_t d[l + 8];

  struct dev_gpio_rq_s rq = {
    .type = DEV_GPIO_GET_INPUT,
    .io_first = c->io[0],
    .io_last = c->io[1],
  };

  rq.input.data = d;

  error_t err = dev_gpio_wait_rq(&c->gpio, &rq);
  if (err)
    termui_con_printf(con, "request error %i", err);
  else
    termui_con_printf(con, "%P", d, l);

  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(gpio_until_cmd)
{
  struct termui_optctx_dev_gpio_opts *c = ctx;
  error_t err;

  if (used & GPIO_OPT_IORANGE)
    {
      if (c->io[0] > c->io[1])
        return -EINVAL;
    }
  else
    {
      c->io[1] = c->io[0];
    }

  size_t l = (c->io[1] - c->io[0] + 8) / 8;
  uint8_t d[l + 8];

  struct dev_gpio_rq_s rq = {
    .type = DEV_GPIO_GET_INPUT,
    .io_first = c->io[0],
    .io_last = c->io[1],
    .input.data = d,
  };

  err = dev_gpio_wait_rq(&c->gpio, &rq);
  if (err) {
    termui_con_printf(con, "READ request error %i\n", err);
    return err;
  }

  termui_con_printf(con, "Waiting %P...\n", d, l);

  rq.type = DEV_GPIO_UNTIL;
  rq.until.data = d;
  rq.until.mask = dev_gpio_mask1;

  err = dev_gpio_wait_rq(&c->gpio, &rq);
  if (err) {
    termui_con_printf(con, "UNTIL request error %i\n", err);
    return err;
  }

  rq.type = DEV_GPIO_GET_INPUT;
  rq.input.data = d;

  err = dev_gpio_wait_rq(&c->gpio, &rq);
  if (err) {
    termui_con_printf(con, "READ again request error %i\n", err);
    return err;
  }

  termui_con_printf(con, "Changed %P\n", d, l);

  return 0;
}


/* options descriptors array */
static TERMUI_CON_OPT_DECL(dev_gpio_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--gpio-dev", GPIO_OPT_DEV,
                                    struct termui_optctx_dev_gpio_opts, gpio, DRIVER_CLASS_GPIO,
                                    TERMUI_CON_OPT_CONSTRAINTS(GPIO_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_ENUM_ENTRY("-m", "--mode", GPIO_OPT_MODE, struct termui_optctx_dev_gpio_opts,
                            mode, dev_pin_driving_e,
                            TERMUI_CON_OPT_CONSTRAINTS(GPIO_OPT_MODE, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-i", "--io", GPIO_OPT_IO, struct termui_optctx_dev_gpio_opts, io[0], 1,
                               TERMUI_CON_OPT_CONSTRAINTS(GPIO_OPT_IO | GPIO_OPT_IORANGE, 0))

  TERMUI_CON_OPT_INTEGER_ENTRY("-r", "--io-range", GPIO_OPT_IORANGE, struct termui_optctx_dev_gpio_opts, io[0], 2,
                               TERMUI_CON_OPT_CONSTRAINTS(GPIO_OPT_IO | GPIO_OPT_IORANGE, 0))

  TERMUI_CON_OPT_ENTRY("-s", "--set", GPIO_OPT_SET,
                       TERMUI_CON_OPT_CONSTRAINTS(GPIO_OPT_MASK | GPIO_OPT_MASK_CLR |
                                                  GPIO_OPT_SET | GPIO_OPT_CLEAR | GPIO_OPT_TOGGLE, 0))

  TERMUI_CON_OPT_ENTRY("-c", "--clear", GPIO_OPT_CLEAR,
                       TERMUI_CON_OPT_CONSTRAINTS(GPIO_OPT_MASK | GPIO_OPT_MASK_CLR |
                                                  GPIO_OPT_SET | GPIO_OPT_CLEAR | GPIO_OPT_TOGGLE, 0))

  TERMUI_CON_OPT_ENTRY("-t", "--toggle", GPIO_OPT_TOGGLE,
                       TERMUI_CON_OPT_CONSTRAINTS(GPIO_OPT_MASK | GPIO_OPT_MASK_CLR |
                                                  GPIO_OPT_SET | GPIO_OPT_CLEAR | GPIO_OPT_TOGGLE, 0))

  TERMUI_CON_OPT_STRING_ENTRY("-M", "--mask", GPIO_OPT_MASK, struct termui_optctx_dev_gpio_opts, mask, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(GPIO_OPT_MASK |
                                                  GPIO_OPT_SET | GPIO_OPT_CLEAR | GPIO_OPT_TOGGLE, 0))

  TERMUI_CON_OPT_STRING_ENTRY("-C", "--mask-clr", GPIO_OPT_MASK_CLR, struct termui_optctx_dev_gpio_opts, mask_clr, 1,
                              TERMUI_CON_OPT_CONSTRAINTS(GPIO_OPT_MASK_CLR |
                                                         GPIO_OPT_SET | GPIO_OPT_CLEAR | GPIO_OPT_TOGGLE,
                                                         GPIO_OPT_MASK))

  TERMUI_CON_LIST_END
};

/* console commands descriptors array */
TERMUI_CON_GROUP_DECL(dev_shell_gpio_group) =
{
  TERMUI_CON_ENTRY(gpio_mode_cmd, "mode",
		   TERMUI_CON_OPTS_CTX(dev_gpio_opts,
                                       GPIO_OPT_DEV | GPIO_OPT_MODE | GPIO_OPT_IO | GPIO_OPT_IORANGE,
                                       GPIO_OPT_MASK, gpio_opts_cleanup)
		   )

  TERMUI_CON_ENTRY(gpio_in_cmd, "in",
		   TERMUI_CON_OPTS_CTX(dev_gpio_opts,
                                       GPIO_OPT_DEV | GPIO_OPT_IO | GPIO_OPT_IORANGE,
                                       0, gpio_opts_cleanup)
		   )

  TERMUI_CON_ENTRY(gpio_out_cmd, "out",
		   TERMUI_CON_OPTS_CTX(dev_gpio_opts,
                                       GPIO_OPT_DEV | GPIO_OPT_IO | GPIO_OPT_IORANGE | GPIO_OPT_TOGGLE |
                                       GPIO_OPT_SET | GPIO_OPT_CLEAR | GPIO_OPT_MASK, GPIO_OPT_MASK_CLR, gpio_opts_cleanup)
		   )

  TERMUI_CON_ENTRY(gpio_until_cmd, "until",
		   TERMUI_CON_OPTS_CTX(dev_gpio_opts,
                                       GPIO_OPT_DEV | GPIO_OPT_IO | GPIO_OPT_IORANGE,
                                       0, gpio_opts_cleanup)
		   )

  TERMUI_CON_LIST_END
};

