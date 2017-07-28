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

    Copyright Sebastien Cerdan <sebcerdan@gmail.com> (c) 2014

*/

#include <limits.h>
#include <stdlib.h>
#include <string.h>

#include <device/shell.h>
#include <device/device.h>
#include <device/driver.h>

#include <device/class/spi.h>
#include <device/class/gpio.h>

#include <termui/console_opt.h>

enum spi_opt_e
{
  SPI_OPT_DEV       = 0x01,
  SPI_OPT_BIT_RATE  = 0x02,
  SPI_OPT_BIT_ORDER = 0x04,
  SPI_OPT_CLK_MODE  = 0x08,
  SPI_OPT_POLARITY  = 0x10,
  SPI_OPT_CS_ID     = 0x20,
  SPI_OPT_CS_OP     = 0x40,
  SPI_OPT_WR_DATA   = 0x80,
  SPI_OPT_SIZE      = 0x100,
  SPI_OPT_WIDTH     = 0x200,
  SPI_OPT_DEV_GPIO  = 0x400,
};

struct termui_optctx_dev_spi_opts
{
  /* spi device. */
  struct device_spi_ctrl_s spi;
#ifdef CONFIG_DEVICE_GPIO
  struct device_gpio_s gpio;
#endif
  enum dev_spi_polarity_e polarity;
  uint8_t csid;
  enum dev_spi_cs_op_e csop;

  union {
    /* Transfer */
    struct {
      struct shell_opt_buffer_s data;
      size_t count;
    };

    /* Config */
    struct {
      uint_fast8_t width;
      uint32_t bit_rate;
      enum dev_spi_ckmode_e ck_mode;
      enum dev_spi_bit_order_e bit_order;
    };
  };
};

/****************************** config ************************/

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(spi_opts_cleanup)
{
  struct termui_optctx_dev_spi_opts *c = ctx;

  if (c->data.buffered)
    shell_buffer_drop(c->data.addr);

  if (device_check_accessor(&c->spi.base))
    device_put_accessor(&c->spi.base);

#ifdef CONFIG_DEVICE_GPIO
  if (device_check_accessor(&c->gpio.base))
    device_put_accessor(&c->gpio.base);
#endif
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_spi_config)
{
  struct termui_optctx_dev_spi_opts *c = ctx;

  struct dev_spi_ctrl_config_s cfg = {
    .ck_mode = DEV_SPI_CK_MODE_0,
    .bit_order = DEV_SPI_MSB_FIRST,
    .miso_pol = DEV_SPI_ACTIVE_HIGH,
    .mosi_pol = DEV_SPI_ACTIVE_HIGH,
    .bit_rate1k = 100,
    .word_width = 8
  };

  if (used & SPI_OPT_CLK_MODE)
    cfg.ck_mode = c->ck_mode;

  if (used & SPI_OPT_BIT_ORDER)
    cfg.bit_order = c->bit_order;

  if (used & SPI_OPT_POLARITY)
    cfg.miso_pol = cfg.mosi_pol = c->polarity;

  if (used & SPI_OPT_BIT_RATE)
    cfg.bit_rate1k = c->bit_rate >> 10;

  if (used & SPI_OPT_WIDTH)
    cfg.word_width = c->width;

  if (DEVICE_OP(&c->spi, config, &cfg))
    return -EINVAL;

  return 0;
}

#ifdef CONFIG_MUTEK_CONTEXT_SCHED
static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_spi_transfer)
{
  struct termui_optctx_dev_spi_opts *c = ctx;
  struct dev_spi_ctrl_transfer_s tr;
  size_t count;
  error_t err;

  if (used & SPI_OPT_WR_DATA)
    {
      tr.data.out_width = 1;
      tr.data.out = (const uint8_t*)c->data.addr;
      tr.data.count = count = c->data.size;
    }
  else
    {
      tr.data.out_width = 0;
      tr.data.out = (const uint8_t*)"\xff";
      tr.data.count = count = c->count;
    }

  if (!tr.data.count)
    return 0;

  void *in = shell_buffer_new(con, tr.data.count, "spi", NULL, 0);
  if (!in)
    return -EINVAL;

  tr.data.in = in;
  tr.data.in_width = 1;
  tr.cs_op = DEV_SPI_CS_NOP_NOP;

  if (!(used & SPI_OPT_POLARITY))
    c->polarity = DEV_SPI_ACTIVE_LOW;

# ifdef CONFIG_DEVICE_GPIO
  if (used & SPI_OPT_DEV_GPIO)
    dev_gpio_out(&c->gpio, c->csid, c->polarity);
# endif

  if (used & SPI_OPT_CS_OP)
    {
      tr.cs_op = c->csop;
      tr.cs_cfg.id = c->csid;
      tr.cs_cfg.polarity = c->polarity;
    }

  err = dev_spi_wait_transfer(&c->spi, &tr);

# ifdef CONFIG_DEVICE_GPIO
  if (used & SPI_OPT_DEV_GPIO)
    dev_gpio_out(&c->gpio, c->csid, c->polarity ^ 1);
# endif

  if (err)
    termui_con_printf(con, "error %i\n", err);
  else
    shell_buffer_advertise(con, in, count);

  shell_buffer_drop(in);
  return err ? -EINVAL : 0;
}

# ifdef CONFIG_DEVICE_SPI_TRANSACTION
static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_spi_transaction)
{
  struct termui_optctx_dev_spi_opts *c = ctx;
  struct dev_spi_ctrl_transaction_rq_s tr;
  size_t count;
  error_t err;

  dev_spi_transaction_init(&tr);
# ifdef CONFIG_DEVICE_GPIO
  device_init_accessor(&tr.base.gpio.base);
# endif

  if (used & SPI_OPT_WR_DATA)
    {
      tr.data.out_width = 1;
      tr.data.out = (const uint8_t*)c->data.addr;
      tr.data.count = count = c->data.size;
    }
  else
    {
      tr.data.out_width = 0;
      tr.data.out = (const uint8_t*)"\xff";
      tr.data.count = count = c->count;
    }

  if (!tr.data.count)
    return 0;

  void *in = shell_buffer_new(con, tr.data.count, "spi", NULL, 0);
  if (!in)
    return -EINVAL;

  tr.data.in = in;
  tr.data.in_width = 1;

  tr.base.cs_cfg.id = c->csid;
  tr.base.cs_cfg.polarity = (used & SPI_OPT_POLARITY)
    ? c->polarity : DEV_SPI_ACTIVE_LOW;

  if (used & SPI_OPT_CS_ID)
    tr.base.cs_ctrl = 1;

# ifdef CONFIG_DEVICE_GPIO
  if (used & SPI_OPT_DEV_GPIO)
    {
      err = device_copy_accessor(&tr.base.gpio.base, &c->gpio.base);
      if (err)
        goto err;
      tr.base.cs_gpio = 1;
    }
# endif

  err = dev_spi_wait_transaction(&c->spi, &tr);

# ifdef CONFIG_DEVICE_GPIO
  if (used & SPI_OPT_DEV_GPIO)
    device_put_accessor(&tr.base.gpio.base);
# endif

  if (err)
    termui_con_printf(con, "error %i\n", err);
  else
    shell_buffer_advertise(con, in, count);

 err:
  shell_buffer_drop(in);
  return err ? -EINVAL : 0;
}
# endif

#endif

static TERMUI_CON_OPT_DECL(dev_spi_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--spi-dev", SPI_OPT_DEV,
    struct termui_optctx_dev_spi_opts, spi, DRIVER_CLASS_SPI_CTRL,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_DEV, 0)
    TERMUI_CON_OPT_HELP("selects an spi device", NULL)
  )

  TERMUI_CON_OPT_INTEGER_ENTRY("-i", "--cs-id", SPI_OPT_CS_ID,
    struct termui_optctx_dev_spi_opts, csid, 1,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_CS_ID, 0)
    TERMUI_CON_OPT_HELP("Specifies the id of ths CS signal",
                        NULL))

  TERMUI_CON_OPT_ENUM_ENTRY("-c", "--cs-op", SPI_OPT_CS_OP,
    struct termui_optctx_dev_spi_opts, csop, dev_spi_cs_op_e,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_DEV_GPIO | SPI_OPT_CS_OP, SPI_OPT_CS_ID)
    TERMUI_CON_OPT_HELP("Use the SPI controller to drive the CS signal",
                        NULL))

  TERMUI_CON_OPT_INTEGER_ENTRY("-r", "--bit-rate", SPI_OPT_BIT_RATE,
    struct termui_optctx_dev_spi_opts, bit_rate, 1,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_BIT_RATE, 0))

  TERMUI_CON_OPT_ENUM_ENTRY("-o", "--bit-order", SPI_OPT_BIT_ORDER,
    struct termui_optctx_dev_spi_opts, bit_order, dev_spi_bit_order_e,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_BIT_ORDER, 0))

  TERMUI_CON_OPT_ENUM_ENTRY("-p", "--polarity", SPI_OPT_POLARITY,
    struct termui_optctx_dev_spi_opts, polarity, dev_spi_polarity_e,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_POLARITY, SPI_OPT_CS_ID)
    TERMUI_CON_OPT_HELP("Defines the polarity of cs/mosi/miso signals",
                        NULL)
  )

  TERMUI_CON_OPT_ENUM_ENTRY("-m", "--clock-mode", SPI_OPT_CLK_MODE,
    struct termui_optctx_dev_spi_opts, ck_mode, dev_spi_ckmode_e,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_CLK_MODE, 0))

  TERMUI_CON_OPT_SHELL_BUFFER_RAW_ENTRY("-D", "--data", SPI_OPT_WR_DATA,
    struct termui_optctx_dev_spi_opts, data, NULL,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_SIZE | SPI_OPT_WR_DATA, 0)
    TERMUI_CON_OPT_HELP("Specifies the content of a write transfer",
                        NULL)
  )

  TERMUI_CON_OPT_INTEGER_ENTRY("-s", "--size", SPI_OPT_SIZE,
    struct termui_optctx_dev_spi_opts, count, 1,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_SIZE | SPI_OPT_WR_DATA, 0)
    TERMUI_CON_OPT_HELP("Defines the size of a read transfer",
                        NULL)
  )

  TERMUI_CON_OPT_INTEGER_ENTRY("-w", "--word-width", SPI_OPT_WIDTH,
    struct termui_optctx_dev_spi_opts, width, 1,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_WIDTH, 0))

#ifdef CONFIG_DEVICE_GPIO
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-g", "--gpio-cs-dev", SPI_OPT_DEV_GPIO,
    struct termui_optctx_dev_spi_opts, gpio, DRIVER_CLASS_GPIO,
    TERMUI_CON_OPT_CONSTRAINTS(SPI_OPT_DEV_GPIO | SPI_OPT_CS_OP, SPI_OPT_CS_ID)
    TERMUI_CON_OPT_HELP("Use a GPIO pin to drive the CS signal",
                        NULL)
  )
#endif

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_spi_ctrl_group) =
{
  TERMUI_CON_ENTRY(dev_shell_spi_config, "config",
    TERMUI_CON_OPTS_CTX(dev_spi_opts,
                        SPI_OPT_DEV,
                        SPI_OPT_BIT_RATE | SPI_OPT_BIT_ORDER |
                        SPI_OPT_POLARITY | SPI_OPT_CLK_MODE | SPI_OPT_WIDTH,
                        spi_opts_cleanup)
  )

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
  TERMUI_CON_ENTRY(dev_shell_spi_transfer, "transfer",
    TERMUI_CON_OPTS_CTX(dev_spi_opts,
                        SPI_OPT_DEV | SPI_OPT_WR_DATA |
                        SPI_OPT_SIZE,
# ifdef CONFIG_DEVICE_GPIO
                        SPI_OPT_DEV_GPIO |
# endif
                        SPI_OPT_CS_ID | SPI_OPT_CS_OP | SPI_OPT_POLARITY,
                        spi_opts_cleanup)
  )

# ifdef CONFIG_DEVICE_SPI_TRANSACTION
  TERMUI_CON_ENTRY(dev_shell_spi_transaction, "transaction",
    TERMUI_CON_OPTS_CTX(dev_spi_opts,
                        SPI_OPT_DEV | SPI_OPT_WR_DATA |
                        SPI_OPT_SIZE,
#  ifdef CONFIG_DEVICE_GPIO
                        SPI_OPT_DEV_GPIO |
#  endif
                        SPI_OPT_CS_ID | SPI_OPT_POLARITY,
                        spi_opts_cleanup)
  )
# endif
#endif

  TERMUI_CON_LIST_END
};

