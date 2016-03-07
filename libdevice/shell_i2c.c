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

#include <mutek/mem_alloc.h>

#include <device/request.h>
#include <device/shell.h>
#include <device/device.h>
#include <device/driver.h>

#include <device/class/i2c.h>

#include <termui/console_opt.h>

enum pwm_opt_e
{
  I2C_OPT_DEV       = 0x01,
  I2C_OPT_BIT_RATE  = 0x02,
  I2C_OPT_ADDR      = 0x04,
  I2C_OPT_READ      = 0x08,
  I2C_OPT_WRITE     = 0x10
};

/****************************** config ************************/

struct termui_optctx_dev_i2c_config_opts
{
  /* i2c device. */
  struct device_i2c_s       i2c;

  /* i2c bit rate. */
  union
  {
      struct dev_i2c_config_s cfg;
      uint32_t                bit_rate;
  };
};

struct termui_optctx_dev_i2c_opts
{
  /* i2c device. */
  struct device_i2c_s       i2c;

  /* i2c slave address. */
  uint8_t                   saddr;

  /* i2c transfers. */
  struct dev_i2c_transfer_s transfer[CONFIG_DEVICE_SHELL_I2C_TRANSFER_MAX];

  /* i2c transfer count. */
  size_t                    transfer_count;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(i2c_config_opts_cleanup)
{
  struct termui_optctx_dev_i2c_config_opts *data = ctx;

  if (device_check_accessor(&data->i2c))
    device_put_accessor(&data->i2c);
}

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(i2c_opts_cleanup)
{
  struct termui_optctx_dev_i2c_opts *data = ctx;

  uint32_t ti;

  if (device_check_accessor(&data->i2c))
    device_put_accessor(&data->i2c);

  for (ti = 0; ti < data->transfer_count; ++ti)
    {
      if (data->transfer[ti].type == DEV_I2C_READ)
        mem_free(data->transfer[ti].data);
    }
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_i2c_config)
{
  struct termui_optctx_dev_i2c_config_opts *data = ctx;

  error_t err = dev_i2c_config(&data->i2c, &data->cfg);

  if (err)
    termui_con_printf(con, "error: failed to apply i2c configuration.\n");

  return err;
}

static TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_shell_i2c_parse_transfer)
{
  struct termui_optctx_dev_i2c_opts *data = ctx;

  struct dev_i2c_transfer_s *tr = &data->transfer[data->transfer_count];

  if (data->transfer_count >= CONFIG_DEVICE_SHELL_I2C_TRANSFER_MAX)
    {
      termui_con_printf(con, "error: the maximum number of transfer is reached.");
      return -ENOMEM;
    }

  if (opt->id & I2C_OPT_READ)
    {
      uint32_t size = strtoul(argv[0], 0, 0);
      if (size == 0 || size == ULONG_MAX || size > ((1<<16)-1))
        {
          termui_con_printf(con, "error: invalid read byte count.");
          return -EINVAL;
        }

      tr->data = mem_alloc(size, (mem_scope_sys));
      if (tr->data == NULL)
        {
          termui_con_printf(con, "error: cannot allocate buffer for read transfer.");
          return -ENOMEM;
        }
      tr->size = size;
      tr->type = DEV_I2C_READ;
    }
  else
    {
      tr->data = (uint8_t *) argv[0];
      tr->size = argl[0];
      tr->type = DEV_I2C_WRITE;
    }

  ++data->transfer_count;
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_i2c_scan)
{
  struct termui_optctx_dev_i2c_opts *data = ctx;

  struct dev_i2c_transfer_s tr = {
    .type = DEV_I2C_WRITE,
    .data = NULL,
    .size = 0
  };

  uint_fast8_t saddr, count = 0;

  for (saddr = 0; saddr < 0x7f; ++saddr)
    {
#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
      error_t err = dev_i2c_wait_request(&data->i2c, saddr, &tr, 1);
#else
      error_t err = dev_i2c_spin_request(&data->i2c, saddr, &tr, 1);
#endif
      if (!err)
        {
          ++count;
          termui_con_printf(con, "found slave at address 0x%02x\n", saddr);
        }
    }

  termui_con_printf(con, "\ni2c scan found %u slaves", count);
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_i2c_io)
{
  struct termui_optctx_dev_i2c_opts *data = ctx;

  struct dev_request_status_s status;
  struct dev_i2c_rq_s         req =
    {
      .saddr          = data->saddr,
      .transfer       = data->transfer,
      .transfer_count = data->transfer_count
    };

  size_t count;
  struct dev_i2c_transfer_s *tr = &data->transfer[0];

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
  dev_request_sched_init(&req.base, &status);
#else
  dev_request_spin_init(&req.base, &status);
#endif

  DEVICE_OP(&data->i2c, request, &req);

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
  dev_request_sched_wait(&status);
#else
  dev_request_spin_wait(&status);
#endif

  count = req.error ? req.error_transfer : req.transfer_count;
  while (count-- > 0)
    {
      if (tr->type == DEV_I2C_READ)
          termui_con_printf(con, "read  (%u bytes): %P", tr->size, tr->data,
                            tr->size);
      else
        termui_con_printf(con, "write (%u bytes): done", tr->size);

      if (count > 0)
        termui_con_printf(con, "\n");

      ++tr;
    }

  if (req.error)
  {
    termui_con_printf(con, "error: cannot perform remaining i2c transfers");
    termui_con_printf(con, "\n  %s transfer at index %u (%u bytes) failed (%s)",
                      (tr->type == DEV_I2C_READ ? "read" : "write"),
                      req.error_transfer, tr->size,
                      strerror(req.error));
  }

  return req.error;
}

static TERMUI_CON_OPT_DECL(dev_i2c_config_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--i2c-dev", I2C_OPT_DEV,
    struct termui_optctx_dev_i2c_config_opts, i2c, DRIVER_CLASS_I2C,
    TERMUI_CON_OPT_CONSTRAINTS(I2C_OPT_DEV, 0)
    TERMUI_CON_OPT_HELP("This option selects an i2c device", NULL)
  )

  TERMUI_CON_OPT_INTEGER_ENTRY("-b", "--bit-rate", I2C_OPT_BIT_RATE,
    struct termui_optctx_dev_i2c_config_opts, bit_rate, 1,
    TERMUI_CON_OPT_CONSTRAINTS(I2C_OPT_BIT_RATE, I2C_OPT_DEV)
    TERMUI_CON_OPT_HELP("This option defines the bit rate on the i2c bus",
                        NULL)
  )

  TERMUI_CON_LIST_END
};

static TERMUI_CON_OPT_DECL(dev_i2c_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--i2c-dev", I2C_OPT_DEV,
    struct termui_optctx_dev_i2c_opts, i2c, DRIVER_CLASS_I2C,
    TERMUI_CON_OPT_CONSTRAINTS(I2C_OPT_DEV, 0)
    TERMUI_CON_OPT_HELP("This option selects an i2c device", NULL)
  )

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-a", "--addr", I2C_OPT_ADDR,
    struct termui_optctx_dev_i2c_opts, saddr, 1, 1 /* min */, 128 /* max */,
    TERMUI_CON_OPT_CONSTRAINTS(I2C_OPT_ADDR, I2C_OPT_DEV)
    TERMUI_CON_OPT_HELP("This option defines the i2c slave address",
                        NULL)
  )

  TERMUI_CON_OPT_ENTRY("-r", "--read", I2C_OPT_READ,
    TERMUI_CON_OPT_PARSE(dev_shell_i2c_parse_transfer, 1)
    TERMUI_CON_OPT_HELP("This option reads n bytes from the slave",
                        NULL)
  )

  TERMUI_CON_OPT_ENTRY("-w", "--write", I2C_OPT_WRITE,
    TERMUI_CON_OPT_PARSE(dev_shell_i2c_parse_transfer, 1)
    TERMUI_CON_OPT_HELP("This option write the given buffer to the slave",
                        NULL)
  )

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_i2c_group) =
{
  TERMUI_CON_ENTRY(dev_shell_i2c_config, "config",
    TERMUI_CON_OPTS_CTX(dev_i2c_config_opts,
                        I2C_OPT_DEV | I2C_OPT_BIT_RATE,
                        0,
                        i2c_config_opts_cleanup)
  )

  TERMUI_CON_ENTRY(dev_shell_i2c_scan, "scan",
    TERMUI_CON_OPTS_CTX(dev_i2c_opts,
                        I2C_OPT_DEV,
                        0,
                        i2c_opts_cleanup)
  )

  TERMUI_CON_ENTRY(dev_shell_i2c_io, "transfer",
    TERMUI_CON_OPTS_CTX(dev_i2c_opts,
                        I2C_OPT_DEV | I2C_OPT_ADDR,
                        I2C_OPT_READ | I2C_OPT_WRITE,
                        i2c_opts_cleanup)
  )

  TERMUI_CON_LIST_END
};

