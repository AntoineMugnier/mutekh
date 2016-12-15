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
    Copyright Vincent Defilippi <vincentdefilippi@gmail.com> (c) 2016

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

enum i2c_opt_e
{
  I2C_OPT_DEV       = 0x01,
  I2C_OPT_BIT_RATE  = 0x02,
  I2C_OPT_ADDR      = 0x04,
  I2C_OPT_READ      = 0x08,
  I2C_OPT_WRITE     = 0x10
};

struct termui_optctx_dev_i2c_opts
{
  /* i2c controller. */
  struct device_i2c_ctrl_s                ctrl;

  /* i2c slave address. */
  uint8_t                                 saddr;

  /* i2c transfers. */
  struct dev_i2c_ctrl_transaction_data_s  transfer[CONFIG_DEVICE_SHELL_I2C_TRANSFER_MAX];

  /* i2c transfer count. */
  size_t                                  transfer_count;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(i2c_opts_cleanup)
{
  struct termui_optctx_dev_i2c_opts *data = ctx;

  uint32_t ti;

  if (device_check_accessor(&data->ctrl.base))
    device_put_accessor(&data->ctrl.base);

  for (ti = 0; ti < data->transfer_count; ++ti)
    {
      if (data->transfer[ti].type == DEV_I2C_CTRL_TRANSACTION_READ)
        mem_free(data->transfer[ti].data);
    }
}

static TERMUI_CON_PARSE_OPT_PROTOTYPE(dev_shell_i2c_parse_transfer)
{
  struct termui_optctx_dev_i2c_opts *data = ctx;

  struct dev_i2c_ctrl_transaction_data_s *tr = &data->transfer[data->transfer_count];

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
      tr->type = DEV_I2C_CTRL_TRANSACTION_READ;
    }
  else
    {
      tr->data = (uint8_t *) argv[0];
      tr->size = argl[0];
      tr->type = DEV_I2C_CTRL_TRANSACTION_WRITE;
    }

  ++data->transfer_count;
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_i2c_scan)
{
  struct termui_optctx_dev_i2c_opts *data = ctx;

  uint8_t tr_data[] = {0xAA};

  struct dev_i2c_ctrl_transaction_data_s tr = {
    .data = tr_data,
    .size = sizeof(tr_data),
    .type = DEV_I2C_CTRL_TRANSACTION_WRITE,
  };

  struct dev_i2c_ctrl_transaction_rq_s rq;
  dev_i2c_transaction_init(&rq);
  rq.transfer = &tr;
  rq.transfer_count = 1;

  uint8_t count = 0;
  for (uint8_t saddr = 0; saddr < 128; ++saddr)
    {
      rq.base.saddr = saddr;
#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
      dev_i2c_wait_transaction(&data->ctrl, &rq);
#else
      dev_i2c_spin_transaction(&data->ctrl, &rq);
#endif

      if (rq.base.err != -EHOSTUNREACH)
        {
          termui_con_printf(con, "found slave @ 0x%02x\n", saddr);
          count++;
        }
    }

  termui_con_printf(con, "\ni2c scan found %u slaves", count);
  return 0;
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_i2c_io)
{
  struct termui_optctx_dev_i2c_opts *data = ctx;

  struct dev_i2c_ctrl_transaction_rq_s rq;
  dev_i2c_transaction_init(&rq);
  rq.base.saddr = data->saddr;
  rq.transfer = data->transfer;
  rq.transfer_count = data->transfer_count;

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)
  dev_i2c_wait_transaction(&data->ctrl, &rq);
#else
  dev_i2c_spin_transaction(&data->ctrl, &rq);
#endif

  switch (rq.base.err)
    {
      case -EBUSY:
        termui_con_printf(con, "Error: I2C controller busy.\n");
        return rq.base.err;

      case -EHOSTUNREACH:
        termui_con_printf(con, "Error: Invalid slave address.\n");
        return rq.base.err;

      case -EIO:
        termui_con_printf(con, "Error: Unexpected I/O error.\n");
        return rq.base.err;

      case -ENOTSUP:
        termui_con_printf(con, "Error: Operation not supported.\n");
        return rq.base.err;

      case -EAGAIN:
      default:
        break;
    }

  bool_t restart = 0;
  termui_con_printf(con, "START\n");
  for (uint8_t i = 0; i < rq.transfer_count; i++)
    {
      struct dev_i2c_ctrl_transaction_data_s *tr = &data->transfer[i];

      if (restart)
        termui_con_printf(con, "RESTART\n");

      if (tr->type == DEV_I2C_CTRL_TRANSACTION_WRITE &&
          rq.base.err == -EAGAIN &&
          i == rq.transfer_index)
        {
          termui_con_printf(con, "Error: NACK received\n");
          termui_con_printf(con, "STOP\n");
          return rq.base.err;
        }
      else
        termui_con_printf(con, "%s (%u/%u bytes): %P\n",
          (tr->type == DEV_I2C_CTRL_TRANSACTION_READ ? "read " : "write"),
          tr->size, tr->size, tr->data, tr->size);

      restart = 0;
      if (i + 1 < rq.transfer_count && tr->type != data->transfer[i + 1].type)
        restart = 1;
    }
    termui_con_printf(con, "STOP\n");

  return rq.base.err;
}

static TERMUI_CON_OPT_DECL(dev_i2c_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--i2c-dev", I2C_OPT_DEV,
    struct termui_optctx_dev_i2c_opts, ctrl, DRIVER_CLASS_I2C_CTRL,
    TERMUI_CON_OPT_CONSTRAINTS(I2C_OPT_DEV, 0)
    TERMUI_CON_OPT_HELP("This option selects an i2c device", NULL)
  )

  TERMUI_CON_OPT_INTEGER_RANGE_ENTRY("-a", "--addr", I2C_OPT_ADDR,
    struct termui_optctx_dev_i2c_opts, saddr, 1, 0 /* min */, 127 /* max */,
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

