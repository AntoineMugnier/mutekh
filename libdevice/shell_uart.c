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

  Copyright (c) Nicolas Pouillon <nipo@ssji.net>, 2016
*/

#include <device/shell.h>
#include <device/device.h>
#include <device/driver.h>
#include <inttypes.h>

#include <device/class/uart.h>

enum mem_opts_e
{
  UART_OPT_DEV      = 0x01,
  UART_OPT_BAUDRATE = 0x02,
  UART_OPT_BITS     = 0x04,
  UART_OPT_STOP     = 0x08,
  UART_OPT_PARITY   = 0x10,
  UART_OPT_FLOW     = 0x20,
};

struct termui_optctx_dev_uart_opts
{
  struct device_uart_s uart;
  uint32_t baudrate;
  uint32_t bits;
  enum dev_uart_parity_e parity;
  uint32_t stop;
};

static TERMUI_CON_ARGS_CLEANUP_PROTOTYPE(uart_opts_cleanup)
{
  struct termui_optctx_dev_uart_opts *c = ctx;

  if (device_check_accessor(&c->uart.base))
      device_put_accessor(&c->uart.base);
}

static TERMUI_CON_COMMAND_PROTOTYPE(dev_shell_uart_config)
{
  struct termui_optctx_dev_uart_opts *c = ctx;
  struct dev_uart_config_s config = {
    .baudrate = c->baudrate,
    .data_bits = used & UART_OPT_BITS ? c->bits : 8,
    .stop_bits = used & UART_OPT_STOP ? c->stop : 1,
    .flow_ctrl = !!(used & UART_OPT_FLOW),
    .half_duplex = 0,
    .parity = used & UART_OPT_PARITY ? c->parity : DEV_UART_PARITY_NONE,
  };

  return dev_uart_config(&c->uart, &config);
}

static TERMUI_CON_OPT_DECL(dev_uart_opts) =
{
  TERMUI_CON_OPT_DEV_ACCESSOR_ENTRY("-d", "--uart", UART_OPT_DEV,
                                    struct termui_optctx_dev_uart_opts, uart, DRIVER_CLASS_UART,
                                    TERMUI_CON_OPT_CONSTRAINTS(UART_OPT_DEV, 0)
                                    )

  TERMUI_CON_OPT_INTEGER_ENTRY("-B", "--baudrate", UART_OPT_BAUDRATE,
                               struct termui_optctx_dev_uart_opts, baudrate, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(UART_OPT_BAUDRATE, 0)
                               )

  TERMUI_CON_OPT_INTEGER_ENTRY("-b", "--bits", UART_OPT_BITS,
                               struct termui_optctx_dev_uart_opts, bits, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(UART_OPT_BITS, 0)
                               )

  TERMUI_CON_OPT_INTEGER_ENTRY("-s", "--stop", UART_OPT_STOP,
                               struct termui_optctx_dev_uart_opts, stop, 1,
                               TERMUI_CON_OPT_CONSTRAINTS(UART_OPT_STOP, 0)
                               )

  TERMUI_CON_OPT_ENUM_ENTRY("-p", "--parity", UART_OPT_PARITY,
                            struct termui_optctx_dev_uart_opts, parity,
                            dev_uart_parity_e,
                            TERMUI_CON_OPT_CONSTRAINTS(UART_OPT_PARITY, 0)
                            )

  TERMUI_CON_OPT_ENTRY("-f", "--flow-ctrl", UART_OPT_FLOW,
                       TERMUI_CON_OPT_CONSTRAINTS(UART_OPT_FLOW, 0)
                       )

  TERMUI_CON_LIST_END
};

TERMUI_CON_GROUP_DECL(dev_shell_uart_group) =
{
  TERMUI_CON_ENTRY(dev_shell_uart_config, "config",
		   TERMUI_CON_OPTS_CTX(dev_uart_opts,
                                       UART_OPT_DEV | UART_OPT_BAUDRATE,
                                       UART_OPT_FLOW | UART_OPT_BITS
                                       | UART_OPT_STOP | UART_OPT_PARITY,
                                       uart_opts_cleanup)
                   )

  TERMUI_CON_LIST_END
};

