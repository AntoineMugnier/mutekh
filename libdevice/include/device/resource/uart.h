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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2018
    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014
*/

#ifndef __DEVICE_RESOURCE_UART_H__
#define __DEVICE_RESOURCE_UART_H__

#include <hexo/types.h>
#include <device/device.h>
#include <device/resources.h>
#include <hexo/enum.h>

ENUM_DESCRIPTOR(dev_uart_parity_e, strip:DEV_UART_PARITY_, upper);

/** UART parity. */
enum dev_uart_parity_e
{
  DEV_UART_PARITY_NONE,
  DEV_UART_PARITY_ODD,
  DEV_UART_PARITY_EVEN,
};

/** @This structure describes the configuration of a UART device. */
struct dev_uart_config_s
{
  /** baud rate in bps. */
  uint32_t BITFIELD(baudrate,22);

  /** data bits. */
  uint32_t BITFIELD(data_bits,5);

  /** stop bits. */
  uint32_t BITFIELD(stop_bits,2);

  /** flow control. */
  uint32_t BITFIELD(flow_ctrl,1);

  /** parity. */
  enum dev_uart_parity_e BITFIELD(parity,2);
};

/** @This appends a UART resource entry to the device tree.
    @csee DEV_RES_UART */
config_depend_alwaysinline(CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_uart(struct device_s           *dev,
                            const struct dev_uart_config_s *cfg),
{
  struct dev_resource_s *r;

  error_t err = device_res_alloc(dev, &r, DEV_RES_UART);
  if (err)
    return err;

  r->u.uart.baudrate    = cfg->baudrate;
  r->u.uart.data_bits   = cfg->data_bits;
  r->u.uart.stop_bits   = cfg->stop_bits;
  r->u.uart.parity      = cfg->parity;
  r->u.uart.flow_ctrl   = cfg->flow_ctrl;

  return 0;
})

ALWAYS_INLINE error_t device_get_res_uart(const struct device_s *dev,
                                          struct dev_uart_config_s *cfg)
{
  struct dev_resource_s *r;

  r = device_res_get(dev, DEV_RES_UART, 0);
  if (r == NULL)
    return -ENOENT;

  cfg->baudrate  = r->u.uart.baudrate;
  cfg->data_bits = r->u.uart.data_bits;
  cfg->stop_bits = r->u.uart.stop_bits;
  cfg->parity    = r->u.uart.parity;
  cfg->flow_ctrl = r->u.uart.flow_ctrl;

  return 0;
}

/** @This specifies an UART configuration resource entry in a static
    device resources table declaration. @csee DEV_RES_UART
    @see #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_UART(                               \
    __baudrate, __data, __parity, __stop, __flow)           \
  {                                                         \
    .type = DEV_RES_UART,                                   \
    .u    = { .uart = {                                     \
      .baudrate  = (__baudrate),                            \
      .data_bits = (__data),                                \
      .stop_bits = (__stop),                                \
      .parity    = (__parity),                              \
      .flow_ctrl = (__flow)                                 \
    } }                                                     \
  }

#endif
