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

/**
 * @file
 * @module{Devices support library}
 * @short UART driver configuration API
 */

#ifndef __DEVICE_UART_H__
#define __DEVICE_UART_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>


/** UART baudrates. */
enum dev_uart_baudrate_e
{
  DEV_UART_BAUD_110,
  DEV_UART_BAUD_300,
  DEV_UART_BAUD_600,
  DEV_UART_BAUD_1200,
  DEV_UART_BAUD_2400,
  DEV_UART_BAUD_4800,
  DEV_UART_BAUD_9600,
  DEV_UART_BAUD_14400,
  DEV_UART_BAUD_19200,
  DEV_UART_BAUD_28800,
  DEV_UART_BAUD_38400,
  DEV_UART_BAUD_56000,
  DEV_UART_BAUD_57600,
  DEV_UART_BAUD_115200,
};

/** UART data bits modes. */
enum dev_uart_data_bits_e
{
  DEV_UART_DATA_6_BITS,
  DEV_UART_DATA_7_BITS,
  DEV_UART_DATA_8_BITS,
  DEV_UART_DATA_9_BITS,
};

/** UART stop bits. */
enum dev_uart_stop_bits_e
{
  DEV_UART_STOP_1_BIT,
  DEV_UART_STOP_2_BITS,
};

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
  enum dev_uart_baudrate_e  baudrate;

  /** data bits. */
  enum dev_uart_data_bits_e data_bits;

  /** stop bits. */
  enum dev_uart_stop_bits_e stop_bits;

  /** parity. */
  enum dev_uart_parity_e    parity;

  /** flow control. */
  bool_t                    flow_ctrl;

  /** half duplex. */
  bool_t                    half_duplex;
};


/* forward declarations. */
struct device_uart_s;

#define DEVUART_CONFIG(n) error_t (n)(struct device_uart_s     *udev, \
                                      struct dev_uart_config_s *cfg)  \
/**/

/** @This defines the prototype of the configuration function. */
typedef DEVUART_CONFIG(devuart_config_t);

/** Driver types. */
DRIVER_CLASS_TYPES(uart,
                   devuart_config_t *f_config;
                  );


/** @this helper configures the @tt udev uart device with the given
    configuration @tt cfg (and returns 0) or returns a negative error code.

    Note: if the device is busy or already in use, the function returns a
    EBUSY error.
 */
ALWAYS_INLINE
error_t dev_uart_config(struct device_uart_s     *udev,
                        struct dev_uart_config_s *cfg)
{
  return DEVICE_OP(udev, config, cfg);
}


/** @This specify a UART device configuration to store in the device tree.
    @see #DEV_STATIC_RES_UART
 */
ALWAYS_INLINE
error_t device_add_res_uart(struct device_s           *dev,
                            enum dev_uart_baudrate_e  baudrate,
                            enum dev_uart_data_bits_e data_bits,
                            enum dev_uart_stop_bits_e stop_bits,
                            enum dev_uart_parity_e    parity,
                            bool_t                    flow_ctrl,
                            bool_t                    half_duplex)
{
#if defined(CONFIG_DEVICE_UART)
  struct dev_resource_s *r;

  error_t err = device_res_alloc(dev, &r, DEV_RES_UART);
  if (err)
    return err;

  r->u.uart.baudrate    = baudrate;
  r->u.uart.data_bits   = data_bits;
  r->u.uart.stop_bits   = stop_bits;
  r->u.uart.parity      = parity;
  r->u.uart.flow_ctrl   = flow_ctrl;
  r->u.uart.half_duplex = half_duplex;

  return 0;
#else
  return -ENOTSUP;
#endif
}

#if defined(CONFIG_DEVICE_UART)

/** @This can be used to include an UART default configuration in the device
    resource list.
    @see device_res_add_uart @see #DEV_DECLARE_STATIC_RESOURCES
 */
# define DEV_STATIC_RES_UART(                               \
    __baudrate, __data, __stop, __parity, __flow, __duplex) \
  {                                                         \
    .type = DEV_RES_UART,                                   \
    .u    = { .uart = {                                     \
      .baudrate  = (__baudrate),                            \
      .data_bits = (__data),                                \
      .stop_bits = (__stop),                                \
      .parity    = (__parity),                              \
      .flow_ctrl = (__flow),                                \
      .half_duplex = (__duplex),                            \
    } }                                                     \
  }

#else

# define DEV_STATIC_RES_UART(                               \
    __baudrate, __data, __stop, __parity, __flow, __duplex) \
  {                                                         \
    .type = DEV_RES_UNUSED,                                 \
  }

#endif

#endif

