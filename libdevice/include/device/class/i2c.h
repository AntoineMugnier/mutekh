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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009,2014
    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013
    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014

*/

/**
   @file
   @module{Devices support library}
   @short I2c bus driver API
*/

#ifndef __DEVICE_I2C_H__
#define __DEVICE_I2C_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <gct_platform.h>
#include <gct/container_clist.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <mutek/kroutine.h>

struct device_s;
struct driver_s;
struct device_i2c_s;
struct driver_i2c_s;

struct dev_i2c_config_s {
  /** Device bit rate in Hz. */
  uint32_t bit_rate;
};

#define GCT_CONTAINER_ALGO_dev_i2c_request_queue CLIST

struct dev_i2c_request_s
{
  /** The @ref kroutine_exec function is called on this kroutine when a
      request ends. */
  struct kroutine_s kr;

  /** Address of the I2C slave device. */
  uint8_t saddr;

  /** Pointer to data to write. May be @tt NULL if @tt wdata_len is 0. */
  const uint8_t *wdata;

  /** Number of bytes to write. May be 0 is no data is to write. */
  size_t wdata_len;

  /** Pointer to data to read. May be @tt NULL if @tt rdata_len is 0. */
  uint8_t *rdata;

  /** Number of bytes to read. May be 0 if no data is to read. */
  size_t rdata_len;

  // Device controlled from here.

  /** Driver's private data. */
  void *drvdata;

  GCT_CONTAINER_ENTRY(dev_i2c_request_queue, queue_entry);

  /** Error offset in bytes from start of request:
      @list
      @item 0 after address
      @item 1 after first data bytes (whether read or write)
      @item (wdata_len + rdata_len) at last byte
      @end{list}
   */
  size_t error_offset;

  /** Request completion error. */
  error_t error;
};

GCT_CONTAINER_TYPES(dev_i2c_request_queue, struct dev_i2c_request_s *, queue_entry);
GCT_CONTAINER_FCNS(dev_i2c_request_queue, inline, dev_i2c_request_queue,
	           init, destroy, isempty, pushback, pop, head);

/** @see devi2c_config_t */
#define DEVI2C_CONFIG(n) error_t (n) (                            \
    struct device_i2c_s *i2cdev,                                 \
    const struct dev_i2c_config_s *config)

/** @see devi2c_request_t */
#define DEVI2C_REQUEST(n) void (n) (                              \
    const struct device_i2c_s *i2cdev,                            \
    struct dev_i2c_request_s *tr)

/** @This configures an I2C controller.

    A config request updates the controller bit rate.  Changing
    configuration while there are requests in queue has undefined
    behavior.
*/
typedef DEVI2C_CONFIG(devi2c_config_t);

/** @This starts an I2C request.

    A I2C request is a made of:
    @list
    @item address selection,
    @item an optional written buffer,
    @item an optional read buffer.
    @end{list}

    Buffers must be initialized and their corresponding data length
    must be set accordingly.

    A request without any data to request has an undefined behavior.

    All fields of the request object except @tt pvdata, @tt
    error_offset, @tt error and @tt i2cdev must be properly
    initialized before calling this function. The @tt count field may
    be zero.

    The @ref kroutine_exec function will be called on @tt tr->kr when
    the request ends. This can happen before this function returns.
    A new request may be started from the kroutine.  @tt tr->error
    value indicates the error status of the request.  @tt
    tr->error_offset is the offset from start of data bytes.
*/
typedef DEVI2C_REQUEST(devi2c_request_t);

DRIVER_CLASS_TYPES(i2c,
  devi2c_config_t *f_config;
  devi2c_request_t *f_request;
);

/** @this reconfigures the i2c controller configuration.

    @param config Configuration structure.
    @returns 0 or -ENOTSUP if unsupported configuration.
 */
static inline
error_t dev_i2c_config(
  struct device_i2c_s *i2cdev,
  const struct dev_i2c_config_s *config)
{
  return DEVICE_OP(i2cdev, config, config);
}

/** @this does a request to/from the i2c slave device targetted by
    @tt saddr.  Sequence is:
    @list
    @item Start condition, saddr + W
    @item Write of @tt wdata for @tt wdata_len bytes
    @item Restart condition, saddr + R
    @item Read of @tt rdata for @tt rdata_len bytes
    @item Stop condition
    @end{list}

    If there is only one of @tt wdata and @tt rdata, no restart
    condition is done, a sole read or write transaction is issued.

    @this is a synchronous helper write/read function. @this makes the
    calling context sleep while operation completes.  If scheduler is
    unavailable, @this spins.

    @returns 0 on success or an error code.
*/
config_depend(CONFIG_DEVICE_I2C)
ssize_t dev_i2c_wait_write_read(
  const struct device_i2c_s *i2cdev,
  uint8_t saddr,
  const uint8_t *wdata,
  size_t wdata_len,
  uint8_t *rdata,
  size_t rdata_len);

/** @this does the same as @fn dev_i2c_wait_write_read but does not
    use the scheduler.  @this always spins on completion.
*/
config_depend(CONFIG_DEVICE_I2C)
ssize_t dev_i2c_spin_write_read(
  const struct device_i2c_s *i2cdev,
  uint8_t saddr,
  const uint8_t *wdata,
  size_t wdata_len,
  uint8_t *rdata,
  size_t rdata_len);

/** Synchronous helper read function.

    Shortcut for @tt dev_i2c_wait_write_read(i2cdev, saddr, NULL, 0, data, size).
*/
config_depend(CONFIG_DEVICE_I2C)
static inline
ssize_t dev_i2c_wait_read(
  const struct device_i2c_s *i2cdev,
  uint8_t saddr,
  uint8_t *data,
  size_t size)
{
  return dev_i2c_wait_write_read(i2cdev, saddr, NULL, 0, data, size);
}

/** Synchronous helper read function.

    Shortcut for @tt dev_i2c_spin_write_read(i2cdev, saddr, NULL, 0, data, size).
*/
config_depend(CONFIG_DEVICE_I2C)
static inline
ssize_t dev_i2c_spin_read(
  const struct device_i2c_s *i2cdev,
  uint8_t saddr,
  uint8_t *data,
  size_t size)
{
  return dev_i2c_spin_write_read(i2cdev, saddr, NULL, 0, data, size);
}

/** Synchronous helper write function.

    Shortcut for @tt dev_i2c_wait_write_read(i2cdev, saddr, data, size, NULL, 0).
*/
config_depend(CONFIG_DEVICE_I2C)
static inline
ssize_t dev_i2c_wait_write(
  const struct device_i2c_s *i2cdev,
  uint8_t saddr,
  const uint8_t *data,
  size_t size)
{
  return dev_i2c_wait_write_read(i2cdev, saddr, data, size, NULL, 0);
}

/** Synchronous helper write function.

    Shortcut for @tt dev_i2c_spin_write_read(i2cdev, saddr, data, size, NULL, 0).
*/
config_depend(CONFIG_DEVICE_I2C)
static inline
ssize_t dev_i2c_spin_write(
  const struct device_i2c_s *i2cdev,
  uint8_t saddr,
  const uint8_t *data,
  size_t size)
{
  return dev_i2c_spin_write_read(i2cdev, saddr, data, size, NULL, 0);
}

#endif
