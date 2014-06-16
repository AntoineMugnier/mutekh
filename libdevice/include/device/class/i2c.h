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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009
    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2013
    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014

*/

/**
 * @file
 * @module{Devices support library}
 * @short I2c bus driver API
 */

#ifndef __DEVICE_I2C_H__
#define __DEVICE_I2C_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/device.h>
#include <device/driver.h>
#include <mutek/kroutine.h>

struct device_s;
struct driver_s;
struct device_i2c_ctrl_s;
struct driver_i2c_ctrl_s;
struct dev_i2c_ctrl_transfer_s;


/***************************************** config */

/** @This structure contains the information used to configure the I2C
    controller. **/
struct dev_i2c_ctrl_config_s
{
  /** This field gives the bitrate in bits per seconds. */
  uint32_t bit_rate;
};

/** @see devi2c_ctrl_config_t */
#define DEVI2C_CTRL_CONFIG(n) error_t (n) ( \
  const struct device_i2c_ctrl_s *i2cdev,   \
  struct dev_i2c_ctrl_config_s *cfg)        \
/**/

/**
   @This changes the configuration of the controller. If the controller does
   not support requested configuration, this function return @tt -ENOTSUP.

   @This function returns @tt -EBUSY if a transfer is currently being
   processed.
*/
typedef DEVI2C_CTRL_CONFIG(devi2c_ctrl_config_t);


/***************************************** transfer */

enum dev_i2c_ctrl_transfer_op_e
{
  DEV_I2C_OP_NONE,
  DEV_I2C_OP_START,
  DEV_I2C_OP_STOP,
  DEV_I2C_OP_START_STOP
};

enum dev_i2c_ctrl_transfer_dir_e
{
  DEV_I2C_TR_WRITE,
  DEV_I2C_TR_READ
};

struct dev_i2c_ctrl_transfer_s
{
  /** The @ref kroutine_exec function is called on this kroutine when a
      transfer ends. When this happens, either the @tt count field of the
      transfer is zero or the @tt err field is set. */
  struct kroutine_s                 kr;

  /** Enable start/stop phase */
  enum dev_i2c_ctrl_transfer_op_e   op:2;

  /** Direction of the I2C transfer. */
  enum dev_i2c_ctrl_transfer_dir_e  dir:1;

  /** Address of the I2C slave device. */
  uint8_t                           saddr;

  /** Number of bytes to transfer. This field will be updated during the
      transfer. If 0 no data phase*/
  size_t                            count;

  /** Pointer to I2C buffer data */
  uint8_t                           *data;

  /** Callback private data. */
  void                              *pvdata;

  /** Transfer completion error. */
  error_t                           error;

  /** Associated I2C controller device. */
  struct device_i2c_ctrl_s          *i2cdev;
};

/** @see devi2c_ctrl_transfer_t */
#define DEVI2C_CTRL_TRANSFER(n) void (n) ( \
  const struct device_i2c_ctrl_s *i2cdev,  \
  struct dev_i2c_ctrl_transfer_s *tr       \
)                                          \
/**/

/** @This starts an I2C transfer. A single I2C data transfer can be started at
    the same time. This is the low level transfer function of the I2C device
    class.

    All fields of the transfer object except @tt pvdata, @tt err and @i2cdev
    must be properly initialized before calling this function. The @tt count
    field may be zero. The transfer will fail with @tt -EBUSY if an another
    transfer is currently being processed.

    The @ref kroutine_exec function will be called on @tt tr->kr when the
    transfer ends. This can happen before this function returns. It is ok to
    start a new transfer from the kroutine. This @tt tr->err value indicates
    the error status of the transfer.
*/
typedef DEVI2C_CTRL_TRANSFER(devi2c_ctrl_transfer_t);

/***************************************** device class */

DRIVER_CLASS_TYPES(i2c_ctrl,
  devi2c_ctrl_config_t      *f_config;
  devi2c_ctrl_transfer_t    *f_transfer;
);


/***************************************** helpers */

/** Config helper to change the bit rate of the I2C controller.

    @returns 0 if the configuration succeeded or a negative error code.
*/
config_depend(CONFIG_DEVICE_I2C)
error_t dev_i2c_set_bit_rate(struct device_i2c_ctrl_s *i2cdev,
                             uint32_t                 bps);

/** Synchronous helper scan function. This function uses the scheduler
    api to put current context in wait state if the acknowledge is pending
    This function spins in a loop waiting for scan operation to complete
    when scheduler is disabled.

    The slave address is given in @tt saddr argument.

    @returns 0 if the address matches with a slave device or negative
    error code.
*/
config_depend(CONFIG_DEVICE_I2C)
error_t dev_i2c_wait_scan(const struct device_i2c_ctrl_s    *i2cdev,
                          uint8_t                           saddr);

/** Synchronous helper scan function. This function spins in a loop
    waiting for read operation to complete.

    The slave address is given in @tt saddr argument.

    @returns 0 if the address matches with a slave device or negative
    error code.
*/
config_depend(CONFIG_DEVICE_I2C)
error_t dev_i2c_spin_scan(const struct device_i2c_ctrl_s    *i2cdev,
                          uint8_t                           saddr);

/** Synchronous helper read function. This function uses the scheduler
    api to put current context in wait state if no data is currently
    available. This function spins in a loop waiting for read
    operation to complete when scheduler is disabled.

    The slave address is given in @tt saddr argument.

    @returns processed bytes count or negative error code.
*/
config_depend(CONFIG_DEVICE_I2C)
ssize_t dev_i2c_wait_read(const struct device_i2c_ctrl_s    *i2cdev,
                          enum dev_i2c_ctrl_transfer_op_e   op,
                          uint8_t                           saddr,
                          uint8_t                           *data,
                          size_t                            size);

/** Synchronous helper read function. This function spins in a loop
    waiting for read operation to complete.

    The slave address is given in @tt saddr argument.

    @returns processed bytes count or negative error code.
*/
config_depend(CONFIG_DEVICE_I2C)
ssize_t dev_i2c_spin_read(const struct device_i2c_ctrl_s    *i2cdev,
                          enum dev_i2c_ctrl_transfer_op_e   op,
                          uint8_t                           saddr,
                          uint8_t                           *data,
                          size_t                            size);

/** Synchronous helper write function. This function uses the scheduler
    api to put current context in wait state if some data is currently
    pending. This function spins in a loop waiting for write
    operation to complete when scheduler is disabled.

    The slave address is given in @tt saddr argument.

    @returns processed bytes count or negative error code.
*/
config_depend(CONFIG_DEVICE_I2C)
ssize_t dev_i2c_wait_write(const struct device_i2c_ctrl_s    *i2cdev,
                           enum dev_i2c_ctrl_transfer_op_e   op,
                           uint8_t                           saddr,
                           const uint8_t                     *data,
                           size_t                            size);

/** Synchronous helper write function. This function spins in a loop
    waiting for write operation to complete.

    The slave address is given in @tt saddr argument.

    @returns processed bytes count or negative error code.
*/
config_depend(CONFIG_DEVICE_I2C)
ssize_t dev_i2c_spin_write(const struct device_i2c_ctrl_s    *i2cdev,
                           enum dev_i2c_ctrl_transfer_op_e   op,
                           uint8_t                           saddr,
                           const uint8_t                     *data,
                           size_t                            size);

#endif

