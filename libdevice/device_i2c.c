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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009, 2014
    Copyright Julien Peeters <contact@julienpeeters.net> (c) 2014

    Synchronous read and write functions for i2c devices.
*/

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/class/i2c.h>

#include <hexo/lock.h>
#include <hexo/interrupt.h>

#if defined(CONFIG_MUTEK_SCHEDULER)
# include <mutek/scheduler.h>
#endif

#if defined(CONFIG_DEVICE_I2C_REQUEST)
GCT_CONTAINER_PROTOTYPES(dev_i2c_ctrl_queue, extern inline, dev_i2c_ctrl_queue,
                   init, destroy, pop, remove, push, push_back, isempty);
#endif

extern inline
error_t dev_i2c_config(
  struct device_i2c_s *i2cdev,
  const struct dev_i2c_config_s *config);

extern inline
ssize_t dev_i2c_spin_request(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    struct dev_i2c_transfer_s *tr,
    uint8_t tr_count);

extern inline
ssize_t dev_i2c_spin_write_read(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    const uint8_t *wdata,
    size_t wsize,
    uint8_t *rdata,
    size_t rsize);

extern inline
ssize_t dev_i2c_spin_read(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    uint8_t *data,
    size_t size);

extern inline
ssize_t dev_i2c_spin_write(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    const uint8_t *data,
    size_t size);

#if defined(CONFIG_MUTEK_SCHEDULER)

extern inline
ssize_t dev_i2c_wait_request(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    struct dev_i2c_transfer_s *tr,
    uint8_t tr_count);

extern inline
ssize_t dev_i2c_wait_write_read(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    const uint8_t *wdata,
    size_t wsize,
    uint8_t *rdata,
    size_t rsize);

extern inline
ssize_t dev_i2c_wait_read(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    uint8_t *data,
    size_t size);

extern inline
ssize_t dev_i2c_wait_write(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    const uint8_t *data,
    size_t size);

#endif
