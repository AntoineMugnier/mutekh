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
#include <device/request.h>
#include <mutek/kroutine.h>

struct device_s;
struct driver_s;
struct device_i2c_s;
struct driver_i2c_s;

struct dev_i2c_config_s
{
    /** Device bit rate in Hz. */
    uint32_t bit_rate;
};

enum dev_i2c_way_e
{
    DEV_I2C_READ,
    DEV_I2C_WRITE,
};

struct dev_i2c_transfer_s {
    uint8_t *data;
    uint16_t size;
    enum dev_i2c_way_e type:1;
};

struct dev_i2c_request_s
{
    struct dev_request_s base;

    /** Address of the I2C slave device. */
    uint8_t saddr;

    /** Elementary transfers to do */
    struct dev_i2c_transfer_s *transfer;

    /** Count of trasfers pointed by @tt transfer */
    uint8_t transfer_count;

    // Device controlled from here.

    /** Request completion error.
        @list
        @item -EHOSTUNREACH Got a NACK after slave address
        @item -EIO Got a NACK after some data byte
        @end list
     */
    error_t error;

    /** Transfer that errored */
    uint8_t error_transfer;

    /** Error offset in bytes from start of transfer if -EIO.  @tt
        error_offset represents count of bytes exchanged during
        transfer before NACK.  This way, @tt error_offset is 1 if
        first data byte is NACKed.
     */
    uint16_t error_offset;
};

STRUCT_COMPOSE(dev_i2c_request_s, base);

/** @see devi2c_config_t */
#define DEVI2C_CONFIG(n) error_t (n) (                            \
    struct device_i2c_s *i2cdev,                                 \
    const struct dev_i2c_config_s *config)

/** @see devi2c_request_t */
#define DEVI2C_REQUEST(n) void (n) (                              \
    const struct device_i2c_s *i2cdev,                            \
    struct dev_i2c_request_s *req)

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

    A request without any transfer to do has an undefined behavior.

    A transfer with a zero-sized array has an undefined behavior.

    @tt base, @tt transfer and @tt transfer_count must be properly
    initialized before calling this function.

    The @ref kroutine_exec function will be called on @tt tr->base.kr
    when the request ends. This can happen before this function
    returns.  A new request may be started from the kroutine.  @tt
    tr->error, tr->error_offset and tr->error_request indicates error
    position.
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




inline ssize_t dev_i2c_spin_request(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    struct dev_i2c_transfer_s *tr,
    uint8_t tr_count)
{
    struct dev_request_status_s status;
    struct dev_i2c_request_s req =
    {
        .saddr = saddr,
        .transfer = tr,
        .transfer_count = tr_count,
    };

    dev_request_spin_init(&req.base, &status);

    DEVICE_OP(i2cdev, request, &req);

    dev_request_spin_wait(&status);

    return req.error;
}


/** @this does the same as @fn dev_i2c_wait_write_read but does not
    use the scheduler.  @this always spins on completion.
*/
config_depend(CONFIG_DEVICE_I2C)
inline ssize_t dev_i2c_spin_write_read(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    const uint8_t *wdata,
    size_t wsize,
    uint8_t *rdata,
    size_t rsize)
{
    struct dev_i2c_transfer_s tr[] = {
        {
            .type = DEV_I2C_WRITE,
            .data = (uint8_t *)wdata,
            .size = wsize,
        },
        {
            .type = DEV_I2C_READ,
            .data = rdata,
            .size = rsize,
        },
    };

    return dev_i2c_spin_request(i2cdev, saddr, tr, 2);
}

/** Synchronous helper read function.

    Shortcut for @tt dev_i2c_spin_request(i2cdev, saddr, NULL, 0, data, size).
*/
config_depend(CONFIG_DEVICE_I2C)
static inline
ssize_t dev_i2c_spin_read(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    uint8_t *data,
    size_t size)
{
    struct dev_i2c_transfer_s tr[] = {
        {
            .type = DEV_I2C_READ,
            .data = data,
            .size = size,
        },
    };

    return dev_i2c_spin_request(i2cdev, saddr, tr, 1);
}

/** Synchronous helper write function.

    Shortcut for @tt dev_i2c_spin_request(i2cdev, saddr, data, size, NULL, 0).
*/
config_depend(CONFIG_DEVICE_I2C)
static inline
ssize_t dev_i2c_spin_write(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    const uint8_t *data,
    size_t size)
{
    struct dev_i2c_transfer_s tr[] = {
        {
            .type = DEV_I2C_WRITE,
            .data = (uint8_t *)data,
            .size = size,
        },
    };

    return dev_i2c_spin_request(i2cdev, saddr, tr, 1);
}

#if defined(CONFIG_MUTEK_SCHEDULER)

inline ssize_t dev_i2c_wait_request(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    struct dev_i2c_transfer_s *tr,
    uint8_t tr_count)
{
    struct dev_request_status_s status;
    struct dev_i2c_request_s req =
    {
        .saddr = saddr,
        .transfer = tr,
        .transfer_count = tr_count,
    };

    dev_request_sched_init(&req.base, &status);

    DEVICE_OP(i2cdev, request, &req);

    dev_request_sched_wait(&status);

    return req.error;
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

    Both read and write buffers are mandatory.

    @this is a synchronous helper write/read function. @this makes the
    calling context wait while operation completes.

    @returns 0 on success or an error code.
*/
config_depend(CONFIG_DEVICE_I2C)
inline ssize_t dev_i2c_wait_write_read(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    const uint8_t *wdata,
    size_t wsize,
    uint8_t *rdata,
    size_t rsize)
{
    struct dev_i2c_transfer_s tr[] = {
        {
            .type = DEV_I2C_WRITE,
            .data = (uint8_t *)wdata,
            .size = wsize,
        },
        {
            .type = DEV_I2C_READ,
            .data = rdata,
            .size = rsize,
        },
    };

    return dev_i2c_wait_request(i2cdev, saddr, tr, 2);
}

/** @this does a request from the i2c slave device targetted by @tt
    saddr.  Sequence is:
    @list
    @item Start condition, saddr + R
    @item Read of @tt data for @tt size bytes
    @item Stop condition
    @end{list}

    @this is a synchronous helper write/read function. @this makes the
    calling context wait while operation completes.

    @returns 0 on success or an error code.
*/
config_depend(CONFIG_DEVICE_I2C)
static inline
ssize_t dev_i2c_wait_read(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    uint8_t *data,
    size_t size)
{
    struct dev_i2c_transfer_s tr[] = {
        {
            .type = DEV_I2C_READ,
            .data = data,
            .size = size,
        },
    };

    return dev_i2c_wait_request(i2cdev, saddr, tr, 1);
}

/** @this does a request from the i2c slave device targetted by @tt
    saddr.  Sequence is:
    @list
    @item Start condition, saddr + W
    @item Write of @tt data for @tt size bytes
    @item Stop condition
    @end{list}

    @this is a synchronous helper write/read function. @this makes the
    calling context wait while operation completes.

    @returns 0 on success or an error code.
*/
config_depend(CONFIG_DEVICE_I2C)
static inline
ssize_t dev_i2c_wait_write(
    const struct device_i2c_s *i2cdev,
    uint8_t saddr,
    const uint8_t *data,
    size_t size)
{
    struct dev_i2c_transfer_s tr[] = {
        {
            .type = DEV_I2C_WRITE,
            .data = (uint8_t *)data,
            .size = size,
        },
    };

    return dev_i2c_wait_request(i2cdev, saddr, tr, 1);
}

#endif

/* Resource stuff */

ALWAYS_INLINE error_t dev_i2c_res_add_address(struct device_s *dev, uint8_t saddr)
{
    return device_res_alloc_uint(dev, DEV_RES_I2C_ADDR, saddr, 0, NULL);
}

#define DEV_STATIC_RES_I2C_ADDRESS(addr_)      \
  {                                             \
    .type = DEV_RES_I2C_ADDR,                   \
      .u = { .id = {                            \
        .major = (addr_),                       \
        .minor = 0,                             \
      } }                                       \
  }

ALWAYS_INLINE error_t dev_i2c_res_get_addr(const struct device_s *dev,
                                           uint8_t *addr,
                                           uint_fast8_t index)
{
    struct dev_resource_s *r;

    r = device_res_get(dev, DEV_RES_I2C_ADDR, index);
    if (r == NULL)
        return -ENOENT;

    *addr = r->u.id.major;

    return 0;
}

#endif
