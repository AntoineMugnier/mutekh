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

   @section {Description}

   I2C controller class abstracts access to an I2C bus.

   I2C bus only has one configuration parameter: bus speed.  Bus speed
   is shared for the whole bus as all devices need to be able to
   decode their own addresses and monitor Start and Stop conditions.

   I2C requests are atomic, they may not be interleaved.  There is one
   request queue by device, all requests are handled in order.  A
   request starts with a Start condition, ends with a Stop condition.
   A request may imply Restart conditions if needed.

   A request structure contains the following information:
   @list
   @item Slave address,
   @item An array of transfers and its count of entries. Each transfer
     defines:
     @list
     @item Way of transfer (Read or Write),
     @item Some data buffer to exchange (written to or read from),
     @item Size of said buffer.
     @end list
   @end list

   Consecutive transfers in a request are concatenated if they operate
   the same way, or a restart is issued in-between if they are in
   different ways.

   @end section

   @section {Example}

   The following code creates a request to read first 128 bytes of an
   I2C eeprom at address 0x50:

   @code
   struct device_i2c_s i2c_dev;

   // Lookup accessor for i2c_dev here...

   uint8_t addr[2] = {0, 0};
   uint8_t data[128];
   struct dev_i2c_transfer_s transfer[2] =
   {
       {
           .type = DEV_I2C_WRITE,
           .data = addr,
           .size = sizeof(addr),
       },
       {
           .type = DEV_I2C_READ,
           .data = data,
           .size = sizeof(data),
       },
   };
   struct dev_i2c_rq_s rq =
   {
       .saddr = 0x50,
       .transfer = transfer,
       .transfer_count = 2,
   };

   kroutine_init(&rq.base.kr, my_callback, KROUTINE_IMMEDIATE);

   DEVICE_OP(&i2c_dev, request, &rq);
   @end code

   This will create the following transaction on the bus:
   @list
   @item Start condition
   @item Slave selection, address 0x50, Write
   @item Write byte (twice)
   @item Restart condition
   @item Slave selection, address 0x50, Read
   @item Read byte (128 times)
   @item Stop condition
   @end list

   @end section

   @section {Error handling}

   When kroutine is called back after a transaction completion, @tt
   error field of the request contains the completion status.  Normal
   operation completion has @tt error field set to @tt 0.

   Usual I2C error conditions are NACK after some byte.  They may
   occur on any byte of the transaction, including address selection
   bytes.  In case this happens, @tt error field may take one of the
   following values:

   @list
   @item @tt -EIO: NACK occurred after a data byte,
   @item @tt -EHOSTUNREACH: NACK occurred after a slave selection byte.
   @end list

   Then, @tt error_transfer field of the request corresponds to the
   failed transfer, @tt error_offset contains the actually transferred
   data bytes before NACK. @tt error_offset counts from the start of
   transfer (not request).

   If caller needs to retry a request, it may resubmit again the same
   request structure without modification from the kroutine.

   Other error conditions include the following values:

   @list
   @item @tt -ENOTSUP: Enqueued request is not supported by device
   (most probably because of some hardware-dependant limitations),
   @item @tt -ETIMEDOUT: Slave did some clock-stretching and bus
   controlled issued a timeout condition.  Timeout value is
   driver-dependant.
   @end list

   @end section
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

/** @this is an I2C controller master configuration structure */
struct dev_i2c_config_s
{
    /** Device bit rate in Hz. */
    uint32_t bit_rate;
};

/** Way of an I2C elementary transfer */
enum dev_i2c_way_e
{
    /** From slave to master */
    DEV_I2C_READ,
    /** From master to slave */
    DEV_I2C_WRITE,
};

/** @this is an elementary transfer in an I2C request */
struct dev_i2c_transfer_s {
    /** Data buffer to transfer (either read or write) */
    uint8_t *data;
    /** Size of @tt data buffer */
    uint16_t size;
    /** Way of transfer */
    enum dev_i2c_way_e BITFIELD(type,1);
};

/**
   @this is an I2C Request structure.

   Caller must initialize @tt base, @tt saddr, @tt transfer and @tt
   transfer_count fields.  Driver may not modify them.

   Driver fills @tt error, @tt error_transfer and @tt error_offset
   before calling the kroutine. @tt error_transfer and @tt
   error_offset fields have no meaning if @tt error is @tt 0.
 */
struct dev_i2c_rq_s
{
    struct dev_request_s base;

    /** Address of the I2C slave device. */
    uint8_t saddr;

    /** Elementary transfers to do */
    struct dev_i2c_transfer_s *transfer;

    /** Count of trasfers pointed by @tt transfer */
    uint8_t transfer_count;

    // Device controlled from here.

    /** Request completion error:
        @list
        @item -EHOSTUNREACH Got a NACK after slave address,
        @item -EIO Got a NACK after some data byte.
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

STRUCT_INHERIT(dev_i2c_rq_s, dev_request_s, base);

/** @see dev_i2c_config_t */
#define DEV_I2C_CONFIG(n) error_t (n) (                            \
    struct device_i2c_s *accessor,                                 \
    const struct dev_i2c_config_s *config)

/** @see dev_i2c_request_t */
#define DEV_I2C_REQUEST(n) void (n) (                              \
    const struct device_i2c_s *accessor,                            \
    struct dev_i2c_rq_s *req)

/** @This configures an I2C controller.

    A config request updates the controller bit rate.  Changing
    configuration while there are requests in queue has undefined
    behavior.
*/
typedef DEV_I2C_CONFIG(dev_i2c_config_t);

/** @This starts an I2C request.

    A I2C request is a made of:
    @list
    @item address selection,
    @item an optional written buffer,
    @item an optional read buffer.
    @end list

    Buffers must be initialized and their corresponding data length
    must be set accordingly.

    A request without any transfer to do has an undefined behavior.

    A transfer with a zero-sized array has an undefined behavior.

    @tt base, @tt transfer and @tt transfer_count must be properly
    initialized before calling this function.

    The @ref kroutine_exec function will be called on @tt tr->base.kr
    when the request ends. This can happen before this function
    returns.  A new request may be started from the kroutine. @tt
    rq->error, tr->error_offset and tr->error_request indicates error
    position.
*/
typedef DEV_I2C_REQUEST(dev_i2c_request_t);

DRIVER_CLASS_TYPES(i2c,
    dev_i2c_config_t *f_config;
    dev_i2c_request_t *f_request;
);

#define DRIVER_I2C_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_i2c_s){   \
    .class_ = DRIVER_CLASS_I2C,                                  \
    .f_config = prefix ## _config,                               \
    .f_request = prefix ## _request,                             \
  })

/** @this reconfigures the i2c controller configuration.

    @param config Configuration structure.
    @returns 0 or -ENOTSUP if unsupported configuration.
 */
inline
error_t dev_i2c_config(
  struct device_i2c_s *accessor,
  const struct dev_i2c_config_s *config)
{
  return DEVICE_OP(accessor, config, config);
}




inline ssize_t dev_i2c_spin_request(
    const struct device_i2c_s *accessor,
    uint8_t saddr,
    struct dev_i2c_transfer_s *tr,
    uint8_t tr_count)
{
    struct dev_request_status_s status;
    struct dev_i2c_rq_s req =
    {
        .saddr = saddr,
        .transfer = tr,
        .transfer_count = tr_count,
    };

    dev_request_spin_init(&req.base, &status);

    DEVICE_OP(accessor, request, &req);

    dev_request_spin_wait(&status);

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
    @end list

    Both read and write buffers are mandatory.

    @this is a synchronous helper write/read function. @this makes the
    calling context spin while operation completes.

    @returns 0 on success or an error code.
*/
config_depend(CONFIG_DEVICE_I2C)
inline ssize_t dev_i2c_spin_write_read(
    const struct device_i2c_s *accessor,
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

    return dev_i2c_spin_request(accessor, saddr, tr, 2);
}

/** Synchronous helper read function.

    Shortcut for @tt dev_i2c_spin_request(accessor, saddr, NULL, 0, data, size).
*/
config_depend(CONFIG_DEVICE_I2C)
inline
ssize_t dev_i2c_spin_read(
    const struct device_i2c_s *accessor,
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

    return dev_i2c_spin_request(accessor, saddr, tr, 1);
}

/** Synchronous helper write function.

    Shortcut for @tt dev_i2c_spin_request(accessor, saddr, data, size, NULL, 0).
*/
config_depend(CONFIG_DEVICE_I2C)
inline
ssize_t dev_i2c_spin_write(
    const struct device_i2c_s *accessor,
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

    return dev_i2c_spin_request(accessor, saddr, tr, 1);
}

#if defined(CONFIG_MUTEK_CONTEXT_SCHED)

inline ssize_t dev_i2c_wait_request(
    const struct device_i2c_s *accessor,
    uint8_t saddr,
    struct dev_i2c_transfer_s *tr,
    uint8_t tr_count)
{
    struct dev_request_status_s status;
    struct dev_i2c_rq_s req =
    {
        .saddr = saddr,
        .transfer = tr,
        .transfer_count = tr_count,
    };

    dev_request_sched_init(&req.base, &status);

    DEVICE_OP(accessor, request, &req);

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
    @end list

    Both read and write buffers are mandatory.

    @this is a synchronous helper write/read function. @this makes the
    calling context wait while operation completes.

    @returns 0 on success or an error code.
*/
config_depend(CONFIG_DEVICE_I2C)
inline ssize_t dev_i2c_wait_write_read(
    const struct device_i2c_s *accessor,
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

    return dev_i2c_wait_request(accessor, saddr, tr, 2);
}

/** @this does a request from the i2c slave device targetted by @tt
    saddr.  Sequence is:
    @list
    @item Start condition, saddr + R
    @item Read of @tt data for @tt size bytes
    @item Stop condition
    @end list

    @this is a synchronous helper write/read function. @this makes the
    calling context wait while operation completes.

    @returns 0 on success or an error code.
*/
config_depend(CONFIG_DEVICE_I2C)
inline
ssize_t dev_i2c_wait_read(
    const struct device_i2c_s *accessor,
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

    return dev_i2c_wait_request(accessor, saddr, tr, 1);
}

/** @this does a request from the i2c slave device targetted by @tt
    saddr.  Sequence is:
    @list
    @item Start condition, saddr + W
    @item Write of @tt data for @tt size bytes
    @item Stop condition
    @end list

    @this is a synchronous helper write/read function. @this makes the
    calling context wait while operation completes.

    @returns 0 on success or an error code.
*/
config_depend(CONFIG_DEVICE_I2C)
inline
ssize_t dev_i2c_wait_write(
    const struct device_i2c_s *accessor,
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

    return dev_i2c_wait_request(accessor, saddr, tr, 1);
}

#endif

/* Resource stuff */

config_depend_and2_alwaysinline(CONFIG_DEVICE_I2C, CONFIG_DEVICE_RESOURCE_ALLOC,
error_t dev_i2c_res_add_address(struct device_s *dev, uint8_t saddr),
{
    return device_res_alloc_uint(dev, DEV_RES_I2C_ADDR, saddr, 0, NULL);
})

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

#ifdef CONFIG_DEVICE_I2C
# define DEV_STATIC_RES_DEV_I2C(path_)                          \
  DEV_STATIC_RES_DEVCLASS_PARAM("i2c", path_, DRIVER_CLASS_I2C)
#else
# define DEV_STATIC_RES_DEV_I2C(path_)                                  \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }
#endif


#endif
