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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006
*/

/**
 * @file
 * @module{Devices support library}
 * @short Character device driver API
 */                                                                 

#ifndef __DEVICE_CHAR_H__
#define __DEVICE_CHAR_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <gct_platform.h>
#include <gct/container_clist.h>
#include <mutek/kroutine.h>

#include <device/driver.h>
#include <device/request.h>

struct driver_s;
struct dev_char_rq_s;
struct driver_char_s;
struct device_char_s;

enum dev_char_rq_type_e {
    /** Copy characters from device to caller, wait for total
        completion or error */
    DEV_CHAR_READ,
    /** Copy characters from caller to device, wait for total
        completion or error */
    DEV_CHAR_WRITE,
    /** Copy characters from device to caller, finish on first
        blocking cause */
    DEV_CHAR_READ_NONBLOCK,
    /** Copy characters from caller to device, finish on first
        blocking cause */
    DEV_CHAR_WRITE_NONBLOCK,
};

#define GCT_CONTAINER_ALGO_dev_char_queue CLIST

struct dev_char_rq_s
{
  struct dev_request_s base;

  /** request type */
  enum dev_char_rq_type_e type;

  /** character buffer */
  uint8_t *data;
  /** character buffer size */
  size_t size;

  // Driver-controlled data

  /** error code set by driver */
  error_t error;
};

STRUCT_COMPOSE(dev_char_rq_s, base);

/** Char device class @ref devchar_request_t function template. */
#define DEVCHAR_REQUEST(n)                                             \
  void (n)(                                                            \
    const struct device_char_s *cdev,                                  \
    struct dev_char_rq_s *rq)

/**
   Char device class request() function type. Enqueue a read or write request.

   @param dev pointer to device descriptor
   @param rq pointer to request. data, size and callback, field must be intialized.
*/
typedef DEVCHAR_REQUEST(devchar_request_t);

DRIVER_CLASS_TYPES(char, 
                   devchar_request_t *f_request;
                   );


inline ssize_t dev_char_spin_request(
    const struct device_char_s *cdev,
    struct dev_char_rq_s *rq)
{
    struct dev_request_status_s status;
    ssize_t todo = rq->size;

    dev_request_spin_init(&rq->base, &status);

    DEVICE_OP(cdev, request, rq);

    dev_request_spin_wait(&status);

    return rq->error ? rq->error : (todo - rq->size);
}

#if defined(CONFIG_MUTEK_SCHEDULER)

inline ssize_t dev_char_wait_request(
    const struct device_char_s *cdev,
    struct dev_char_rq_s *rq)
{
    struct dev_request_status_s status;
    ssize_t todo = rq->size;

    dev_request_sched_init(&rq->base, &status);

    DEVICE_OP(cdev, request, rq);

    dev_request_sched_wait(&status);

    return rq->error ? rq->error : (todo - rq->size);
}

/** Synchronous helper read function. This function uses the scheduler
    api to put current context in wait state if no data is available
    from device yet. This function spins in a loop waiting for read
    operation to complete when scheduler is disabled.

    @returns transferred size or a negative error code
*/
config_depend(CONFIG_DEVICE_CHAR)
inline ssize_t dev_char_wait_read(
    const struct device_char_s *cdev,
    uint8_t *data, size_t size)
{
    struct dev_char_rq_s rq =
    {
        .type = DEV_CHAR_READ,
        .data = data,
        .size = size,
    };

    return dev_char_wait_request(cdev, &rq);
}

/** Synchronous helper write function. This function uses the scheduler
    api to put current context in wait state if no data is available
    from device yet. This function spins in a loop waiting for write
    operation to complete when scheduler is disabled.

    @returns transferred size or a negative error code
*/
config_depend(CONFIG_DEVICE_CHAR)
inline ssize_t dev_char_wait_write(
    const struct device_char_s *cdev,
    const uint8_t *data, size_t size)
{
    struct dev_char_rq_s rq =
    {
        .type = DEV_CHAR_WRITE,
        .data = (uint8_t *)data,
        .size = size,
    };

    return dev_char_wait_request(cdev, &rq);
}

#endif

/** Synchronous helper read function. This function spins in a loop
    waiting for read operation to complete.

    @returns transferred size or a negative error code
*/
config_depend(CONFIG_DEVICE_CHAR)
inline ssize_t dev_char_spin_read(
    const struct device_char_s *cdev,
    uint8_t *data, size_t size)
{
    struct dev_char_rq_s rq =
    {
        .type = DEV_CHAR_READ,
        .data = data,
        .size = size,
    };

    return dev_char_spin_request(cdev, &rq);
}

/** Synchronous helper write function. This function spins in a loop
    waiting for write operation to complete.

    @returns transferred size or a negative error code
*/
config_depend(CONFIG_DEVICE_CHAR)
inline ssize_t dev_char_spin_write(
    const struct device_char_s *cdev,
    const uint8_t *data, size_t size)
{
    struct dev_char_rq_s rq =
    {
        .type = DEV_CHAR_WRITE,
        .data = (uint8_t *)data,
        .size = size,
    };

    return dev_char_spin_request(cdev, &rq);
}

#endif
