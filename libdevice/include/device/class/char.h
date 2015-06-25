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
   @file
   @module{Devices support library}
   @short Character device driver API

   @section {Description}

   Character device class abstracts access to a byte-stream oriented
   device.

   Two main types of requests are available on this driver: Read and
   Write.

   Each request targets a data buffer and a direction for transfer,
   either @tt DEV_CHAR_READ or @tt DEV_CHAR_WRITE.

   A variant of read and write operations permits to return from
   request with as-much data as possible, but without more waiting
   than necessary if no new data is available from hardware.  These
   are @tt DEV_CHAR_READ_PARTIAL or @tt DEV_CHAR_WRITE_PARTIAL.  This
   is not a non-blocking transfer.  Request may block, but as soon as
   it is considered and as soon as at least one byte has been
   transferred, it may return.

   @end section

   @section {Example}

   The following code reads 32 characters from a device, but accepts
   to get less if less data is available from the underlying hardware:

   @code
   struct device_char_s char_dev;

   // Lookup accessor for char_dev here...

   uint8_t data[32];
   struct dev_char_rq_s rq =
   {
       .type = DEV_CHAR_READ_PARTIAL,
       .data = data,
       .size = sizeof(data),
   };

   kroutine_init(&rq.base.kr, my_callback, KROUTINE_IMMEDIATE);

   DEVICE_OP(&char_dev, request, &rq);
   @end code

   @end section

   @section {Error handling}

   When kroutine is called back after a request completion, @tt error
   field of the request contains the completion status.  Normal
   operation completion has @tt error field set to @tt 0.

   For a partial read or write, completion without the whole buffer
   tranferred is a normal completion.

   If underlying hardware gets an error condition, @tt error is set to
   -EIO.

   @end section

   @section {Request Completion Information}

   Driver updates @tt data and @tt size fields of the request in order
   to indicate what is left to transfer.

   In kroutine called after a partial transfer, caller may resubmit
   the same request untouched from the kroutine to request for more
   data.

   @end section
 */                                                                 

#ifndef __DEVICE_CHAR_H__
#define __DEVICE_CHAR_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>
#include <device/request.h>

struct driver_s;
struct dev_char_rq_s;
struct driver_char_s;
struct device_char_s;

/** @this defines possible request types */
enum dev_char_rq_type_e
{
  /** Copy characters from device to caller, wait for total
      completion or error */
  DEV_CHAR_READ,
  /** Copy characters from caller to device, wait for total
      completion or error */
  DEV_CHAR_WRITE,
  /** Copy characters from device to caller, finish on first
      blocking cause after some byte transfer */
  DEV_CHAR_READ_PARTIAL,
  /** Copy characters from caller to device, finish on first
      blocking cause after some byte transfer */
  DEV_CHAR_WRITE_PARTIAL,
};

struct dev_char_rq_s
{
  struct dev_request_s base;

  /** request type */
  enum dev_char_rq_type_e type;

  /** character buffer, updated by the driver to point to the next
      unprocessed character. */
  uint8_t *data;
  /** character buffer size, updated by the driver. */
  size_t size;

  /** error code set by driver */
  error_t error;
};

STRUCT_INHERIT(dev_char_rq_s, dev_request_s, base);

/** Char device class @ref dev_char_request_t function template. */
#define DEV_CHAR_REQUEST(n)                                             \
  void (n)(                                                            \
    const struct device_char_s *accessor,                                  \
    struct dev_char_rq_s *rq)

/**
   Char device class request() function type. Enqueue a read or write request.

   @param dev pointer to device descriptor
   @param rq pointer to request.
*/
typedef DEV_CHAR_REQUEST(dev_char_request_t);

DRIVER_CLASS_TYPES(char, 
                   dev_char_request_t *f_request;
                   );

#define DRIVER_CHAR_METHODS(prefix)                                \
  (&(const struct driver_char_s){                                  \
    .class_ = DRIVER_CLASS_CHAR,                                   \
    .f_request = prefix ## _request,                               \
  })


inline ssize_t dev_char_spin_request(
    const struct device_char_s *accessor,
    struct dev_char_rq_s *rq)
{
    struct dev_request_status_s status;
    ssize_t todo = rq->size;

    dev_request_spin_init(&rq->base, &status);

    DEVICE_OP(accessor, request, rq);

    dev_request_spin_wait(&status);

    return rq->error ? rq->error : (todo - rq->size);
}

#if defined(CONFIG_MUTEK_SCHEDULER)

inline ssize_t dev_char_wait_request(
    const struct device_char_s *accessor,
    struct dev_char_rq_s *rq)
{
    struct dev_request_status_s status;
    ssize_t todo = rq->size;

    dev_request_sched_init(&rq->base, &status);

    DEVICE_OP(accessor, request, rq);

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
    const struct device_char_s *accessor,
    uint8_t *data, size_t size)
{
    struct dev_char_rq_s rq =
    {
        .type = DEV_CHAR_READ,
        .data = data,
        .size = size,
    };

    return dev_char_wait_request(accessor, &rq);
}

/** Synchronous helper write function. This function uses the scheduler
    api to put current context in wait state if no data is available
    from device yet. This function spins in a loop waiting for write
    operation to complete when scheduler is disabled.

    @returns transferred size or a negative error code
*/
config_depend(CONFIG_DEVICE_CHAR)
inline ssize_t dev_char_wait_write(
    const struct device_char_s *accessor,
    const uint8_t *data, size_t size)
{
    struct dev_char_rq_s rq =
    {
        .type = DEV_CHAR_WRITE,
        .data = (uint8_t *)data,
        .size = size,
    };

    return dev_char_wait_request(accessor, &rq);
}

#endif

/** Synchronous helper read function. This function spins in a loop
    waiting for read operation to complete.

    @returns transferred size or a negative error code
*/
config_depend(CONFIG_DEVICE_CHAR)
inline ssize_t dev_char_spin_read(
    const struct device_char_s *accessor,
    uint8_t *data, size_t size)
{
    struct dev_char_rq_s rq =
    {
        .type = DEV_CHAR_READ,
        .data = data,
        .size = size,
    };

    return dev_char_spin_request(accessor, &rq);
}

/** Synchronous helper write function. This function spins in a loop
    waiting for write operation to complete.

    @returns transferred size or a negative error code
*/
config_depend(CONFIG_DEVICE_CHAR)
inline ssize_t dev_char_spin_write(
    const struct device_char_s *accessor,
    const uint8_t *data, size_t size)
{
    struct dev_char_rq_s rq =
    {
        .type = DEV_CHAR_WRITE,
        .data = (uint8_t *)data,
        .size = size,
    };

    return dev_char_spin_request(accessor, &rq);
}

#endif
