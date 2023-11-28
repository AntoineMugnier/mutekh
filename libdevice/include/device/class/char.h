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
   @module {Core::Devices support library}
   @short Character device driver API
   @index {Character device} {Device classes}
   @csee DRIVER_CLASS_CHAR

   This class provides access to byte-stream oriented devices.

   Two main types of operation are available on this driver: Byte
   buffer read and byte buffer write. Some variants
   operations exist as defined in @ref dev_char_rq_type_e.

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

   dev_char_rq_init(&rq, my_callback);

   DEVICE_OP(&char_dev, request, &rq);
   @end code

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

/** @internal op */
#define  _DEV_CHAR_WRITE 1
/** @internal mode */
#define  _DEV_CHAR_ALL 2
/** @internal mode */
#define  _DEV_CHAR_PARTIAL 4
/** @internal mode */
#define  _DEV_CHAR_NONBLOCK 8
/** @internal mode */
#define  _DEV_CHAR_FRAME 16
/** @internal mode */
#define  _DEV_CHAR_POLL 32
/** @internal flush */
#define  _DEV_CHAR_FLUSH 64

/** @this defines possible request types */
enum dev_char_rq_type_e
{
  /** Read data from the device. If no error occur, the request does
      not terminate until all data have been read. */
  DEV_CHAR_READ = _DEV_CHAR_ALL,
  /** Read data from the device. If no error occur, the request does
      not terminate until all data have been read. The size field is
      updated but no data is stored. */
  DEV_CHAR_DISCARD = _DEV_CHAR_ALL | _DEV_CHAR_FLUSH,
  /** Write data to the device. If no error occur, the request does
      not terminate until all data have been written. */
  DEV_CHAR_WRITE = _DEV_CHAR_WRITE | _DEV_CHAR_ALL,
  /** This is equivalent to @ref DEV_CHAR_WRITE, forcing flush of
      output data. On devices where data are always transmitted sent
      immediately, this is handled as a regular write. */
  DEV_CHAR_WRITE_FLUSH = _DEV_CHAR_WRITE | _DEV_CHAR_ALL | _DEV_CHAR_FLUSH,
  /** Read data from the device. If no error occur, the request does
      not terminate until at least one byte of data has been read. */
  DEV_CHAR_READ_PARTIAL = _DEV_CHAR_PARTIAL,
  /** Write data to the device. If no error occur, the request does
      not terminate until at least one byte of data has been written. */
  DEV_CHAR_WRITE_PARTIAL = _DEV_CHAR_WRITE | _DEV_CHAR_PARTIAL,
  /** This is equivalent to @ref DEV_CHAR_WRITE_PARTIAL, forcing flush
      of output data. On devices where data are always transmitted sent
      immediately, this is handled as a regular write. */
  DEV_CHAR_WRITE_PARTIAL_FLUSH = _DEV_CHAR_WRITE | _DEV_CHAR_PARTIAL | _DEV_CHAR_FLUSH,
  /** Read data from the device. The request terminates
      immediately even if no data is currently available. */
  DEV_CHAR_READ_NONBLOCK = _DEV_CHAR_NONBLOCK,
  /** Write data to the device. The request terminates
      immediately even if no data can be written currently. */
  DEV_CHAR_WRITE_NONBLOCK = _DEV_CHAR_WRITE | _DEV_CHAR_NONBLOCK,
  /** This is equivalent to @ref DEV_CHAR_WRITE_NONBLOCK, forcing flush
      of output data. */
  DEV_CHAR_WRITE_NONBLOCK_FLUSH = _DEV_CHAR_WRITE | _DEV_CHAR_NONBLOCK | _DEV_CHAR_FLUSH,
  /** This request terminates when the specified amount of data bytes
      is currently available from the device. The data field of the
      request is not used and the size field is not updated. Most
      device may not support this operation for size greater than 1. */
  DEV_CHAR_READ_POLL = _DEV_CHAR_POLL,
  /** This request terminates when the specified amount of data bytes
      may be written to the device without blocking. The data field of
      the request is not used and the size field is not updated. Most
      device may not support this operation for size greater than 1. */
  DEV_CHAR_WRITE_POLL = _DEV_CHAR_WRITE | _DEV_CHAR_POLL,
  /** Read a frame from the device. This operation is only
      supported by devices working with framed data. The returned data
      is guaranteed to match a frame start boundary. The returned
      frame data may be smaller than the request buffer. */
  DEV_CHAR_READ_FRAME = _DEV_CHAR_FRAME,
  /** Write a frame to the device. This operation is only
      supported by devices working with framed data. The size of the
      request may be limited by the maximum frame size supported by
      the device. */
  DEV_CHAR_WRITE_FRAME = _DEV_CHAR_WRITE | _DEV_CHAR_FRAME,
};

struct dev_char_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  /** request type */
  enum dev_char_rq_type_e type;

  /** character buffer, updated by the driver to point to the next
      unprocessed character. */
  uint8_t *data;
  /** character buffer size, updated by the driver. */
  size_t size;

  /** error code set by driver */
};

DEV_REQUEST_INHERIT(char); DEV_REQUEST_QUEUE_OPS(char);

/** @see dev_char_request_t */
#define DEV_CHAR_REQUEST(n)                                             \
  void (n)(                                                            \
    const struct device_char_s *accessor,                                  \
    struct dev_char_rq_s *rq)

/**
   Char device class request() function type. Enqueue a read or write request.

   The @tt type, @tt data and @tt size fields of the request must be
   initialized. The size can not be 0. The @tt error field of the
   request is updated when the request completes.

   When no error is reported, the @tt data and @tt size fields are
   updated in order to indicate what is left to transfer. These fields
   are left in an undefined state when an error is reported.

   The following error codes are valid:

   @list
     @item @tt -ENOENT: The targeted sub-device does not exists.
       This is only reported when the driver implements sub-devices.
     @item @tt -ENOTSUP: The request operation is not supported by
       the device.
     @item @tt -EIO: Temporary or permanent hardware error.
     @item @tt -EPIPE: Data have been lost due to a buffer overflow or
       a sync error condition. This error is reported once.
     @item @tt -ENOSPC: The request date size is too small to store the
       incoming frame or the outgoing frame is to large to be processed
       by the device. This error is reported once.
     @item @tt -EBADDATA: The incoming data has been dropped due to a
       bad parity or bad checksum condition. This error is reported once.
   @end list

   The kroutine of the request may be executed from within this
   function. Please read @xref {Nested device request completion}.

   @param dev pointer to device descriptor
   @param rq pointer to request.
*/
typedef DEV_CHAR_REQUEST(dev_char_request_t);

/** @see dev_char_cancel_t */
#define DEV_CHAR_CANCEL(n)                                             \
  error_t (n)(                                                            \
    const struct device_char_s *accessor,                                  \
    struct dev_char_rq_s *rq)
/**
   Char device class cancel() function type. Cancel a request.

   This function cancels a request which have previously been passed to
   the @ref dev_char_request_t function.

   The function returns 0 if the request has been cancelled or @tt
   -EBUSY if the request has already ended or will terminate very
   soon. It may also return @tt -ENOTSUP. The request kroutine is not
   executed when this function returns 0.

   When canceling a request of types @ref DEV_CHAR_READ or @ref
   DEV_CHAR_READ_FRAME, data which might have already been transferred
   will be lost.

   When canceling a request of types @@ref DEV_CHAR_WRITE or @ref
   DEV_CHAR_WRITE_FRAME, some data might have been sent partially.

   The @tt size and @tt data fields of a cancelled request have
   undefined values.
*/
typedef DEV_CHAR_CANCEL(dev_char_cancel_t);

DRIVER_CLASS_TYPES(DRIVER_CLASS_CHAR, char,
                   dev_char_request_t *f_request;
                   dev_char_cancel_t *f_cancel;
                   );

/** @see driver_char_s */
#define DRIVER_CHAR_METHODS(prefix)                                \
  ((const struct driver_class_s*)&(const struct driver_char_s){    \
    .class_ = DRIVER_CLASS_CHAR,                                   \
    .f_request = prefix ## _request,                               \
    .f_cancel = prefix ## _cancel,                               \
  })

DEV_REQUEST_WAIT_FUNC(char);

/** Blocking helper function. This function uses the scheduler api to
    put current context in wait state until the operation completes.
    @returns transferred size or -1 in case of error.
    @csee dev_char_wait_rq */
config_depend_and2_inline(CONFIG_DEVICE_CHAR, CONFIG_MUTEK_CONTEXT_SCHED,
ssize_t dev_char_wait_op(const struct device_char_s *accessor,
                         enum dev_char_rq_type_e type, uint8_t *data, size_t size),
{
  struct dev_char_rq_s rq;
  error_t err;

  rq.type = type;
  rq.data = data;
  rq.size = size;

  err = dev_char_wait_rq(accessor, &rq);
  assert(err <= 0);
  return err < 0 ? err : size - rq.size;
})

#endif
