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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2016-2022
*/

/**
   @file
   @module {Core::Devices support library}
   @short Sound device driver API
   @index {PCM} {Device classes}
   @csee DRIVER_CLASS_PCM

   @section {Purpose}

   This class provides access to isochronous sample streams taken at a
   constant sampling rate.  This is most usually suited for sound
   devices, but may be used for other equivalent data streams.

   Because this kind of device cannot tolerate software to be late
   (there is no backpressure mechanism), it uses the request kroutine
   to indicate requirement for software to refill buffers.

   Buffers are allocated by the device, either to general purpose
   system RAM, or in dedicated memory space.  This is up to the
   driver.

   Application may require multiple streams to be handled
   synchronously by the device.  Either all in the same direction or
   not.  Because some hardware will require samples to be interleaved,
   driver can enforce data sample stride it needs.  It is up to the
   application to adapt the stream format.

   @end section
 */

#ifndef __DEVICE_PCM_H__
#define __DEVICE_PCM_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>
#include <device/request.h>

struct driver_s;
struct dev_pcm_rq_s;
struct driver_pcm_s;
struct device_pcm_s;

/**
   @this defines the operation type.
 */
enum dev_pcm_op_type_e
{
  DEV_PCM_OP_INIT,
  DEV_PCM_OP_STREAM,
  DEV_PCM_OP_CLOSE,
};

/**
   @this defines the sample format in sample buffers.
 */
enum dev_pcm_data_type_e
{
  DEV_PCM_DT_INT8,
  DEV_PCM_DT_UINT8,
  DEV_PCM_DT_INT16LE,
  DEV_PCM_DT_UINT16LE,
  DEV_PCM_DT_INT16BE,
  DEV_PCM_DT_UINT16BE,
  DEV_PCM_DT_INT32LE,
  DEV_PCM_DT_UINT32LE,
  DEV_PCM_DT_INT32BE,
  DEV_PCM_DT_UINT32BE,
};

/**
   @this defines the stream direction.  Direction name is oriented
   from device's perspective towards exterior of the chip.
 */
enum dev_pcm_data_direction_e
{
  DEV_PCM_DIR_INPUT,
  DEV_PCM_DIR_OUTPUT,
};

struct dev_pcm_stream_s {
  void *buffer;
  uint8_t stride;

  enum dev_pcm_data_type_e sample_type : 4;
  enum dev_pcm_data_direction_e direction : 1;
  uint8_t channel_id:3;
};

/**
   @this is the PCM request structure.  Application must fill it with
   @tt session_id = 0, a @tt sample_rate, @tt sample_count, @tt
   stream_count, and for each stream, @tt sample_type, @tt direction
   and @tt channel_id, then request with op = .._INIT.

   If it accepts the request, driver will fill @tt session_id, give
   @tt effective_sample_rate, allocate buffers internally, and give
   one buffer pointer per stream in @tt buffer and fill in @tt stride
   for each stream.

   Application should then fill / retrieve every sample in each stream
   pointed by @tt buffer / @tt stride / @tt sample_count and resubmit
   the request with op = .._STREAM.

   When done with streaming, application should submit one last
   request with op = .._CLOSE.

   After call to _INIT and _STREAM, buffer pointer may change and
   application should use the currently-pointed buffers. These may be
   offline DMA-able buffers managed by the driver.

   @tt session_id should remain constant for every subsequent _STREAM
   and _CLOSE operations.
 */
struct dev_pcm_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  enum dev_pcm_op_type_e op;
  uint32_t session_id;

  uint32_t sample_rate;
  uint32_t effective_sample_rate;
  size_t stream_count;
  size_t sample_count;

  struct dev_pcm_stream_s stream[0];
};

DEV_REQUEST_INHERIT(pcm); DEV_REQUEST_QUEUE_OPS(pcm);

/** @see dev_pcm_request_t */
#define DEV_PCM_REQUEST(n)                                               \
  error_t (n)(const struct device_pcm_s *accessor,                     \
           struct dev_pcm_rq_s *rq)

/**
   Pcm device class request() function type.  This function allocates
   the buffers for all streams and starts streaming.  Request kroutine
   will be called periodically until request is stopped or an error
   occurs.

   @tt frames_left must be non-zero upon calling.

   @param dev Pointer to device descriptor
   @param rq Pointer to request
   @returns 0 if request is blocking, -ENOTSUP/-EINVAL/-ENOMEM if some
            parameters are not possible
*/
typedef DEV_PCM_REQUEST(dev_pcm_request_t);

DRIVER_CLASS_TYPES(DRIVER_CLASS_PCM, pcm,
                   dev_pcm_request_t *f_request;
                   );

/** @see driver_pcm_s */
#define DRIVER_PCM_METHODS(prefix)                                \
  ((const struct driver_class_s*)&(const struct driver_pcm_s){    \
    .class_ = DRIVER_CLASS_PCM,                                   \
    .f_request = prefix ## _request,                              \
  })

#ifdef CONFIG_DEVICE_PCM
/** @This provides a @ref DEV_RES_DEV_PARAM resource entry which
    specifies a dependency on a PCM device. */
# define DEV_STATIC_RES_DEV_PCM(path_)                          \
  DEV_STATIC_RES_DEVCLASS_PARAM("pcm", path_, DRIVER_CLASS_PCM)
#else
/** @hidden */
# define DEV_STATIC_RES_DEV_PCM(path_)                                  \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }
#endif

#endif
