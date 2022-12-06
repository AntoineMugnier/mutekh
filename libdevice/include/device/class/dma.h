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
  @short Direct memory access driver API
  @index {Direct memory access} {Device classes}
  @csee DRIVER_CLASS_DMA

  This class of device provides a generic API for accessing DMA
  controllers from other device drivers.

  Transfer between memory and device registers are both supported. A
  @ref dev_dma_rq_s {DMA operation} request may perform multiple
  transfers of the same @ref dev_dma_rq_type_e {type}. A request
  embeds an array of @ref dev_dma_desc_s {transfer descriptors}. Each
  transfer entry specifies the addresses, size, stride and access
  width that must be used to perform the copy. It is possible to loop
  over the array of descriptors during a single request.

  These modes of operation are supported:
  @list
  @item In continuous mode, a callback is invoked after
    each transfer of the request, letting the initiator decide if the
    transfer must go on.
  @item In single shot mode, a callback is invoked when the whole
    operation is over. A loop count is provided when the requested
    is submitted, allowing the hardware to perform the whole operation
    without software intervention.
  @end list

  A per request channel mask is used to specifies the channels that
  can be used to perform an operation.
*/

#ifndef __DEVICE_DMA_H__
#define __DEVICE_DMA_H__

#include <mutek/kroutine.h>
#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>
#include <device/class/timer.h>
#include <device/request.h>

struct device_dma_s;

/** @internal source is a device register */
#define _DEV_DMA_SRC_REG         1
/** @internal source is a memory buffer */
#define _DEV_DMA_SRC_MEM         2
/** @internal destination is a device register */
#define _DEV_DMA_DST_REG         4
/** @internal destination is a memory buffer */
#define _DEV_DMA_DST_MEM         8
/** @internal loop an indefinite number of times over descriptors */
#define _DEV_DMA_CONTINUOUS      16

/** @This specifies the type of dma operation.
    @see dev_dma_rq_s */
enum dev_dma_rq_type_e
{
  /** @This type of request performs some memory to memory
      transfers. All fields of the descriptors in @tt {src.mem} and
      @tt {dst.mem} must be initialized. The request terminates and
      the callback is invoked when the last descriptor is reached and
      the loop counter reaches 0. */
  DEV_DMA_MEM_MEM = _DEV_DMA_SRC_MEM | _DEV_DMA_DST_MEM,
  /** @This type of request performs some memory to device register
      transfers. Fields of the descriptors in @tt {src.mem} and @tt
      {dst.reg} must be initialized. The request terminates and the
      callback is invoked when the last descriptor is reached and
      the loop counter reaches 0. */
  DEV_DMA_MEM_REG = _DEV_DMA_SRC_MEM | _DEV_DMA_DST_REG,
  /** @This type of request performs an indefinite number of memory to
      device register transfers. All fields of the descriptors in @tt
      {src.mem} and @tt {dst.reg} must be initialized. The callback is
      invoked after each transfer. The request terminates when the
      callback returns 0. */
  DEV_DMA_MEM_REG_CONT = _DEV_DMA_SRC_MEM | _DEV_DMA_DST_REG | _DEV_DMA_CONTINUOUS,
  /** @This type of request performs some device register to memory
      transfers. Fields of the descriptors in @tt {src.reg} and @tt
      {dst.mem} must be initialized. The request terminates and the
      callback is invoked when the last descriptor is reached and
      the loop counter reaches 0. */
  DEV_DMA_REG_MEM = _DEV_DMA_SRC_REG | _DEV_DMA_DST_MEM,
  /** @This type of request performs an indefinite number of device
      register to memory transfers. All fields of the descriptors in
      @tt {src.reg} and @tt {dst.mem} must be initialized. The
      callback is invoked after each transfer. The request terminates
      when the callback returns 0. */
  DEV_DMA_REG_MEM_CONT = _DEV_DMA_SRC_REG | _DEV_DMA_DST_MEM | _DEV_DMA_CONTINUOUS,
  /** @This type of request performs some device register to device
      register transfers. Fields of the descriptors in @tt {src.reg}
      and @tt {dst.reg} must be initialized. The request terminates
      and the callback is invoked when the last descriptor is reached
      and the loop counter reaches 0. */
  DEV_DMA_REG_REG = _DEV_DMA_SRC_REG | _DEV_DMA_DST_REG,
  /** @This type of request performs an indefinite number of device
      register to device register transfers. All fields of the
      descriptors in @tt {src.reg} and @tt {dst.mem} must be
      initialized. The callback is invoked after each transfer. The
      request terminates when the callback returns 0. */
  DEV_DMA_REG_REG_CONT = _DEV_DMA_SRC_REG | _DEV_DMA_DST_REG | _DEV_DMA_CONTINUOUS,
};

enum dev_dma_inc_e
{
  DEV_DMA_INC_0_UNIT  = 0,
  DEV_DMA_INC_1_UNITS = 1,
  DEV_DMA_INC_2_UNITS = 2,
  DEV_DMA_INC_4_UNITS = 3,
};

# define DEV_DMA_GET_INC(inc) ((inc) == DEV_DMA_INC_4_UNITS ? 4 : (inc))


/** @This specifies parameter of a single DMA transfer. This is used
    as an array element in the @ref dev_dma_rq_s structure. */
struct dev_dma_desc_s
{
  union {
    struct {
      /** This specifies the address of the source memory buffer.
          This field may be modified by the driver. */
      uintptr_t   addr;
      /** This specifies the number of units minus one to transfer from memory. */
      uint16_t    size;
      /** This specifies the log2 of the memory load access width in bytes.
          This defines the unit size. */
      uint16_t    BITFIELD(width, 2);
      /** This specifies the memory source address increment in units */
      enum dev_dma_inc_e inc:2;
      /** This specifies the number of units to add to the source address pointer
          when looping. */
      uint16_t    BITFIELD(stride,12);
    }            mem;
    struct {
      /** This specifies the address of the device to load from register. */
      uintptr_t   addr;
      /** This specifies the number of units minus one to transfer from memory. */
      uint16_t    size;
      /** This specifies the log2 of the register load access width in bytes.
          This defines the unit size. */
      uint16_t    BITFIELD(width,2);
      uint16_t    BITFIELD(_unused,2);
      /** This specifies the number of units that is transfered at once from
          the device. Value 0 means that all units can be transfered at once */
      uint16_t    BITFIELD(burst,12);
    }            reg;
  }             src;

  union {
    struct {
      /** This specifies the address of the destination memory buffer.
          This field may be modified by the driver. */
      uintptr_t   addr;
      /** This specifies the memory destination address increment in word unit */
      enum dev_dma_inc_e inc:2;
      uint16_t    BITFIELD(_unused,2);
      /** This specifies the number of units to add to the destination address pointer
          when looping. */
      uint16_t    BITFIELD(stride,12);
    }            mem;
    struct {
      /** This specifies the address of the device register to write to. */
      uintptr_t   addr;
      uint16_t    BITFIELD(_unused,4);
      /** This specifies the number of units that is transfered at once to
          the device. Value 0 means that all units can be transfered at once */
      uint16_t    BITFIELD(burst,12);
    }            reg;
  }             dst;
};

struct dev_dma_rq_s;

/** @see dev_dma_callback_t */
#define DEV_DMA_CALLBACK(n) bool_t (n)(struct dev_dma_rq_s *rq, \
                                       uint_fast16_t desc_id, \
                                       error_t err)

/** @This is called when a DMA transfer has completed.

    Depending on the @ref dev_dma_rq_type_e {type} of operation
    requested, this will be called multiple times during a single
    operation:

    @list
      @item When a continuous DMA operation has been requested, this function
      is called for each completed descriptor. The @tt desc_id indicates the
      completed descriptor. The DMA operation terminates when this callback
      returns 0. The driver loops over the array of descriptors until this
      happens.
      @item For other type of operations, this callback is called once
      when the operation terminates as the last descriptor is reached
      and the loop counter is decremented to 0.
    @end list

    In any case, the @tt err parameter is not 0, the operation is
    terminated ans the callback will not be called anymore.

    It is not permitted to call the devdma_request_t function in order
    to start a new operation from this callback. This callback may be
    invoked from the interrupt handler of the DMA controller. Deferred
    execution must be implemented as needed. */

typedef DEV_DMA_CALLBACK(dev_dma_callback_t);

#define GCT_CONTAINER_ALGO_dev_dma_queue CLIST

/** @This is the DMA operation object. The array of DMA transfer
    descriptors declared in this structure has null size. The @ref
    #DEV_DMA_RQ_TYPE macro must be used instead to declare a DMA
    operation embedding some transfer descriptors. */
struct dev_dma_rq_s
{
  /** Type of DMA operation requested */
  enum dev_dma_rq_type_e        type:5;

  /** Reserved for use by the driver of the DMA controller */
  uint8_t                       drv_pv:5;

  /** Number of descriptors embedded at the end of the object minus one */
  uint8_t                       desc_count_m1:6;

  /** Number of iterations over the list of descriptors when not in
      continuous mode minus one. */
  uint16_t                      loop_count_m1;

  /** Mask of channels the operation is allowed to execute on. This is
      used for channel allocation. It should be specified in resources
      of the device relying on the DMA. */
  uint32_t                      chan_mask;

  /** This identifies a link between the device and the DMA
      controller. The exact meaning is hardware dependent. */
  struct {
    uint16_t                    src;
    uint16_t                    dst;
  }                             dev_link;

  /** Completion callback. */
  dev_dma_callback_t            *f_done;

  union {
    GCT_CONTAINER_ENTRY(dev_dma_queue, queue_entry);
    struct {
      /** Accumulated number of unit transferred by DMA. For continous
          operation, it contains number of transferred units since last dma
          callback */
      uint32_t                   size;
      /** Index of last descriptor processed by DMA. */
      uint32_t                   desc_idx;
    }                            cancel;
  };

  /** This may be used by the driver in order to reuse some internal
      data structure associated to a repeated request. It must be set
      to @tt NULL before submitting a new request. It should be left
      untouched when the same request is submitted again. Only the @tt
      {mem.addr}, @tt {mem.size} and @tt {mem.stride} fields of the
      request descriptor are allowed to change between submissions
      when not reset to @tt NULL. */
  void                          *cache_ptr;

  /** Array of transfer descriptors. The driver is allowed to modify
      the @tt {src.mem.addr} and @tt {dst.mem.addr} fields during
      execution of the request. */
  struct dev_dma_desc_s        desc[0];
};

GCT_CONTAINER_TYPES(dev_dma_queue, struct dev_dma_rq_s *, queue_entry);
GCT_CONTAINER_FCNS(dev_dma_queue, static inline, dev_dma_queue,
                   init, destroy, pushback, remove, isempty, head);

/** @This declare a @ref dev_dma_rq_s object type with an embedded
    array of @ref dev_dma_desc_s at the end. */
#define DEV_DMA_RQ_TYPE(n)                      \
  struct {                                      \
    struct dev_dma_rq_s rq;                     \
    struct dev_dma_desc_s desc[n];              \
  }

/** @see devdma_request_t */
#define DEVDMA_REQUEST(n)	__attribute__ ((sentinel)) \
  error_t (n) (const struct device_dma_s *accessor, ...)

/**
   Dma device class request() function type.

   @This enqueues one or more @ref dev_dma_rq_s objects. The argument
   list of pointers to request must be @tt NULL terminated.

   When multiple requests are passed on the same calls, they are
   guaranteed to be started in order. This means that a request
   sending data to a device might be postponed until an other
   request is ready to sink data from the device.

   The @ref dev_dma_rq_s::cache_ptr field of the request must be set
   to @tt NULL unless the same request has been pushed previously.

   Execution of the request consists in performing multiple transfers
   specified in the embedded descriptors. The @ref dev_dma_rq_type_e
   {type} of request specifies the exact behavior. The @tt
   {src.mem.addr} and @tt {dst.mem.addr} fields of request descriptors
   are left undefined when the request terminates. Other fields of the
   descriptors are not modified.

   A request will not start until one of the channel specified in @tt
   chan_mask becomes idle. It returns @tt -ENOENT if the channel
   mask does not have at least one bit set for an implemented channel.

   The function returns @tt -ENOTSUP when the requested type of
   operation is not supported.

   If a descriptor related issue is detected (unsupported alignment,
   non mapped address...), the error is reported to the callback
   function instead.
*/
typedef DEVDMA_REQUEST(devdma_request_t);

/** @see dev_dma_cancel_t */
#define DEV_DMA_CANCEL(n)	error_t  (n) (struct device_dma_s *accessor, \
                                              struct dev_dma_rq_s *rq)

/**
   @This cancel a dma request.

   The function returns 0 if the request has been cancelled or @tt
   -EBUSY if the request has already ended or will terminate very
   soon. It may also return @tt -ENOTSUP. The DMA callback is not
   executed when this function returns 0.

   In case of success the @ref cancel structure field of the request
   contains information about cancelled request
*/

typedef DEV_DMA_CANCEL(devdma_cancel_t);

struct dev_dma_status_s
{
  uintptr_t   src_addr;
  uintptr_t   dst_addr;
};

/** @see dev_dma_status_t */
#define DEV_DMA_GET_STATUS(n)	error_t  (n) (struct device_dma_s *accessor,    \
                                              struct dev_dma_rq_s *rq,          \
                                              struct dev_dma_status_s * status)

/**
   @This get status of a dma request.

   In case of success the @ref cancel structure field of the request
   contains information about cancelled request
*/

typedef DEV_DMA_GET_STATUS(devdma_get_status_t);

DRIVER_CLASS_TYPES(DRIVER_CLASS_DMA, dma,
                   devdma_request_t *f_request;
                   devdma_cancel_t *f_cancel;
                   devdma_get_status_t *f_get_status;
                   );

/** @see driver_dma_s */
#define DRIVER_DMA_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_dma_s){   \
    .class_ = DRIVER_CLASS_DMA,                                  \
    .f_cancel = prefix ## _cancel,                               \
    .f_request = prefix ## _request,                             \
    .f_get_status = prefix ## _get_status,                       \
  })

#ifdef CONFIG_DEVICE_DMA

/** @This specifies a DMA resource entry in a static
    device resources table declaration. @csee DEV_RES_DMA
    @see #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_DMA(channel_mask_, link_)    \
  {                                                       \
      .type = DEV_RES_DMA,                                \
        .u = { .dma = {                                   \
          .channel_mask = (channel_mask_),                \
          .link = (link_)                                 \
        } }                                               \
  }

# define DEV_STATIC_RES_DEV_DMA(path_)                    \
  DEV_STATIC_RES_DEVCLASS_PARAM("dma", path_, DRIVER_CLASS_DMA)

#else

# define DEV_STATIC_RES_DMA(channel_mask_, link_)        \
  {                                                      \
    .type = DEV_RES_UNUSED,                              \
  }

# define DEV_STATIC_RES_DEV_DMA(path_)                   \
  {                                                      \
    .type = DEV_RES_UNUSED,                              \
  }
#endif

config_depend_alwaysinline(CONFIG_DEVICE_DMA,
error_t device_res_get_dma(const struct device_s *dev, uint_fast16_t id,
                           uint32_t *channel_mask, uint32_t *link),
{
  struct dev_resource_s *r;

  if (!(r = device_res_get(dev, DEV_RES_DMA, id)))
    return -ENOENT;

  *channel_mask = r->u.dma.channel_mask;
  if (link != NULL)
    *link = r->u.dma.link;

  return 0;
})

#endif
