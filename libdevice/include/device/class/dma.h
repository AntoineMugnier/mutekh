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
 * @short DMA device driver API
 */                                                                 

#ifndef __DEVICE_DMA_H__
#define __DEVICE_DMA_H__

#include <mutek/kroutine.h>
#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>
#include <device/class/timer.h>
#include <device/request.h>


enum dev_dma_inc_e
{
  DEV_DMA_INC_1_BYTE,
  DEV_DMA_INC_2_BYTE,
  DEV_DMA_INC_4_BYTE,
  DEV_DMA_INC_NONE
};

struct device_dma_s;

/** Dma request @see devdma_request_t */
struct dev_dma_rq_s
{
  struct dev_request_s        base;

  size_t                      size;

  uintptr_t                   src;

  uintptr_t                   dst;

  enum dev_dma_inc_e          src_inc:2;
  enum dev_dma_inc_e          dst_inc:2;

  uint8_t                     channel;        //< Channel for this request */
  uint8_t                     const_data:1;   //< Constant data */
  uint8_t                     arch_rq:1;      //< Architecture dependant request */

  error_t                     err;            //< error code set by driver

  const struct device_dma_s  *accessor;       //< associated dma device
};

STRUCT_INHERIT(dev_dma_rq_s, dev_request_s, base);

/** Dma device class @ref devdma_request_t function template. */
#define DEVDMA_REQUEST(n)	void (n) (const struct device_dma_s *accessor, struct dev_dma_rq_s *req)

/**
   Dma device class request() function type. Enqueue a dma transfert
   request.

   @ref src_inc and @ref dst_inc defines the incrementation step of
   respectively @ref src and @ref dst addresses. When
   @ref DEV_DMA_INC_NONE is used, address must not be incremented
   by DMA. This is useful for FIFO access.
   When @ref const_data field is set, src field points to the constant
   data to copy repeatedly to the destination and the @ref const_len
   field must be used to indicate length of the pattern.
   @ref channel specifies which channel must be used for the transfert.
   On mono channel DMA's this field must be set to 0.
 
   When @ref arch_rq field is set, @ref struct dev_dma_rq_s contains
   some fields that are platform specific. For example, DMA triggering
   signal or peripheral ID when transfer to/from a peripheral can be
   part of @ref struct dev_dma_rq_s.

   @ref err returns @tt -ENOTSUP when the requested operation is not
   supported and @tt 0 otherwise.
*/

typedef DEVDMA_REQUEST(devdma_request_t);


DRIVER_CLASS_TYPES(dma, 
                   devdma_request_t *f_request;
                   );

inline error_t dev_dma_spin_copy(struct dev_dma_rq_s* rq)
{
    struct dev_request_status_s status;

    dev_request_spin_init(&rq->base, &status);

    DEVICE_OP(rq->accessor, request, rq);

    dev_request_spin_wait(&status);

    return rq->err;
}

#ifdef CONFIG_MUTEK_SCHEDULER
inline error_t dev_dma_wait_copy(struct dev_dma_rq_s* rq)
{
    struct dev_request_status_s status;

    dev_request_sched_init(&rq->base, &status);

    DEVICE_OP(rq->accessor, request, rq);

    dev_request_sched_wait(&status);

    return rq->err;
}
#endif

#ifdef CONFIG_DEVICE_DMA
/** @This can be used to include a DMA resource entry in a static
    device resources table declaration.*/
# define DEV_STATIC_RES_DMA(label_, channel_, config_)    \
  {                                                       \
    .flags = DEVICE_RES_FLAGS_DEPEND,                     \
      .type = DEV_RES_DMA,                                \
        .u = { .dma = {                                   \
          .label = (label_),                              \
          .channel = (channel_),                          \
          .config = (config_),                            \
        } }                                               \
  }
#else
# define DEV_STATIC_RES_DMA(label_, channel_, config_)   \
  {                                                      \
    .type = DEV_RES_UNUSED,                              \
  }
#endif

#endif

