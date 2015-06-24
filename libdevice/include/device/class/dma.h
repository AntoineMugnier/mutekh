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

enum dev_dma_intl_e
{
  DEV_DMA_INTL_READ = 0, 
  DEV_DMA_INTL_WRITE
};

enum dev_dma_transfer_type_e
{
/** @This is the basic DMA transfer mode. Copy a memory block of size @tt size
    from @tt src to @tt dst. */
  DEV_DMA_BASIC,
/** @This is used to write data continously to a destination address @tt dst.
    DMA transfer source swicthes alternatively between @tt src[0] and @tt src[1].
    @tt count is decremented and kroutine is called each time DMA 
    terminates a write operation. When kroutine is called with @tt count null,
    DMA transfer is terminated and request will be not used anymore by driver.
    If @tt count is not null when kroutine is called, it can be incremented by the
    kroutine to continue operation as long as necessary. In the same way, @tt dst
    can be modified by kroutine to write contigous memory area. Kroutine policy must
    always be @ref KROUTINE_IMMEDIATE. @tt dst[0] and @tt dst[1] must be equal. */
  DEV_DMA_DBL_BUF_SRC,
/** @This is used to read data continously from a source address @tt src.
    DMA transfer destination swicthes alternatively between @tt dst[0] and @tt dst[1].
    @tt count is decremented and kroutine is called each time DMA terminates
    a write operation. When kroutine is called with @tt count null, DMA transfer is
    terminated and request will be not used anymore by driver. If @tt count is not null
    when kroutine is called, it can be incremented by the kroutine to continue operation
    as long as necessary. In the same way, @tt dst can be modified by kroutine to write
    contigous memory area. Kroutine policy must always be @ref KROUTINE_IMMEDIATE. 
    @tt src[0] and @tt src[1] must be equal. */
  DEV_DMA_DBL_BUF_DST,
/** @This is used to chain a list of DMA operation on a channel. The kroutine
    is called when all operation have been performed. This can be useful to move multiple
    memory blocks into one contiguous memory area. The number of operation to perform
    @tt count is decremented each time an operation is terminated.*/
  DEV_DMA_SCATTER_GATHER,
/** @This is used when 2 DMA operations must be synchonized. This can be useful to
    perform a SPI transfer. In this case, a first DMA operation is in charge of writing
    data to SPI device and a second DMA operation is in charge of reading data from SPI.
    The read DMA operation must be started before the write operation to avoid loosing data.
    @ref tr[0] and param[0] are used for read operation and @tt ref tr[1] and param[1]
    are used for write operation. The kroutine is called only when the read operation is
    finished. */
  DEV_DMA_INTERLEAVED,
};

struct dev_dma_entry_s
{
  size_t                      size;
  uintptr_t                   src;
  uintptr_t                   dst;
};

struct dev_dma_param_s
{
  uint8_t                       channel;
  enum dev_dma_inc_e            src_inc:2;
  enum dev_dma_inc_e            dst_inc:2;
  uint8_t                       const_data:1;  
};

struct device_dma_s;

/** Dma request @see devdma_request_t */
struct dev_dma_rq_s
{
  struct dev_request_s          base;

  enum dev_dma_transfer_type_e  type;

  union
  {
    /** Used with @ref DEV_DMA_BASIC */
    struct dev_dma_entry_s      basic;

    /** Used with @ref DEV_DMA_INTERLEAVED */
    /** Used with @ref DEV_DMA_DBL_BUF_SRC */
    /** Used with @ref DEV_DMA_DBL_BUF_DST */

    struct dev_dma_entry_s      tr[2];

    /** Used with @ref DEV_DMA_SCATTER_GATHER */
    struct dev_dma_entry_s   ** sg;
  };

 
  struct dev_dma_param_s        param[2];

  uint16_t                      count;
  uint8_t                       arch_rq:1;
                             
  error_t                       error;    
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
   by DMA. @ref src and @ref dst can be modified by driver.
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

#define DRIVER_DMA_METHODS(prefix)                               \
  &(const struct driver_dma_s){                                  \
    .class_ = DRIVER_CLASS_DMA,                                  \
    .f_request = prefix ## _request,                             \
  }

inline error_t dev_dma_spin_copy(const struct device_dma_s *accessor, 
                                 struct dev_dma_rq_s* rq)
{
    struct dev_request_status_s status;

    dev_request_spin_init(&rq->base, &status);

    DEVICE_OP(accessor, request, rq);

    dev_request_spin_wait(&status);

    return rq->error;
}

#ifdef CONFIG_MUTEK_SCHEDULER
inline error_t dev_dma_wait_copy(const struct device_dma_s *accessor,
                                 struct dev_dma_rq_s* rq)
{
    struct dev_request_status_s status;

    dev_request_sched_init(&rq->base, &status);

    DEVICE_OP(accessor, request, rq);

    dev_request_sched_wait(&status);

    return rq->error;
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

