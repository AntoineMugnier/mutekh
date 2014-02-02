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

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_clist.h>

#include <device/driver.h>

struct dev_dma_rq_s;

/** Dma device read/write callback */
#define DEVDMA_CALLBACK(n) void (n) (const struct dev_dma_rq_s *rq)

/**
   Dma device callback.

   @param dev pointer to device descriptor
   @param rq pointer to request data.
*/
typedef DEVDMA_CALLBACK(devdma_callback_t);

/** @This specifies use of fixed source address for dma transfer. */
#define DEV_DMA_FLAG_FIXED_SRC    0x01
/** @This specifies use of fixed destination address for dma transfer. */
#define DEV_DMA_FLAG_FIXED_DST    0x02
/** @This specifies use of constant source data for dma transfer. @see #DEV_DMA_FLAG_CONST_LEN */
#define DEV_DMA_FLAG_CONST_DATA   0x04
/** @This specifies len of constant source data for dma transfer. @see #DEV_DMA_FLAG_CONST_DATA */
#define DEV_DMA_FLAG_CONST_LEN(n) ((n) << 3)

/** Dma request @see devdma_request_t */
struct dev_dma_rq_s
{
  size_t			size;

  /** Source address in source address space */
  uintptr_t			src;
  /** Destination address in destination address space */
  uintptr_t          		dst;

#ifdef CONFIG_DEVICE_ADDRESS_SPACES
  /** Source address space identifier */
  address_space_id_t            src_as;
  /** Destination address space identifier */
  address_space_id_t            dst_as;
#endif

  uint_fast8_t                  flags;

  devdma_callback_t		*callback;      //< callback function
  void				*pvdata;        //< private data for callback

  error_t			error;          //< error code set by driver

  const struct device_dma_s     *ddev;          //< associated dma device
  void				*drvdata;       //< driver private data
  CONTAINER_ENTRY_TYPE(CLIST)   queue_entry; //< used by driver to enqueue request
};

CONTAINER_TYPE(dev_dma_queue, CLIST, struct dev_dma_rq_s, queue_entry);
CONTAINER_FUNC(dev_dma_queue, CLIST, static inline, dev_dma_queue);


/** Dma device class @ref devdma_request_t function template. */
#define DEVDMA_REQUEST(n)	error_t  (n) (const struct device_dma_s *ddev, struct dev_dma_rq_s *rq)

/**
   Dma device class request() function type. Enqueue a dma transfert request.

   The dma request flags can be set to perform a DMA transfert with
   fixed source address, fixed destination address or constant source
   data. When the @ref #DEV_DMA_FLAG_FIXED_DATA flag is used, the @tt
   src field points to the constant data to copy repeatedly to the
   destination and the @ref #DEV_DMA_FLAG_CONST_LEN macro must be used
   to indicate length of the pattern.

   @param dev pointer to device descriptor
   @param rq pointer to request. src, dst, size, flags and callback must be initialized.
   @returns zero if the request has been enqueued or @tt -ENOTSUP if the requested operation is not supported.
*/
typedef DEVDMA_REQUEST(devdma_request_t);


DRIVER_CLASS_TYPES(dma, 
                   devdma_request_t *f_request;
                   );

/** Synchronous dma helper function. This function uses the scheduler
    api to put the current context in wait state until the DMA
    transfer has completed. This function spins in a loop waiting for
    DMA transfer operation to complete when scheduler is disabled.

    @returns error code.
*/
config_depend(CONFIG_DEVICE_DMA)
error_t dev_dma_wait_copy(const struct device_dma_s *ddev,
                          uintptr_t src, uintptr_t dst,
                          size_t size, uint_fast8_t flags);

/** Synchronous dma helper function. This function spins in a loop
    waiting for DMA transfer operation to complete.

    @returns error code.
*/
config_depend(CONFIG_DEVICE_DMA)
error_t dev_dma_spin_copy(const struct device_dma_s *ddev,
                          uintptr_t src, uintptr_t dst,
                          size_t size, uint_fast8_t flags);

/** Same as @ref dev_dma_wait_copy with explicit address spaces for
    source and destination addresses. */
config_depend_and2(CONFIG_DEVICE_DMA, CONFIG_DEVICE_ADDRESS_SPACES)
error_t dev_dma_wait_copy_as(const struct device_dma_s *ddev,
                             uintptr_t src, uintptr_t dst,
                             size_t size, uint_fast8_t flags,
                             address_space_id_t src_as,
                             address_space_id_t dst_as);

/** Same as @ref dev_dma_spin_copy with explicit address spaces for
    source and destination addresses. */
config_depend_and2(CONFIG_DEVICE_DMA, CONFIG_DEVICE_ADDRESS_SPACES)
error_t dev_dma_spin_copy_as(const struct device_dma_s *ddev,
                             uintptr_t src, uintptr_t dst,
                             size_t size, uint_fast8_t flags,
                             address_space_id_t src_as,
                             address_space_id_t dst_as);

#endif

