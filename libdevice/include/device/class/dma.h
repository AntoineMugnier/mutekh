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
 * @module{Device drivers}
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
#define DEVDMA_CALLBACK(n) bool_t (n) (const struct dev_dma_rq_s *rq)

/**
   Dma device callback.

   @param dev pointer to device descriptor
   @param rq pointer to request data.
*/
typedef DEVDMA_CALLBACK(devdma_callback_t);

CONTAINER_TYPE(dev_dma_queue, CLIST,
struct dev_dma_rq_s
{
  size_t			size;
  const uint8_t			*src;
  uint8_t			*dst;

  devdma_callback_t		*callback;      //< callback function
  void				*pvdata;        //< private data for callback

  error_t			error;          //< error code set by driver

  const struct device_dma_s     *ddev;          //< associated dma device
  void				*drvdata;       //< driver private data
  dev_dma_queue_entry_t	queue_entry;    //< used by driver to enqueue requests
}, queue_entry);

CONTAINER_FUNC(dev_dma_queue, CLIST, static inline, dev_dma_queue);


/** Dma device class @ref devdma_request_t function template. */
#define DEVDMA_REQUEST(n)	void  (n) (const struct device_dma_s *cdev, struct dev_dma_rq_s *rq)

/**
   Dma device class request() function type. Enqueue a dma transfert request.

   @param dev pointer to device descriptor
   @param rq pointer to request. data, size and callback, field must be intialized.
*/
typedef DEVDMA_REQUEST(devdma_request_t);


DRIVER_CLASS_TYPES(dma, 
                   devdma_request_t *f_request;
                   );

void dev_dma_wait_copy(const struct device_dma_s *cdev, const uint8_t *src, uint8_t *dst, size_t size);

void dev_dma_spin_copy(const struct device_dma_s *cdev, const uint8_t *src, uint8_t *dst, size_t size);

#endif

