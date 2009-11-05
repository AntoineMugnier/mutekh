/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

/**
 * @file
 * @module{Devices}
 * @short Block device driver API
 */                                                                 

#ifndef __DEVICE_BLOCK_H__
#define __DEVICE_BLOCK_H__

#ifdef __DRIVER_H__
# error This header must not be included after "device/driver.h"
#endif

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/gpct_platform_hexo.h>
#include <gpct/cont_clist.h>

struct device_s;
struct driver_s;

typedef uint32_t dev_block_lba_t;
struct dev_block_rq_s;

/** Block device read/write callback */
#define DEVBLOCK_CALLBACK(n) void (n) (struct device_s *dev, \
				       const struct dev_block_rq_s *rq, \
				       size_t count)

/**
   Block device read callback function. This function is called for
   each group of blocks read. It may be called several times, @tt {rq->count != 0}
   and @tt rq->error must be used to detect operation end. The device
   lock is held when call back function is called.

   @param dev pointer to device descriptor
   @param rq pointer to request data. @tt rq->count field is
          updated with remaining blocks to process. @tt rq->lba is undefined.
	  @tt rq->data is advanced to the next block buffer. @tt rq->error is updated.
   @param count number of processed blocks
   @see #DEVBLOCK_CALLBACK
*/
typedef DEVBLOCK_CALLBACK(devblock_callback_t);

enum dev_block_rq_type_e
  {
    DEV_BLOCK_READ, DEV_BLOCK_WRITE,
  };

CONTAINER_TYPE(dev_blk_queue, CLIST,
struct dev_block_rq_s
{  
  enum dev_block_rq_type_e	type;
  dev_block_lba_t		lba;	/* lba */
  size_t			count;	/* block count */
  devblock_callback_t		*callback; /* callback function */
  void				*pvdata; /* pv data for callback */
  uint8_t			**data; /* table of pointer to data blocks */
  error_t			error; /* error code set by driver */

  void				*drvdata; /* driver private data */
  dev_blk_queue_entry_t		queue_entry; /* used by driver to enqueue requests */
}, queue_entry);

CONTAINER_FUNC(dev_blk_queue, CLIST, static inline, dev_blk_queue);




/** Block device class request() function tempate. */
#define DEVBLOCK_REQUEST(n)	void (n) (struct device_s *dev,	\
					  struct dev_block_rq_s *rq)

/** Block device class request() methode shortcut */

#define dev_block_request(dev, ...) (dev)->drv->f.blk.f_request(dev, __VA_ARGS__ )
/**
   Block device request function type. Request count data blocks
   from the device.

   @param rq pointer to request. lba, count, data and callback
             field must be intialized.
   @param dev pointer to device descriptor
   @param size max data request bytes count
   @see #DEVBLOCK_REQUEST
*/
typedef DEVBLOCK_REQUEST(devblock_request_t);




struct dev_block_params_s
{
  size_t		blk_size;	/* size of a single block */
  dev_block_lba_t	blk_count;	/* device blocks count */
};

/** Block device class getparams() function tempate. */
#define DEVBLOCK_GETPARAMS(n)	const struct dev_block_params_s * (n) (struct device_s *dev)

/** Block device class getparams() methode shortcut */

#define dev_block_getparams(dev) (dev)->drv->f.blk.f_getparams(dev)

/**
   Block device getparams function type.

   @param dev pointer to device descriptor
   @return pointer to device parameters structure
   @see #DEVBLOCK_GETPARAMS
*/
typedef DEVBLOCK_GETPARAMS(devblock_getparams_t);



/** Block device class methodes */
struct dev_class_block_s
{
  devblock_request_t		*f_request;
  devblock_getparams_t		*f_getparams;
};


/** Synchronous helper read function. This function use the scheduler
    api to put current context in wait state if no data is available
    from device yet. This function is equivalent to
    dev_block_spin_read() when scheduler is disabled.
*/
error_t dev_block_wait_read(struct device_s *dev, uint8_t **data, dev_block_lba_t lba, size_t count);

/** Synchronous helper read function. This function spin in a loop
    waiting for read operation to complete.
*/
error_t dev_block_spin_read(struct device_s *dev, uint8_t **data, dev_block_lba_t lba, size_t count);

/** Synchronous helper write function. This function use the scheduler
    api to put current context in wait state if no data is available
    from device yet. This function is equivalent to
    dev_block_spin_write() when scheduler is disabled.
*/
error_t dev_block_wait_write(struct device_s *dev, uint8_t **data, dev_block_lba_t lba, size_t count);

/** Synchronous helper write function. This function spin in a loop
    waiting for write operation to complete.
*/
error_t dev_block_spin_write(struct device_s *dev, uint8_t **data, dev_block_lba_t lba, size_t count);


#endif

