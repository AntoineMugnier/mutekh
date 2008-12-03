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

#ifndef __DEVICE_BLOCK_H__
#define __DEVICE_BLOCK_H__

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
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
   each group of blocks read. It may be called several times, rq->count
   and rq->error must be used to detect operation end. The device
   lock is held when call back function is called.

   @param dev pointer to device descriptor
   @param rq pointer to request data. rq->count field is
          updated with remaining blocks to process. rq->lba is undefined.
	  rq->data is advanced to the next block buffer. rq->error is updated.
   @param count number of processed blocks
   @param data table of pointers to each block data
*/
typedef DEVBLOCK_CALLBACK(devblock_callback_t);




CONTAINER_TYPE(dev_blk_queue, CLIST,
struct dev_block_rq_s
{  
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




/** Block device class read() function tempate. */
#define DEVBLOCK_READ(n)	void (n) (struct device_s *dev,	\
					  struct dev_block_rq_s *rq)

/** Block device class read() methode shortcut */

#define dev_block_read(dev, ...) (dev)->drv->f.blk.f_read(dev, __VA_ARGS__ )
/**
   Block device class read() function type. Read count data blocks
   from the device.

   @param rq pointer to request data. lba, count, data and
          callback field must be intialized.
   @param dev pointer to device descriptor
   @param size max data read bytes count
*/
typedef DEVBLOCK_READ(devblock_read_t);




/** Block device class write() function tempate. */
#define DEVBLOCK_WRITE(n)	void (n) (struct device_s *dev, \
					  struct dev_block_rq_s *rq)


/** Block device class write() methode shortcut */

#define dev_block_write(dev, ...) (dev)->drv->f.blk.f_write(dev, __VA_ARGS__ )
/**
   Block device class write() function type. Write count data blocks
   from the device.

   @param dev pointer to device descriptor
   @param rq pointer to request data. lba, count, data and
          callback field must be intialized.
*/

typedef DEVBLOCK_WRITE(devblock_write_t);



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
   Block device class getparams() function type.

   @params dev pointer to device descriptor
   @return pointer to device parameters structure
*/
typedef DEVBLOCK_GETPARAMS(devblock_getparams_t);



/** Block device class methodes */
struct dev_class_block_s
{
  devblock_read_t		*f_read;
  devblock_write_t		*f_write;
  devblock_getparams_t		*f_getparams;
};


/** Synchronous helper read function. This function use the scheduler
    api to put current context in wait state. This function spin in a
    loop waiting for read operation to complete when scheduler is
    disabled.

    lba, count and data field of request must be initialized.
    data field is preserved.
*/
error_t dev_block_wait_read(struct device_s *dev, struct dev_block_rq_s *rq);

/** Synchronous helper read function. This function spin in a loop
    waiting for read operation to complete.

    lba, count and data field of request must be initialized.
    data field is preserved.
*/
error_t dev_block_lock_read(struct device_s *dev, struct dev_block_rq_s *rq);

/** Synchronous helper write function. This function use the scheduler
    api to put current context in wait state. This function spin in a
    wait loop waiting for read to complete when scheduler is disabled

    lba, count and data field of request must be initialized.
*/
error_t dev_block_wait_write(struct device_s *dev, struct dev_block_rq_s *rq);


#endif

