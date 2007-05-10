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

struct device_s;
struct driver_s;

typedef uint32_t dev_block_lba_t;

CONTAINER_TYPE(dev_blk_queue, CLIST,
struct dev_block_rq_s
{  
  dev_block_lba_t		lba;	/* lba */
  size_t			count;	/* block count */
  void				*pvdata; /* pv data for callback */
  devblock_readcallback_t	*cback; /* callback function */
  uint8_t			**data; /* table of pointer to data blocks */
  error_t			error; /* error code */

  dev_blk_queue_entry_t		queue_entry; /* used by driver to enqueue resquests */
}, queue_entry);

CONTAINER_FUNC(dev_blk_queue, CLIST, static inline, dev_blk_queue);

/** Block device read callback */
#define DEVBLOCK_READCALLBACK(n) void (n) (struct device_s *dev, \
					   const struct dev_block_rq_s *rq, \
					   size_t count)

/**
   Block device read callback function type. This function is called
   for each group of blocks read. It may be called several times,
   count must be used to detect read operation end.

   @param dev pointer to device descriptor
   @param rq pointer to request data. rq->count field is
          updated with remaining blocks to read. rq->lba is
	  advanced to next unread block address. rq->data is
	  updated to point to internal driver data buffers.
	  rq->error is updated.
   @param count number of blocks buffer read
   @param data table of pointers to each read block data
*/
typedef DEVBLOCK_READCALLBACK(devblock_readcallback_t);

/** Block device class read() function tempate. */
#define DEVBLOCK_READ(n)	void (n) (struct device_s *dev,	\
					  struct dev_block_rq_s *rq)

/** Block device class read() methode shortcut */

#define dev_block_read(dev, ...) (dev)->drv->f.blk.f_read(dev, __VA_ARGS__ )
/**
   Block device class read() function type. Read count data blocks
   from the device.

   @param rq pointer to request data. lba, count and
          callback field must be intialized. the data
	  field will be provided by driver on callback.
   @param dev pointer to device descriptor
   @param size max data read bytes count
*/
typedef DEVBLOCK_READ(devblock_read_t);





/** Block device write callback */
#define DEVBLOCK_WRITECALLBACK(n) void (n) (struct device_s *dev, \
					    const struct dev_block_rq_s *rq)

/**
   Block device write callback function type. This function is called
   when a requested write operation completes. This function will be
   called only once per write operation.

   @param dev pointer to device descriptor
   @param rq pointer to request data. rq->count field is
          updated with remaining blocks to write. rq->lba is
	  advanced to next block address.
	  rq->error is updated.
   @param err error code
*/
typedef DEVBLOCK_WRITECALLBACK(devblock_writecallback_t);

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
  size_t		blk_size;	/* size of a single block, must be power of 2 */
  uint_fast8_t		blk_sh_size;  /* log2 size of a single block */
  dev_block_lba_t	blk_count;    /* device blocks count */
};

/** Block device class getparams() function tempate. */
#define DEVBLOCK_GETPARAMS(n)	const struct dev_block_params_s * (n) (struct device_s *dev)

/** Block device class getparams() methode shortcut */

#define dev_block_getparams(dev)->drv->f.blk.f_getparams(dev)

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

#endif

