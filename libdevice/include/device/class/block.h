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
 * @short Block device driver API
 */                                                                 

#ifndef __DEVICE_BLOCK_H__
#define __DEVICE_BLOCK_H__

#include <hexo/types.h>
#include <hexo/error.h>
#include <gct_platform.h>
#include <gct/container_slist.h>

#include <device/driver.h>

struct device_s;
struct driver_s;

typedef uint32_t dev_block_lba_t;
struct dev_block_rq_s;

/** Block device read/write callback */
#define DEVBLOCK_CALLBACK(n) void (n) (const struct dev_block_rq_s *rq, size_t count)

/**
   Block device read callback function. This function is called for
   each group of blocks read. It may be called several times,
   @tt {rq->progress > 0 && rq->progress < rq->count}
   must be used to detect operation end. The device
   lock may be held when call back function is called.

   @param rq pointer to request data.
          @tt rq->proress field is updated to processed blocks count or has negative
	  error code.
   @param count number of processed blocks

   @see #DEVBLOCK_CALLBACK
*/
typedef DEVBLOCK_CALLBACK(devblock_callback_t);

enum dev_block_rq_type_e
  {
    DEV_BLOCK_READ = 1,
    DEV_BLOCK_WRITE = 2,

    DEV_BLOCK_OPMASK = 3,

    DEV_BLOCK_NOCACHE = 4,
  };

#define GCT_CONTAINER_ALGO_dev_blk_queue CLIST

/** Block device request object. */
struct dev_block_rq_s
{  
  GCT_CONTAINER_ENTRY(dev_blk_queue, queue_entry);

  enum dev_block_rq_type_e	type;    //< request type and flags
  dev_block_lba_t		lba;     //< logical block address
  size_t			count;	 //< block count
  uint8_t			**data;  //< table of pointer to data blocks
  ssize_t			progress; //< number of processed blocks or negative error code

  const struct device_block_s   *bdev;     //< associated block device
  devblock_callback_t		*callback; //< callback function
  void				*pvdata;   //< pv data for callback

  void				*drvdata;       //< driver private data
};

GCT_CONTAINER_TYPES(dev_blk_queue, struct dev_block_rq_s *, queue_entry);
GCT_CONTAINER_FCNS(dev_blk_queue, static inline, dev_blk_queue,
                   init, destroy, pushback, pop, isempty, head);

/** Block device class request() function tempate. */
#define DEVBLOCK_REQUEST(n)	void (n) (struct device_block_s *bdev,	\
					  struct dev_block_rq_s *rq)

/**
   Block device request function type. Request count data blocks
   from the block device.

   @param bdev pointer to block device descriptor
   @param rq pointer to request. @tt lba , @tt count , @tt data ,
          @tt progress and @tt callback field must be intialized.
	  @tt progress field may not be zero.
   @see #DEVBLOCK_REQUEST
*/
typedef DEVBLOCK_REQUEST(devblock_request_t);




struct dev_block_params_s
{
  size_t		blk_size;	/* size of a single block */
  dev_block_lba_t	blk_count;	/* device blocks count */
};

/** Block device class getparams() function tempate. */
#define DEVBLOCK_GETPARAMS(n)	const struct dev_block_params_s * (n) (struct device_block_s *bdev)


/**
   Block device getparams function type.

   @param ndev pointer to block device descriptor
   @return pointer to block device parameters structure
   @see #DEVBLOCK_GETPARAMS
*/
typedef DEVBLOCK_GETPARAMS(devblock_getparams_t);


DRIVER_CLASS_TYPES(block, 
                   devblock_request_t		*f_request;
                   devblock_getparams_t		*f_getparams;
                   );

/** Synchronous helper read function. This function use the scheduler
    api to put current context in wait state if no data is available
    from device yet. This function is equivalent to
    dev_block_spin_read() when scheduler is disabled.
*/
config_depend(CONFIG_DEVICE_BLOCK)
error_t dev_block_wait_read(struct device_block_s *bdev, uint8_t **data, dev_block_lba_t lba, size_t count);

/** Synchronous helper read function. This function spin in a loop
    waiting for read operation to complete.
*/
config_depend(CONFIG_DEVICE_BLOCK)
error_t dev_block_spin_read(struct device_block_s *bdev, uint8_t **data, dev_block_lba_t lba, size_t count);

/** Synchronous helper write function. This function use the scheduler
    api to put current context in wait state if no data is available
    from device yet. This function is equivalent to
    dev_block_spin_write() when scheduler is disabled.
*/
config_depend(CONFIG_DEVICE_BLOCK)
error_t dev_block_wait_write(struct device_block_s *bdev, uint8_t **data, dev_block_lba_t lba, size_t count);

/** Synchronous helper write function. This function spin in a loop
    waiting for write operation to complete.
*/
config_depend(CONFIG_DEVICE_BLOCK)
error_t dev_block_spin_write(struct device_block_s *bdev, uint8_t **data, dev_block_lba_t lba, size_t count);


#endif

