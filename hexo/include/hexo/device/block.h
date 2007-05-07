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

/** Block device read callback */
#define DEVBLOCK_READCALLBACK(n) void (n) (struct device_s *dev, void *pvdata, \
					   dev_block_lba_t count, uint8_t const * const * data);

/**
   Block device read callback function type. This function is called
   for each group of blocks read. It may be called several times,
   count must be used to detect read operation end.

   @param dev pointer to device descriptor
   @param pvdata pointer to private data
   @param count number of blocks read
   @param data table of pointers to each block data in memory. NULL indicate read error and terminate the read operation.
*/
typedef DEVBLOCK_READCALLBACK(devblock_readcallback_t);

/** Block device class read() function tempate. */
#define DEVBLOCK_READ(n)	error_t (n) (struct device_s *dev,			\
					  dev_block_lba_t lba, dev_block_lba_t count,		\
					  void *pvdata, devblock_readcallback_t *cback)

/** Block device class read() methode shortcut */

#define dev_block_read(dev, ...) (dev)->drv->f.blk.f_read(dev, __VA_ARGS__ )
/**
   Block device class read() function type. Read count data blocks
   from the device.

   @param dev pointer to device descriptor
   @param lba first block number to read
   @param count number of blocks to read
   @param pvdata pointer to private data passed to callback
   @param cback read callback
   @param size max data read bytes count
*/
typedef DEVBLOCK_READ(devblock_read_t);





/** Block device write callback */
#define DEVBLOCK_WRITECALLBACK(n) void (n) (struct device_s *dev, dev_block_lba_t count, void *pvdata);

/**
   Block device write callback function type. This function is called
   when a requested write operation completes. This function will be
   called only once per write operation.

   @param dev pointer to device descriptor
   @param count indicates wrote blocks count
   @param pvdata pointer to private data
*/
typedef DEVBLOCK_WRITECALLBACK(devblock_writecallback_t);

/** Block device class write() function tempate. */
#define DEVBLOCK_WRITE(n)	error_t (n) (struct device_s *dev,			\
					     dev_block_lba_t lba, dev_block_lba_t count,		\
					     uint8_t const * const * data,		\
					     void *pvdata, devblock_writecallback_t *cback)

/** Block device class write() methode shortcut */

#define dev_block_write(dev, ...) (dev)->drv->f.blk.f_write(dev, __VA_ARGS__ )
/**
   Block device class write() function type. Write count data blocks
   from the device.

   @param dev pointer to device descriptor
   @param lba first block number to write
   @param count number of blocks to write
   @param data table of pointers to each block data in memory
   @param pvdata pointer to private data passed to callback
   @param cback write callback, may be NULL
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

