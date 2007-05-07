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


#include <hexo/types.h>

#include <hexo/device/block.h>
#include <hexo/device.h>
#include <hexo/driver.h>

#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include "block-ata.h"

#include "block-ata-private.h"

/**************************************************************/

/* 
 * device read operation
 */

DEVBLOCK_READ(block_ata_read)
{

}

/* 
 * device write operation
 */

DEVBLOCK_WRITE(block_ata_write)
{

}

/* 
 * device write operation
 */

DEVBLOCK_GETPARAMS(block_ata_getparams)
{
  return 0;
}

/* 
 * device close operation
 */

DEV_CLEANUP(block_ata_cleanup)
{
  struct block_ata_context_s	*pv = dev->drv_pv;

  lock_destroy(&pv->lock);

  mem_free(pv);
}

/*
 * IRQ handler
 */

DEV_IRQ(block_ata_irq)
{
  return 1;
}

/* 
 * device open operation
 */

#ifndef CONFIG_STATIC_DRIVERS
const struct driver_s	block_ata_drv =
{
  .f_init		= block_ata_init,
  .f_cleanup		= block_ata_cleanup,
  .f_irq		= block_ata_irq,
  .f.blk = {
    .f_read		= block_ata_read,
    .f_write		= block_ata_write,
    .f_getparams	= block_ata_getparams,
  }
};
#endif

DEV_INIT(block_ata_init)
{
  struct block_ata_context_s	*pv;

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &block_ata_drv;
#endif

  /* alocate private driver data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  lock_init(&pv->lock);

  dev->drv_pv = pv;

  return 0;
}

