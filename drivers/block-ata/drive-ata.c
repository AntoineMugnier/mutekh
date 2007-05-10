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
#include <timer.h>

#include "block-ata.h"

#include "block-ata-private.h"

/**************************************************************/

/* 
 * device read operation
 */

DEVBLOCK_READ(drive_ata_read)
{

}

/* 
 * device write operation
 */

DEVBLOCK_WRITE(drive_ata_write)
{

}

/* 
 * device write operation
 */

DEVBLOCK_GETPARAMS(drive_ata_getparams)
{
  struct drive_ata_context_s	*pv = dev->drv_pv;

  return &pv->drv_params;
}

/* 
 * device close operation
 */

DEV_CLEANUP(drive_ata_cleanup)
{
  struct drive_ata_context_s	*pv = dev->drv_pv;

  mem_free(pv);
}

/* 
 * device open operation
 */

#ifndef CONFIG_STATIC_DRIVERS

const struct driver_s	drive_ata_drv =
{
  .f_cleanup		= drive_ata_cleanup,
  .f.blk = {
    .f_read		= drive_ata_read,
    .f_write		= drive_ata_write,
    .f_getparams	= drive_ata_getparams,
  }
};
#endif

error_t drive_ata_init(struct device_s *dev, bool_t slave)
{
  struct drive_ata_context_s	*pv;

  assert(sizeof(struct ata_indent_s) == 512);

  /* allocate private driver data */
  pv = mem_alloc(sizeof(*pv), MEM_SCOPE_SYS);

  if (!pv)
    return -1;

  dev->drv_pv = pv;
  pv->slave = slave;
  pv->devhead_reg = ATA_DRVHEAD_RESERVED_HIGH | (slave ? ATA_DRVHEAD_SLAVE : 0);

  controller_ata_reg_w8(dev->parent, ATA_REG_DRVHEAD, pv->devhead_reg);

  /* read identification data */
  controller_ata_reg_w8(dev->parent, ATA_REG_COMMAND, ATA_CMD_IDENTIFY_DRIVE);

  if (controller_ata_waitbusy(dev->parent))
    return 1;

  controller_ata_data_read16(dev->parent, &pv->ident);

  /* does not support CHS mode yet */
  assert(!pv->ident.lba_supported);

  pv->drv_params.blk_size = 512;
  pv->drv_params.blk_sh_size = 9;
  pv->drv_params.blk_count =
    endian_be16(pv->ident.lba_count_low)
    | (endian_be16(pv->ident.lba_count_high) << 16);

  printf("ATA drive : %u sectors : %.40s\n",
	 pv->drv_params.blk_count, pv->ident.model_name);

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &drive_ata_drv;
#endif

  return 0;
}

