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
#include <hexo/error.h>

#include <device/block.h>
#include <hexo/device.h>
#include <device/driver.h>

#include <hexo/iospace.h>
#include <hexo/alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>
#include <timer.h>

#include "block-ata.h"

#include "block-ata-private.h"

/**************************************************************/

/* add a new request in queue and start if idle */
static void drive_ata_rq_start(struct device_s *dev,
			       struct dev_block_rq_s *rq)
{
  struct drive_ata_context_s *dpv = dev->drv_pv;
  bool_t idle = dev_blk_queue_isempty(&dpv->queue);

  dev_blk_queue_pushback(&dpv->queue, rq);

  if (idle)
    {
      struct drive_ata_oper_s *op = rq->drvdata;
      op->start(dev, rq);
    }
}

/* drop current request and start next in queue if any */
static void drive_ata_rq_end(struct device_s *dev)
{
  struct drive_ata_context_s *dpv = dev->drv_pv;
  struct dev_block_rq_s *rq;
  struct drive_ata_oper_s *op;

  dev_blk_queue_pop(&dpv->queue);

  if ((rq = dev_blk_queue_head(&dpv->queue)) != NULL)
    {
      op = rq->drvdata;
      op->start(dev, rq);
    }
  else
    {
      /* drive goes idle, select other drive to give a chance to interrupt */
      controller_ata_reg_w8(dev->parent, ATA_REG_DRVHEAD,
			    (dpv->devhead_reg ^ ATA_DRVHEAD_SLAVE) & ATA_DRVHEAD_SLAVE);
    }
}

/* try irq */
bool_t drive_ata_try_irq(struct device_s *dev)
{
  struct drive_ata_context_s *dpv = dev->drv_pv;
  struct dev_block_rq_s *rq;

  if ((rq = dev_blk_queue_head(&dpv->queue)) != NULL)
    {
      struct drive_ata_oper_s *op = rq->drvdata;

      return op->irq(dev, rq);
    }

  return 0;
}

/* 
 * device read operation
 */

static DRIVE_ATA_START_FUNC(drive_ata_read_start)
{
  struct device_s *parent = dev->parent;
  struct drive_ata_context_s *dpv = dev->drv_pv;
  uint32_t lba = rq->lba & 0x0fffffff;

  dpv->ata_sec_count = __MIN(256, rq->count);

  controller_ata_reg_w8(parent, ATA_REG_DRVHEAD, dpv->devhead_reg | (lba >> 24));

  controller_ata_reg_w8(parent, ATA_REG_CYLINDER_HIGH, lba >> 16);
  controller_ata_reg_w8(parent, ATA_REG_CYLINDER_LOW, lba >> 8);
  controller_ata_reg_w8(parent, ATA_REG_SECTOR_NUMBER, lba);
  controller_ata_reg_w8(parent, ATA_REG_SECTOR_COUNT, dpv->ata_sec_count);

  controller_ata_reg_w8(parent, ATA_REG_COMMAND, ATA_CMD_READ_SECTORS);

}

static DRIVE_ATA_IRQ_FUNC(drive_ata_read_irq)
{
  struct controller_ata_context_s*cpv = dev->parent->drv_pv;
  struct drive_ata_context_s	*dpv = dev->drv_pv;
  uint8_t status;

  controller_ata_reg_w8(dev->parent, ATA_REG_DRVHEAD, dpv->devhead_reg);

  status = controller_ata_reg_r8(dev->parent, ATA_REG_STATUS);

  if (status & ATA_STATUS_ERROR)
    {
      rq->error = EIO;
      rq->callback(dev, rq, 0);

      drive_ata_rq_end(dev);
      return 1;
    }

  if (status & ATA_STATUS_DATA_RQ)
    {
      controller_ata_data_read16(dev->parent, *rq->data);

      rq->lba++;
      rq->count--;
      rq->data++;
      rq->error = 0;
      dpv->ata_sec_count--;

      rq->callback(dev, rq, 1);

      if (rq->count == 0)
	{
	  drive_ata_rq_end(dev);
	}
      else
	{
	  if (dpv->ata_sec_count == 0)
	    drive_ata_read_start(dev, rq);
	}

      return 1;
    }

  return 0;
}

/* 
 * device write operation
 */

static DRIVE_ATA_START_FUNC(drive_ata_write_start)
{
  struct device_s *parent = dev->parent;
  struct drive_ata_context_s *dpv = dev->drv_pv;
  uint32_t lba = rq->lba & 0x0fffffff;

  dpv->ata_sec_count = __MIN(256, rq->count);

  controller_ata_reg_w8(parent, ATA_REG_DRVHEAD, dpv->devhead_reg | (lba >> 24));
  controller_ata_reg_w8(parent, ATA_REG_CYLINDER_HIGH, lba >> 16);
  controller_ata_reg_w8(parent, ATA_REG_CYLINDER_LOW, lba >> 8);
  controller_ata_reg_w8(parent, ATA_REG_SECTOR_NUMBER, lba);
  controller_ata_reg_w8(parent, ATA_REG_SECTOR_COUNT, dpv->ata_sec_count);

  controller_ata_reg_w8(parent, ATA_REG_COMMAND, ATA_CMD_WRITE_SECTORS);

  while (controller_ata_reg_r8(parent, ATA_REG_STATUS) & ATA_STATUS_BUSY)
    ;

  controller_ata_data_write16(parent, *rq->data);
}

static DRIVE_ATA_IRQ_FUNC(drive_ata_write_irq)
{
  struct controller_ata_context_s*cpv = dev->parent->drv_pv;
  struct drive_ata_context_s	*dpv = dev->drv_pv;
  uint8_t status;

  controller_ata_reg_w8(dev->parent, ATA_REG_DRVHEAD, dpv->devhead_reg);

  status = controller_ata_reg_r8(dev->parent, ATA_REG_STATUS);

  if (!(status & ATA_STATUS_BUSY))
    {
      if (status & ATA_STATUS_ERROR)
	{
	  rq->error = EIO;
	  rq->callback(dev, rq, 0);

	  drive_ata_rq_end(dev);
	}
      else
	{
	  rq->lba++;
	  rq->count--;
	  rq->data++;
	  rq->error = 0;
	  dpv->ata_sec_count--;

	  rq->callback(dev, rq, 1);

	  if (rq->count == 0)
	    {
	      drive_ata_rq_end(dev);
	    }
	  else
	    {
	      if (dpv->ata_sec_count > 0)
		controller_ata_data_write16(dev->parent, *rq->data);
	      else
		drive_ata_write_start(dev, rq);
	    }
	}

      return 1;
    }

  return 0;
}

static const struct drive_ata_oper_s drive_ata_write_oper =
  {
    .irq = drive_ata_write_irq,
    .start = drive_ata_write_start,
  };

static const struct drive_ata_oper_s drive_ata_read_oper =
  {
    .irq = drive_ata_read_irq,
    .start = drive_ata_read_start,
  };

DEVBLOCK_REQUEST(drive_ata_request)
{
  struct controller_ata_context_s *cpv = dev->parent->drv_pv;
  struct drive_ata_context_s *dpv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->parent->lock);

  if (rq->lba + rq->count > dpv->drv_params.blk_count)
    {
      rq->error = ERANGE;
      rq->callback(dev, rq, 0);
    }
  else
    {
      switch (rq->type)
	{
	case DEV_BLOCK_READ:
	  rq->drvdata = (void*)&drive_ata_read_oper;
	  break;
	case DEV_BLOCK_WRITE:
	  rq->drvdata = (void*)&drive_ata_write_oper;
	  break;
	}

      drive_ata_rq_start(dev, rq);
    }

  LOCK_RELEASE_IRQ(&dev->parent->lock);
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
  .class		= device_class_block,
  .f_cleanup		= drive_ata_cleanup,
  .f.blk = {
    .f_request		= drive_ata_request,
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
  pv->devhead_reg = ATA_DRVHEAD_RESERVED_HIGH | (slave ? ATA_DRVHEAD_SLAVE : 0);

  controller_ata_reg_w8(dev->parent, ATA_REG_DRVHEAD, pv->devhead_reg);

  /* read identification data */
  controller_ata_reg_w8(dev->parent, ATA_REG_COMMAND, ATA_CMD_IDENTIFY_DRIVE);

  if (controller_ata_waitbusy(dev->parent))
    return 1;

  controller_ata_data_swapread16(dev->parent, &pv->ident);

  /* does not support CHS mode yet */
  if (pv->ident.lba_supported)
    return 1;

  pv->devhead_reg |= ATA_DRVHEAD_LBA;

  pv->drv_params.blk_size = 512;
  pv->drv_params.blk_count =
    endian_be16(pv->ident.lba_count_low)
    | (endian_be16(pv->ident.lba_count_high) << 16);

  printf("ATA drive : %u sectors : %.40s\n",
	 pv->drv_params.blk_count, pv->ident.model_name);

  dev_blk_queue_init(&pv->queue);

  /* enable interrupts for this drive */
  controller_ata_rega_w8(dev->parent, ATA_REG_DEVICE_CONTROL,
			 ATA_DEVCTRL_RESERVED_HIGH);

#ifndef CONFIG_STATIC_DRIVERS
  dev->drv = &drive_ata_drv;
#endif

  return 0;
}

