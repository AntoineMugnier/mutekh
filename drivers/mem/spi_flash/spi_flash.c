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

    Copyright (c) 2015 Alexandre Becoulet <alexandre.becoulet@free.fr>

*/

#include "spi_flash.h"
#include "spi_flash_spi.o.h"

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

DEV_MEM_INFO(spi_flash_info)
{
  struct device_s *dev = accessor->dev;
  struct spi_flash_private_s *pv = dev->drv_pv;

  if (band_index > 0)
    return -ENOENT;

  memcpy(info, &pv->info->mem_info, sizeof(*info));
  info->size = pv->size >> info->page_log2;

  return 0;
}

static void spi_flash_pv_cleanup(struct device_s *dev)
{
  struct spi_flash_private_s *pv = dev->drv_pv;
  dev_drv_spi_bytecode_cleanup(&pv->spi, &pv->srq);

  dev_request_queue_destroy(&pv->queue);

  mem_free(pv);
}

static struct dev_mem_rq_s *
spi_flash_push(struct device_s *dev, struct dev_mem_rq_s *rq)
{
  struct spi_flash_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  dev_request_queue_pushback(&pv->queue, dev_mem_rq_s_base(rq));

  if (pv->state == SPI_FLASH_STATE_IDLE)
    pv->state = SPI_FLASH_STATE_BUSY;
  else
    rq = NULL;

  LOCK_RELEASE_IRQ(&dev->lock);

  return rq;
}

static struct dev_mem_rq_s *
spi_flash_next(struct device_s *dev, error_t err)
{
  struct spi_flash_private_s *pv = dev->drv_pv;
  struct dev_mem_rq_s *rq;

  if (err == -EAGAIN)
    err = 0;

  LOCK_SPIN_IRQ(&dev->lock);
  rq = dev_mem_rq_s_cast(dev_request_queue_pop(&pv->queue));
  rq->err = err;
  kroutine_exec(&rq->base.kr);

  rq = dev_mem_rq_s_cast(dev_request_queue_head(&pv->queue));
  if (rq == NULL)
    {
      pv->state = SPI_FLASH_STATE_IDLE;
      device_sleep_schedule(dev);
    }
  LOCK_RELEASE_IRQ(&dev->lock);

  return rq;
}

static struct dev_mem_rq_s *
spi_flash_cur(struct device_s *dev)
{
  struct spi_flash_private_s *pv = dev->drv_pv;
  struct dev_mem_rq_s *rq;

  LOCK_SPIN_IRQ(&dev->lock);
  rq = dev_mem_rq_s_cast(dev_request_queue_head(&pv->queue));
  if (rq)
    pv->state = SPI_FLASH_STATE_BUSY;
  else
    pv->state = SPI_FLASH_STATE_IDLE;
  LOCK_RELEASE_IRQ(&dev->lock);

  return rq;
}

static void spi_flash_sleep(struct device_s *dev)
{
  struct spi_flash_private_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->srq;

  if (pv->state == SPI_FLASH_STATE_IDLE &&
      !dev_spi_bytecode_start(&pv->spi, srq, &spi_flash_bc_sleep, 0))
    pv->state = SPI_FLASH_STATE_SLEEP;
}

static error_t spi_flash_start(struct device_s *dev, struct dev_mem_rq_s *rq)
{
  struct spi_flash_private_s *pv = dev->drv_pv;
  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->srq;

  void *bc_entry;

  enum dev_mem_rq_type_e type = rq->type & ~(_DEV_MEM_FLUSH | _DEV_MEM_INVAL);

  if (pv->info->ops_exclude & type)
    return -ENOTSUP;

  if (type & _DEV_MEM_PAGE)
    {
      uint_fast8_t pl2 = 8;

      switch (type & _DEV_MEM_OP_MASK)
        {
        case _DEV_MEM_READ:
          bc_entry = &spi_flash_bc_read_page;
          break;
        case _DEV_MEM_WRITE:
          bc_entry = &spi_flash_bc_write_page;
          break;
        case _DEV_MEM_ERASE:
          pl2 = 12;
          bc_entry = &spi_flash_bc_erase_page;
          break;
        default:
          UNREACHABLE();
        }

      if (rq->page.page_log2 < pl2)
	return -ERANGE;

      if (rq->page.sc_count == 0)
        return -EAGAIN;

      bc_set_reg(&srq->vm, SPI_FLASH_BC_PAGE_OP_BCIN_PAGE_COUNT, 1 << (rq->page.page_log2 - pl2));
      bc_set_reg(&srq->vm, SPI_FLASH_BC_PAGE_OP_BCIN_SC, (bc_reg_t)rq->page.sc);
      bc_set_reg(&srq->vm, SPI_FLASH_BC_PAGE_OP_BCIN_SC_COUNT, rq->page.sc_count);
    }
  else if (type & _DEV_MEM_PARTIAL)
    {
      switch (type & _DEV_MEM_OP_MASK)
        {
        case _DEV_MEM_READ:
          bc_entry = &spi_flash_bc_read_partial;
          break;
        case _DEV_MEM_WRITE:
          if ((rq->partial.addr & 255) + rq->partial.size > 256)
            return -ERANGE;   // cross page boundary
          bc_entry = &spi_flash_bc_write_partial;
          break;
        default:
          UNREACHABLE();
        }

      if (rq->partial.size == 0)
        return -EAGAIN;

      bc_set_reg(&srq->vm, SPI_FLASH_BC_PARTIAL_OP_BCIN_ADDR, rq->partial.addr);
      bc_set_reg(&srq->vm, SPI_FLASH_BC_PARTIAL_OP_BCIN_DATA, (uintptr_t)rq->partial.data);
      bc_set_reg(&srq->vm, SPI_FLASH_BC_PARTIAL_OP_BCIN_SIZE, rq->partial.size);
    }
  else if (type == DEV_MEM_OP_ERASE)
    {
      bc_entry = &spi_flash_bc_chip_erase;
    }
  else
    {
      return -EAGAIN;
    }

  dev_spi_bytecode_start(&pv->spi, srq, bc_entry, 0);
  return 0;		/* started */
}

static KROUTINE_EXEC(spi_flash_srq_done)
{
  struct dev_spi_ctrl_bytecode_rq_s *srq = KROUTINE_CONTAINER(kr, *srq, base.base.kr);
  struct device_s *dev = srq->base.base.pvdata;
  struct spi_flash_private_s *pv = dev->drv_pv;
  error_t err = 0;

  switch (pv->state)
    {
    case SPI_FLASH_STATE_INIT: {
      pv->state = SPI_FLASH_STATE_IDLE;

      if (srq->base.err)
        {
          err = -EIO;
          spi_flash_pv_cleanup(dev);
        }

      device_async_init_done(dev, err);
      break;
    }

    case SPI_FLASH_STATE_BUSY: {
      struct dev_mem_rq_s *rq;
      err = srq->base.err ? -EIO : 0;

      while ((rq = spi_flash_next(dev, err)))
	{
	  err = spi_flash_start(dev, rq);
	  if (!err)
	    break;
	}

      break;
    }

    case SPI_FLASH_STATE_SLEEP: {
      struct dev_mem_rq_s *rq = spi_flash_cur(dev);

      while (rq)
	{
	  err = spi_flash_start(dev, rq);
	  if (!err)
	    break;
	  rq = spi_flash_next(dev, err);
	}

      break;
    }

    case SPI_FLASH_STATE_IDLE:
      UNREACHABLE();
    }
}

DEV_MEM_REQUEST(spi_flash_request)
{
  struct device_s *dev = accessor->dev;

  rq = spi_flash_push(dev, rq);

  if (rq)
    {
      error_t err = spi_flash_start(dev, rq);
      if (err)
	spi_flash_next(dev, err);
    }
}

DEV_USE(spi_flash_use)
{
  struct device_s *dev;

  switch (op)
    {
    case DEV_USE_SLEEP:
      dev = param;
      break;

    case DEV_USE_STOP: {
      struct device_accessor_s *acc = param;
      dev = acc->dev;
      break;
    }

    default:
      return dev_use_generic(param, op);
    }

  if (dev->start_count == 0)
    spi_flash_sleep(dev);

  return 0;
}

error_t spi_flash_init_common(struct device_s *dev,
			      const struct spi_flash_info_s *info)
{
  struct spi_flash_private_s *pv;

  pv = mem_alloc(sizeof (*pv), (mem_scope_sys));

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  dev->drv_pv = pv;

  pv->info = info;
  device_get_param_uint_default(dev, "size", &pv->size, info->size);

  struct dev_spi_ctrl_bytecode_rq_s *srq = &pv->srq;

  /* SPI stuff */
  struct device_timer_s *timer;
  if (dev_drv_spi_bytecode_init(dev, srq, &spi_flash_bytecode,
                                &info->spi_cfg, &pv->spi, NULL, &timer))
    goto err_mem;

  srq->base.base.pvdata = dev;

  kroutine_init_deferred(&srq->base.base.kr, &spi_flash_srq_done);

  /* compute erase and write delays */
  dev_timer_init_sec(timer, &pv->byte_erase_delay, 0, info->byte_erase_delay, 1000000);

  dev_timer_init_sec(timer, &pv->byte_write_delay, 0, info->byte_write_delay, 1000000);

  dev_timer_delay_t delay = info->chip_erase_delay * pv->size;
  if (!delay)
    delay = 1000000; /* 1 sec default when size not known */
  dev_timer_init_sec(timer, &pv->chip_erase_delay, 0, delay, 1000000);

  bc_set_reg(&srq->vm, SPI_FLASH_BCCONST_CTX, (uintptr_t)pv);

  /* set address handler */
  bc_set_reg(&srq->vm, SPI_FLASH_BCCONST_SEND_HEADER,
	     (uintptr_t)info->send_header);

  dev_request_queue_init(&pv->queue);

  /* starts device init sequence */
  if (dev_spi_bytecode_start(&pv->spi, srq, &spi_flash_bc_detect, 0))
    goto err_srq;

  return -EAGAIN;

 err_srq:
  dev_drv_spi_bytecode_cleanup(&pv->spi, srq);
 err_mem:
  mem_free(pv);
  return -1;
}

DEV_CLEANUP(spi_flash_cleanup)
{
  struct spi_flash_private_s *pv = dev->drv_pv;

  if (!dev_request_queue_isempty(&pv->queue))
    return -EBUSY;

  spi_flash_pv_cleanup(dev);
  return 0;
}

