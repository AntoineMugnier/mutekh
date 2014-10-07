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

    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014

*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/class/mem.h>

struct ram_bank_s
{
  uintptr_t addr;
  uintptr_t end;
};

struct ram_context_s
{
  size_t bank_count;
  struct ram_bank_s bank[0];
};

static DEVMEM_INFO(ram_info)
{
  struct device_s *dev = mdev->dev;
  struct ram_context_s *pv = dev->drv_pv;

  if (mdev->number >= pv->bank_count)
    return -ENOENT;

  if (band_index > 0)
    return -ENOENT;

  const struct ram_bank_s *b = pv->bank + mdev->number;

  memset(info, 0, sizeof(*info));
  
  info->type = DEV_MEM_RAM;
  info->flags |= DEV_MEM_WRITABLE | DEV_MEM_VOLATILE |
    DEV_MEM_MAPPED_READ | DEV_MEM_MAPPED_WRITE |
    DEV_MEM_PARTIAL_WRITE | DEV_MEM_PARTIAL_READ |
    DEV_MEM_CROSS_READ | DEV_MEM_CROSS_WRITE;
  info->map_base = b->addr;
  info->size =  b->end - b->addr;

  return 0;
}

static DEVMEM_REQUEST(ram_request)
{
  struct device_s *dev = mdev->dev;
  struct ram_context_s *pv = dev->drv_pv;

  rq->err = 0;

  if (mdev->number >= pv->bank_count)
    rq->err = -ENOENT;
  else if (rq->band_mask & 0xfe)
    rq->err = -ENOENT;
  else if (rq->band_mask & 1)
    {
      const struct ram_bank_s *b = pv->bank + mdev->number;
      dev_mem_mapped_op_helper(b->addr, 1, rq);
    }

  kroutine_exec(&rq->kr, cpu_is_interruptible());
}

static const struct driver_mem_s	ram_mem_drv =
{
  .class_		= DRIVER_CLASS_MEM,
  .f_info               = ram_info,
  .f_request		= ram_request,
};

static DEV_INIT(ram_init);
static DEV_CLEANUP(ram_cleanup);

static const struct devenum_ident_s	ram_ids[] =
{
  DEVENUM_FDTNAME_ENTRY("generic:ram"),
  { 0 }
};

const struct driver_s	ram_drv =
{
  .desc                 = "Generic RAM driver",
  .id_table		= ram_ids,
  .f_init		= ram_init,
  .f_cleanup		= ram_cleanup,
  .classes              = { &ram_mem_drv, 0 }
};

REGISTER_DRIVER(ram_drv);

static DEV_INIT(ram_init)
{
  struct ram_context_s	*pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  size_t i, count = 0;

  while (1)
    {
      if (device_res_get_uint(dev, DEV_RES_MEM, count, NULL, NULL))
        break;
      count++;
    }

  if (count == 0)
    return -1;

  pv = mem_alloc(sizeof(struct ram_context_s) +
                 sizeof(struct ram_bank_s) * count, (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  pv->bank_count = count;
  for (i = 0; i < count; i++)
    device_res_get_uint(dev, DEV_RES_MEM, i, &pv->bank[i].addr, &pv->bank[i].end);

  dev->drv = &ram_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
}

static DEV_CLEANUP(ram_cleanup)
{
  struct ram_context_s	*pv = dev->drv_pv;

  mem_free(pv);
}

