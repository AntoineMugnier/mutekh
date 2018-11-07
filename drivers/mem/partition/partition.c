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
  License along with this program.  If not, see
  <http://www.gnu.org/licenses/>.

  Copyright (c) 2018 Nicolas Pouillon <nipo@ssji.net>
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/class/mem.h>

struct part_desc_s
{
  size_t page_offset;
  size_t page_count;
};

struct mem_part_private_s
{
  struct device_mem_s backend;
  size_t partition_count;
  size_t page_log2;
  struct part_desc_s partition[0];
};

DRIVER_PV(struct mem_part_private_s);

static DEV_MEM_INFO(mem_part_info)
{
  struct device_s *dev = accessor->dev;
  struct mem_part_private_s *pv = dev->drv_pv;

  if (accessor->number >= pv->partition_count)
    return -ENOENT;

  error_t err = DEVICE_OP(&pv->backend, info, info, band_index);
  if (err)
    return err;

  info->size = pv->partition[accessor->number].page_count << pv->page_log2;

  return 0;
}

static DEV_MEM_REQUEST(mem_part_request)
{
  struct device_s *dev = accessor->dev;
  struct mem_part_private_s *pv = dev->drv_pv;
  
  if (accessor->number >= pv->partition_count)
    goto error;

  const struct part_desc_s *partition = &pv->partition[accessor->number];
  uint64_t base = partition->page_offset << pv->page_log2;

  if (rq->type & _DEV_MEM_PARTIAL) {
    uint64_t size = partition->page_count << pv->page_log2;

    if (rq->partial.addr > size
        || rq->partial.addr + rq->partial.size > size)
      goto error;

    rq->partial.addr += base;

    DEVICE_OP(&pv->backend, request, rq);
    return;
  } else {
    for (uint8_t i = 0; i < rq->page.sc_count; ++i) {
      struct dev_mem_page_sc_s *sc = (struct dev_mem_page_sc_s *)&rq->page.sc[i];
      size_t first_page = sc->addr >> pv->page_log2;
      size_t page_count = 1 << (rq->page.page_log2 - pv->page_log2);
      size_t end_page = first_page + page_count;

      if (first_page >= partition->page_count)
        goto error;

      if (end_page > partition->page_count)
        goto error;

      sc->addr += base;
    }

    DEVICE_OP(&pv->backend, request, rq);
    return;
  }
        
 error:
  rq->error = -EINVAL;
  dev_mem_rq_done(rq);
}

static DEV_USE(mem_part_use)
{
  switch (op)
    {
    case DEV_USE_GET_ACCESSOR: {
      struct device_accessor_s *accessor = param;
      struct device_s *dev = accessor->dev;
      struct mem_part_private_s *pv = dev->drv_pv;
      if (accessor->number >= pv->partition_count)
        return -ENOTSUP;
      return 0;
    }

    case DEV_USE_LAST_NUMBER: {
      struct device_accessor_s *accessor = param;
      struct device_s *dev = accessor->dev;
      struct mem_part_private_s *pv = dev->drv_pv;
      accessor->number = pv->partition_count;
      return 0;
    }

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(mem_part_init)
{
  struct mem_part_private_s *pv;
  struct device_mem_s backend;
  struct dev_mem_info_s info;
  size_t partition_count = 0;
  error_t err;
  uintptr_t page_mask;
  
  err = device_get_param_dev_accessor(dev, "backend", &backend.base, DRIVER_CLASS_MEM);
  if (err)
    return err;

  err = DEVICE_OP(&backend, info, &info, 0);
  if (err)
    goto out_device;
  
  page_mask = ((uintptr_t)1 << info.page_log2) - 1;

  err = -EINVAL;
  
  DEVICE_RES_FOREACH(dev, res, {
      if (res->type != DEV_RES_MEM)
        continue;

      partition_count++;

      if (res->u.mem.start & page_mask)
        goto out_device;

      if (res->u.mem.end & page_mask)
        goto out_device;

      if ((res->u.mem.start >> info.page_log2) >= info.size)
        goto out_device;

      if (res->u.mem.end && (res->u.mem.end >> info.page_log2) > info.size)
        goto out_device;
    });
  
  /* allocate private driver data */
  pv = mem_alloc(sizeof(*pv) + sizeof(pv->partition[0]) * partition_count,
                 mem_scope_sys);
  if (!pv) {
    err = -ENOMEM;
    goto out_device;
  }

  pv->page_log2 = info.page_log2;
  pv->partition_count = partition_count;
  err = device_copy_accessor(&pv->backend.base, &backend.base);
  if (err)
    goto out_pv;

  partition_count = 0;
  DEVICE_RES_FOREACH(dev, res, {
      if (res->type != DEV_RES_MEM)
        continue;

      pv->partition[partition_count].page_offset
        = res->u.mem.start >> pv->page_log2;
      if (res->u.mem.end)
        pv->partition[partition_count].page_count
          = (res->u.mem.end - res->u.mem.start) >> pv->page_log2;
      else
        pv->partition[partition_count].page_count
          = info.size - pv->partition[partition_count].page_offset;

      partition_count++;
    });

  dev->drv_pv = pv;

  err = 0;
  goto out_device;

 out_pv:
  mem_free(pv);
 out_device:
  device_put_accessor(&backend.base);
  return err;
}

static DEV_CLEANUP(mem_part_cleanup)
{
  struct mem_part_private_s *pv = dev->drv_pv;

  device_put_accessor(&pv->backend.base);
  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(mem_part_drv, 0, "Memory Partition", mem_part,
               DRIVER_MEM_METHODS(mem_part));

DRIVER_REGISTER(mem_part_drv);

