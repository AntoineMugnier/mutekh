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

  Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2014
  Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/class/mem.h>

#include <arch/nrf5x/nvmc.h>
#include <arch/nrf5x/ficr.h>
#include <arch/nrf5x/uicr.h>

static DEV_MEM_INFO(nrf5x_ram_info)
{
  size_t count = cpu_mem_read_32(NRF_FICR_NUMRAMBLOCK);
  uintptr_t base = 0x20000000;

  if (accessor->number >= count)
    return -ENOENT;

  if (band_index > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));

  info->type = DEV_MEM_RAM;
  info->flags |= DEV_MEM_WRITABLE | DEV_MEM_VOLATILE |
    DEV_MEM_MAPPED_READ | DEV_MEM_MAPPED_WRITE |
    DEV_MEM_PARTIAL_WRITE | DEV_MEM_PARTIAL_READ |
    DEV_MEM_CROSS_READ | DEV_MEM_CROSS_WRITE;

  info->map_base = base;
  for (size_t i = 0; i < accessor->number; ++i) {
    size_t size = cpu_mem_read_32(NRF_FICR_SIZERAMBLOCKS(i));
    info->map_base += size;
  }
  info->size = cpu_mem_read_32(NRF_FICR_SIZERAMBLOCKS(accessor->number));

  return 0;
}

static DEV_MEM_REQUEST(nrf5x_ram_request)
{
  size_t count = cpu_mem_read_32(NRF_FICR_NUMRAMBLOCK);
  uintptr_t base = 0x20000000;

  rq->err = 0;

  if (accessor->number >= count)
    rq->err = -ENOENT;
  else if (rq->band_mask & 0xfe)
    rq->err = -ENOENT;
  else if (rq->band_mask & 1) {
    for (size_t i = 0; i < accessor->number; ++i)
      base += cpu_mem_read_32(NRF_FICR_SIZERAMBLOCKS(i));

    dev_mem_mapped_op_helper(base, 1, rq);
  }

  kroutine_exec(&rq->base.kr);
}

static DEV_INIT(nrf5x_ram_init);
static DEV_CLEANUP(nrf5x_ram_cleanup);

#define nrf5x_ram_use dev_use_generic

DRIVER_DECLARE(nrf5x_ram_drv, 0, "nRF5x Ram", nrf5x_ram,
               DRIVER_MEM_METHODS(nrf5x_ram));

DRIVER_REGISTER(nrf5x_ram_drv);

static DEV_INIT(nrf5x_ram_init)
{
  dev->drv = &nrf5x_ram_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;
}

static DEV_CLEANUP(nrf5x_ram_cleanup)
{
}
