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

  Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2014
*/

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>
#include <hexo/iospace.h>
#include <hexo/flash.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

# include <device/resources.h>
# include <device/device.h>
# include <device/class/mem.h>

#include <arch/nrf5x/nvmc.h>
#include <arch/nrf5x/ficr.h>
#include <arch/nrf5x/uicr.h>

#define NVMC_ADDR NRF_PERIPHERAL_ADDR(NRF5X_NVMC)

enum nrf5x_flash_bank
{
  BANK_CODE,
  BANK_UICR,
};

struct nrf5x_nvmc_private_s
{
  struct dev_mem_flash_op_info_s flash_info;
};

DRIVER_PV(struct nrf5x_nvmc_private_s);

size_t nrf5x_flash_page_size(void)
{
  return cpu_mem_read_32(NRF_FICR_CODEPAGESIZE);
}

size_t nrf5x_flash_page_count(void)
{
  return cpu_mem_read_32(NRF_FICR_CODESIZE);
}

static DEV_MEM_INFO(nrf5x_nvmc_info)
{
  if (band_index > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));

  uint_fast8_t page_log2 = bit_msb_index(cpu_mem_read_32(NRF_FICR_CODEPAGESIZE));

  switch (accessor->number)
    {
    case BANK_CODE:
      info->size = cpu_mem_read_32(NRF_FICR_CODESIZE);
      info->map_base = 0;
      info->page_log2 = page_log2;
      info->erase_log2 = page_log2;
      goto flash_common;

    case BANK_UICR:
      info->size = 1;
      info->map_base = NRF_UICR_BASE;
      info->page_log2 = 7;
      info->erase_log2 = 7;

    flash_common:
      info->erase_cycles_p = 14; // 20K cycles nominal
      info->erase_cycles_m = 0;
      info->type = DEV_MEM_FLASH;
      info->partial_log2 = 2;
      info->flags = 0
        | DEV_MEM_PAGE_READ
        | DEV_MEM_PAGE_WRITE
        | DEV_MEM_ERASE_ONE
        | DEV_MEM_MAPPED_READ
        | DEV_MEM_PARTIAL_WRITE
        | DEV_MEM_PARTIAL_READ
        | DEV_MEM_CROSS_READ;
      break;

    default:
      return -ENOENT;
    }

  return 0;
}

static DEV_MEM_REQUEST(nrf5x_nvmc_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_nvmc_private_s *pv = dev->drv_pv;
  static const struct dev_mem_flash_op_info_s uicr_info = {
    .base = NRF_UICR_BASE,
    .end = NRF_UICR_BASE + 128,
    .page_log2 = 7,
    .page_erase = flash_page_erase,
    .write = flash_page_write,
  };

  rq->error = 0;
  switch (accessor->number)
    {
    case BANK_CODE:
      rq->error = dev_mem_flash_op(&pv->flash_info, rq);
      break;

    case BANK_UICR:
      rq->error = dev_mem_flash_op(&uicr_info, rq);
      break;

    default:
      UNREACHABLE();
    }

  dev_mem_rq_done(rq);
}

static DEV_USE(nrf5x_nvmc_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR:
      if (accessor->number > 1)
        return -ENOTSUP;
      return 0;

    case DEV_USE_LAST_NUMBER:
      accessor->number = 1;
      return 0;

    default:
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(nrf5x_nvmc_init)
{
  struct nrf5x_nvmc_private_s	*pv;

  uintptr_t addr;
  error_t err = device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL);

  if (err)
    return err;

  assert(addr == NRF_PERIPHERAL_ADDR(NRF5X_NVMC));

  /* allocate private driver data */
  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;


  dev->drv_pv = pv;

  pv->flash_info.base = 0;
  pv->flash_info.end = nrf5x_flash_page_size() * nrf5x_flash_page_count();
  pv->flash_info.page_log2 = bit_msb_index(cpu_mem_read_32(NRF_FICR_CODEPAGESIZE));
  pv->flash_info.page_erase = flash_page_erase;
  pv->flash_info.write = flash_page_write;

  return 0;
}

static DEV_CLEANUP(nrf5x_nvmc_cleanup)
{
  struct nrf5x_nvmc_private_s *pv = dev->drv_pv;

  mem_free(pv);

  return 0;
}

DRIVER_DECLARE(nrf5x_nvmc_drv, 0, "nRF5x NVMC", nrf5x_nvmc,
               DRIVER_MEM_METHODS(nrf5x_nvmc));

DRIVER_REGISTER(nrf5x_nvmc_drv);

