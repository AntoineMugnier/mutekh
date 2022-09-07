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
#include <hexo/flash.h>

#include <mutek/mem_alloc.h>
#include <arch/efm32/msc.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/class/mem.h>

DRIVER_PV(struct efm32_msc_context_s
{
});

static DEV_MEM_INFO(efm32_msc_info)
{
#if 0
  struct device_s *dev = accessor->dev;
  struct efm32_msc_context_s *pv = dev->drv_pv;
#endif

  if (band_index > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));

  switch (accessor->number)
    {
    case 0:                     /* RAM */
      info->type = DEV_MEM_RAM;
      info->size = cpu_mem_read_16(0x0fe081fa) << 10;
      info->flags |= DEV_MEM_VOLATILE |
        DEV_MEM_MAPPED_READ | DEV_MEM_MAPPED_WRITE |
        DEV_MEM_PARTIAL_WRITE | DEV_MEM_PARTIAL_READ |
        DEV_MEM_CROSS_READ | DEV_MEM_CROSS_WRITE |
        DEV_MEM_PAGE_READ | DEV_MEM_PAGE_WRITE;
      info->map_base = 0x20000000;
      break;
    case 1:                     /* FLASH code */
    case 2:                     /* FLASH userdata */
      info->type = DEV_MEM_FLASH;
      info->partial_log2 = 2;
      info->flags |= DEV_MEM_ERASE_ONE |
        DEV_MEM_MAPPED_READ | DEV_MEM_PARTIAL_WRITE | DEV_MEM_PARTIAL_READ |
        DEV_MEM_CROSS_READ | DEV_MEM_PAGE_READ | DEV_MEM_PAGE_WRITE;
#if (EFM32_SERIES(CONFIG_EFM32_CFAMILY) == 0)
      info->erase_cycles_p = 12; /* 20480 cycles */
#else
      info->erase_cycles_p = 11; /* 10240 cycles */
#endif
      info->erase_cycles_m = 5;
      if (accessor->number == 1)
        {
          info->page_log2 = EFM32_FLASH_PAGE_SIZE;
          info->size = CONFIG_EFM32_FLASHSIZE >> EFM32_FLASH_PAGE_SIZE;
          info->map_base = 0x00000000;
        }
      else
        {
#if EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32GG
          info->page_log2 = 11;
#else
          info->page_log2 = EFM32_FLASH_PAGE_SIZE;
#endif
          info->size = 1;
          info->map_base = 0x0fe00000;
        }
      info->erase_log2 = info->page_log2;
      break;
    default:
      UNREACHABLE();
    }

  return 0;
}

static DEV_MEM_REQUEST(efm32_msc_request)
{
#if 0
  struct device_s *dev = accessor->dev;
  struct efm32_msc_context_s *pv = dev->drv_pv;
#endif

  static const struct dev_mem_flash_op_info_s code_flash_info = {
    .base = 0,
    .end = CONFIG_EFM32_FLASHSIZE,
    .page_log2 = EFM32_FLASH_PAGE_SIZE,
    .page_erase = flash_page_erase,
    .write = flash_page_write,
  };

  static const struct dev_mem_flash_op_info_s user_flash_info = {
    .base = 0x0fe00000,
#if EFM32_MCU(CONFIG_EFM32_CFAMILY) == EFM32_MCU_EFM32GG
    .end = 0x0fe00000 + (1 << 11),
#else
    .end = 0x0fe00000 + (1 << EFM32_FLASH_PAGE_SIZE),
#endif
    .page_log2 = EFM32_FLASH_PAGE_SIZE,
    .page_erase = flash_page_erase,
    .write = flash_page_write,
  };

  rq->error = 0;
  switch (accessor->number)
    {
    case 0:                     /* RAM */
      rq->error = dev_mem_mapped_op_helper(0x20000000, 0x20000000 + CONFIG_EFM32_RAMSIZE, rq);
      break;
    case 1:                     /* FLASH code */
      rq->error = dev_mem_flash_op(&code_flash_info, rq);
      break;
    case 2:                     /* FLASH userdata */
      rq->error = dev_mem_flash_op(&user_flash_info, rq);
      break;
    default:
      UNREACHABLE();
    }

  dev_mem_rq_done(rq);
}

static DEV_USE(efm32_msc_use)
{
  struct device_accessor_s *accessor = param;

  switch (op)
    {
    case DEV_USE_GET_ACCESSOR:
      if (accessor->number > 2)
        return -ENOTSUP;
      return 0;
    case DEV_USE_LAST_NUMBER:
      accessor->number = 2;
      return 0;
    default:
      return dev_use_generic(param, op);
    }
}

static DEV_INIT(efm32_msc_init)
{
  if (((cpu_mem_read_8(0x0fe081e7) + 10) & 0xff) != EFM32_FLASH_PAGE_SIZE)
    return -EINVAL;

#if 0
  struct efm32_msc_context_s	*pv;

  /* allocate private driver data */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;
#endif

  uintptr_t                 addr;
  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL))
    return -1;
  assert(addr == EFM32_MSC_ADDR);

  return 0;

#if 0
 err_mem:
  mem_free(pv);
  return -1;
#endif
}

static DEV_CLEANUP(efm32_msc_cleanup)
{
#if 0
  struct efm32_msc_context_s	*pv = dev->drv_pv;

  mem_free(pv);
#endif
  return 0;
}

DRIVER_DECLARE(efm32_msc_drv, 0, "EFM32 Memory System Controller", efm32_msc,
               DRIVER_MEM_METHODS(efm32_msc));

DRIVER_REGISTER(efm32_msc_drv);

