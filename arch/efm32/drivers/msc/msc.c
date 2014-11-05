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
#include <arch/efm32_msc.h>

#include <device/resources.h>
#include <device/device.h>
#include <device/irq.h>
#include <device/class/mem.h>

uint32_t efm32_flash_erase(uintptr_t msc_addr, uintptr_t flash_addr);

uint32_t efm32_flash_write(uintptr_t msc_addr, uintptr_t flash_addr,
                           const uint32_t *data, uint32_t words_count);

struct efm32_msc_context_s
{
  uintptr_t                 addr;
  uint8_t                   page_log2;
};

#define EFM32_MSC_ADDR 0x400c0000

static DEV_MEM_INFO(efm32_msc_info)
{
  struct device_s *dev = mdev->dev;
  struct efm32_msc_context_s *pv = dev->drv_pv;

  if (band_index > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));

  switch (mdev->number)
    {
    case 0:                     /* RAM */
      info->type = DEV_MEM_RAM;
      info->size = cpu_mem_read_16(0x0fe081fa) << 10;
      info->flags |= DEV_MEM_WRITABLE | DEV_MEM_VOLATILE |
        DEV_MEM_MAPPED_READ | DEV_MEM_MAPPED_WRITE |
        DEV_MEM_PARTIAL_WRITE | DEV_MEM_PARTIAL_READ |
        DEV_MEM_CROSS_READ | DEV_MEM_CROSS_WRITE;
      info->map_base = 0x20000000;
      break;
    case 1:                     /* FLASH code */
    case 2:                     /* FLASH userdata */
      info->type = DEV_MEM_FLASH;
      info->partial_log2 = 2;
      info->flags |= DEV_MEM_WRITABLE | DEV_MEM_ERASE_ONE |
        DEV_MEM_MAPPED_READ | DEV_MEM_PARTIAL_WRITE | DEV_MEM_PARTIAL_READ |
        DEV_MEM_CROSS_READ;
      info->erase_cycles_p = 12; /* 20480 cycles */
      info->erase_cycles_m = 5;
      if (mdev->number == 1)
        {
          info->page_log2 = pv->page_log2;
          info->size = (cpu_mem_read_16(0x0fe081f8) << 10) >> info->page_log2;
          info->map_base = 0x00000000;
        }
      else
        {
#ifdef CONFIG_EFM32_GIANT_GECKO
          info->page_log2 = 11;
#else
          info->page_log2 = pv->page_log2;
#endif
          info->size = 1;
          info->map_base = 0x0fe00000;
        }
      info->erase_log2 = info->page_log2;
      break;
    default:
      return -ENOENT;
    }

  return 0;
}

static uint32_t efm32_msc_flash_op(uintptr_t base, uint_fast8_t page_log2, struct dev_mem_rq_s *rq)
{
  uint32_t err = 0;

  if (rq->type & (DEV_MEM_OP_PARTIAL_READ | DEV_MEM_OP_PAGE_READ))
    dev_mem_mapped_op_helper(base, page_log2, rq);

  if (rq->type & DEV_MEM_OP_PAGE_ERASE)
    {
      size_t i;
      for (i = 0; !err && i < rq->size; i++)
        {
          CPU_INTERRUPT_SAVESTATE_DISABLE;
          err |= efm32_flash_erase(EFM32_MSC_ADDR, base + rq->addr + (i << page_log2));
          CPU_INTERRUPT_RESTORESTATE;
        }
    }

  if (rq->type & DEV_MEM_OP_PAGE_WRITE)
    {
      size_t i;
      for (i = 0; !err && i < rq->size; i++)
        {
          uintptr_t mask = (1 << rq->sc_log2) - 1;
          CPU_INTERRUPT_SAVESTATE_DISABLE;
          err |= efm32_flash_write(EFM32_MSC_ADDR, base + rq->addr + (i << page_log2),
                   (void*)(rq->sc_data[i >> rq->sc_log2] + ((i & mask) << page_log2)),
                   1 << (page_log2 - 2));
          CPU_INTERRUPT_RESTORESTATE;
        }
    }
  else if (rq->type & DEV_MEM_OP_PARTIAL_WRITE)
    {
      CPU_INTERRUPT_SAVESTATE_DISABLE;
      err |= efm32_flash_write(EFM32_MSC_ADDR, base + rq->addr, (void*)rq->data, rq->size >> 2);
      CPU_INTERRUPT_RESTORESTATE;
    }

  return err;
}

static DEV_MEM_REQUEST(efm32_msc_request)
{
  struct device_s *dev = mdev->dev;
  struct efm32_msc_context_s *pv = dev->drv_pv;

  rq->err = 0;
  switch (mdev->number)
    {
    case 0:                     /* RAM */
      if (rq->type & (DEV_MEM_OP_PARTIAL_READ | DEV_MEM_OP_PARTIAL_WRITE |
                      DEV_MEM_OP_PAGE_READ | DEV_MEM_OP_PAGE_WRITE))
        dev_mem_mapped_op_helper(0x20000000, 0, rq);
      break;
    case 1:                     /* FLASH code */
      if (efm32_msc_flash_op(0x00000000, pv->page_log2, rq))
        rq->err = -EIO;
      break;
    case 2:                     /* FLASH userdata */
      if (efm32_msc_flash_op(0x0fe00000, pv->page_log2, rq))
        rq->err = -EIO;
      break;
    default:
      rq->err = -EINVAL;
    }

  kroutine_exec(&rq->rq.kr, cpu_is_interruptible());
}

static const struct driver_mem_s	efm32_msc_mem_drv =
{
  .class_		= DRIVER_CLASS_MEM,
  .f_info               = efm32_msc_info,
  .f_request		= efm32_msc_request,
};

static DEV_INIT(efm32_msc_init);
static DEV_CLEANUP(efm32_msc_cleanup);

const struct driver_s	efm32_msc_drv =
{
  .desc                 = "EFM32 Memory System Controller",
  .f_init		= efm32_msc_init,
  .f_cleanup		= efm32_msc_cleanup,
  .classes              = { &efm32_msc_mem_drv, 0 }
};

REGISTER_DRIVER(efm32_msc_drv);

static DEV_INIT(efm32_msc_init)
{
  struct efm32_msc_context_s	*pv;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  /* allocate private driver data */
  pv = mem_alloc(sizeof(*pv), (mem_scope_sys));
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  if (device_res_get_uint(dev, DEV_RES_MEM, 0, &pv->addr, NULL))
    goto err_mem;

  pv->page_log2 = (cpu_mem_read_8(0x0fe081e7) + 10) & 0xff;

  dev->drv = &efm32_msc_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_mem:
  mem_free(pv);
  return -1;
}

static DEV_CLEANUP(efm32_msc_cleanup)
{
  struct efm32_msc_context_s	*pv = dev->drv_pv;

  mem_free(pv);
}

