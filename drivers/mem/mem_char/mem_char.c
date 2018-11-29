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

*/

#define LOGK_MODULE "memC"

#include <hexo/types.h>
#include <hexo/endian.h>
#include <hexo/bit.h>
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/device.h>
#include <device/resources.h>
#include <device/driver.h>
#include <device/class/char.h>
#include <device/class/mem.h>

enum mem_char_state_e
{
  MEM_CHAR_IDLE,
  MEM_CHAR_WRITE_HEADER,
  MEM_CHAR_WRITE,
  MEM_CHAR_READ,
  MEM_CHAR_WAIT_HEADER,
};

struct mem_char_header_s
{
  uint8_t magic;
  uint8_t type;
  uint8_t addr[8];
  uint8_t size[4];
};

DRIVER_PV(struct mem_char_context_s
{
  struct device_char_s io;
  struct dev_char_rq_s io_rq;

  struct mem_char_header_s rq_header;

  dev_request_queue_root_t pending_rq;
  uint8_t current_sc_idx;

  enum mem_char_state_e state;

  /* size of memory (in pages) */
  uint64_t size;
  /* log2 of page size */
  uint8_t page_log2;
  /* log2 of erase page size */
  uint8_t erase_log2;
});

static void mem_char_end_rq(struct mem_char_context_s *pv, error_t err)
{
  assert(dev_mem_rq_head(&pv->pending_rq));
  struct dev_mem_rq_s * rq = dev_mem_rq_pop(&pv->pending_rq);
  rq->error = err;
  dev_mem_rq_done(rq);
}

static void mem_char_load_sc_header(struct mem_char_context_s *pv,
                                    struct mem_char_header_s *header,
                                    struct dev_mem_rq_s *rq)
{
  assert(pv->current_sc_idx < rq->page.sc_count);
  endian_le64_na_store(header->addr, rq->page.sc[pv->current_sc_idx].addr);
  endian_le32_na_store(header->size, 1 << rq->page.page_log2);
}

static void mem_char_start_next_rq(struct mem_char_context_s *pv, uint8_t idx)
{
  struct dev_mem_rq_s *rq = dev_mem_rq_head(&pv->pending_rq);
  assert(pv->state == MEM_CHAR_IDLE && rq);

  struct dev_char_rq_s *io_rq = &pv->io_rq;
  struct mem_char_header_s *header = &pv->rq_header;
  switch (rq->type)
    {
    case DEV_MEM_OP_PARTIAL_WRITE:
      endian_le64_na_store(header->addr, rq->partial.addr);
      endian_le32_na_store(header->size, rq->partial.size);
      header->type = 1;
      break;

    case DEV_MEM_OP_PARTIAL_READ:
      endian_le64_na_store(header->addr, rq->partial.addr);
      endian_le32_na_store(header->size, rq->partial.size);
      header->type = 0;
      break;

    case DEV_MEM_OP_PAGE_ERASE:
      pv->current_sc_idx = idx;
      header->type = 2;
      mem_char_load_sc_header(pv, header, rq);
      break;

    case DEV_MEM_OP_PAGE_WRITE:
      pv->current_sc_idx = idx;
      header->type = 1;
      mem_char_load_sc_header(pv, header, rq);
      break;

    case DEV_MEM_OP_PAGE_READ:
      pv->current_sc_idx = idx;
      header->type = 0;
      mem_char_load_sc_header(pv, header, rq);
      break;

    default:
      abort();
    }

  header->magic = 0xA3;

  io_rq->type = DEV_CHAR_WRITE_FRAME;
  io_rq->data = (void*)header;
  io_rq->size = sizeof(*header);

  pv->state = MEM_CHAR_WRITE_HEADER;
  DEVICE_OP(&pv->io, request, &pv->io_rq);
}

static KROUTINE_EXEC(mem_char_io_done)
{
  struct dev_char_rq_s *rq = dev_char_rq_from_kr(kr);
  struct device_s *dev = rq->pvdata;
  struct mem_char_context_s *pv = dev->drv_pv;
  assert(rq == &pv->io_rq);

  struct dev_mem_rq_s *mem_rq = dev_mem_rq_head(&pv->pending_rq);
  logk_trace("%s s:%u t:%u w:%u e:%u", __func__, pv->state, mem_rq->type, rq->size, rq->error);

  switch (pv->state)
    {
    case MEM_CHAR_WRITE_HEADER: {
      switch (rq->error)
        {
        case 0:
          if (rq->size > 0)
            {
              DEVICE_OP(&pv->io, request, rq);
              break;
            }

          switch (mem_rq->type)
            {
            case DEV_MEM_OP_PARTIAL_READ:
              rq->type = DEV_CHAR_READ_FRAME;
              pv->state = MEM_CHAR_READ;
              rq->size = mem_rq->partial.size;
              rq->data = mem_rq->partial.data;
              break;

            case DEV_MEM_OP_PARTIAL_WRITE:
              rq->type = DEV_CHAR_WRITE_FRAME;
              pv->state = MEM_CHAR_WRITE;
              rq->size = mem_rq->partial.size;
              rq->data = mem_rq->partial.data;
              break;

            case DEV_MEM_OP_PAGE_READ:
              rq->type = DEV_CHAR_READ_FRAME;
              pv->state = MEM_CHAR_READ;
              rq->size = 1 << mem_rq->page.page_log2;
              rq->data = mem_rq->page.sc[pv->current_sc_idx].data;
              break;

            case DEV_MEM_OP_PAGE_WRITE:
              rq->type = DEV_CHAR_WRITE_FRAME;
              pv->state = MEM_CHAR_WRITE;
              rq->size = 1 << mem_rq->page.page_log2;
              rq->data = mem_rq->page.sc[pv->current_sc_idx].data;
              break;

            case DEV_MEM_OP_PAGE_ERASE:
              rq->type = DEV_CHAR_READ_FRAME;
              rq->size = sizeof(pv->rq_header);
              rq->data = (void*)&pv->rq_header;
              memset(&pv->rq_header, 0, sizeof(pv->rq_header));
              pv->state = MEM_CHAR_WAIT_HEADER;
              break;

            default:
              abort();
            }

          DEVICE_OP(&pv->io, request, rq);
          break;

        default:
          mem_char_end_rq(pv, rq->error);
          goto new;
        }
      break;
    }

    case MEM_CHAR_WRITE:
    case MEM_CHAR_READ: {
      switch (rq->error)
        {
        case 0:
          if (rq->size > 0)
            {
              DEVICE_OP(&pv->io, request, rq);
              break;
            }

          rq->type = DEV_CHAR_READ_FRAME;
          rq->size = sizeof(pv->rq_header);
          rq->data = (void*)&pv->rq_header;
          memset(&pv->rq_header, 0, sizeof(pv->rq_header));
          pv->state = MEM_CHAR_WAIT_HEADER;
          DEVICE_OP(&pv->io, request, rq);
          break;

        default:
          mem_char_end_rq(pv, rq->error);
          goto new;
        }
      break;
      }

    case MEM_CHAR_WAIT_HEADER:
      switch (rq->error)
        {
        case 0:
          if (rq->size > 0)
            {
              DEVICE_OP(&pv->io, request, rq);
              break;
            }

          if (pv->rq_header.magic == 0xA3)
            {
              if ((mem_rq->type & _DEV_MEM_PAGE) &&
                  (++pv->current_sc_idx < mem_rq->page.sc_count))
                {
                  pv->state = MEM_CHAR_IDLE;
                  mem_char_start_next_rq(pv, pv->current_sc_idx);
                  break;
                }
              else
                {
                  mem_char_end_rq(pv, 0);
                  goto new;
                }
            }

        default:
          mem_char_end_rq(pv, rq->error);
          goto new;
        }
      break;

    case MEM_CHAR_IDLE:
    default:
      abort();
    }

  return;

 new:
  pv->state = MEM_CHAR_IDLE;
  if (!dev_rq_queue_isempty(&pv->pending_rq))
    mem_char_start_next_rq(pv, 0);
}

static DEV_MEM_REQUEST(mem_char_request)
{
  struct device_s *dev = accessor->dev;
  struct mem_char_context_s *pv = dev->drv_pv;
  error_t err = 0;

  LOCK_SPIN_IRQ(&dev->lock);

  switch (rq->type)
    {
    case DEV_MEM_OP_PAGE_ERASE:
    case DEV_MEM_OP_PAGE_READ:
    case DEV_MEM_OP_PAGE_WRITE: {
      if (rq->page.page_log2 < pv->page_log2)
        {
          err = -ENOENT;
          goto err;
        }
      uint_fast8_t i;
      for (i = 0; i < rq->page.sc_count; i++)
        {
          const struct dev_mem_page_sc_s *sc = rq->page.sc + i;
          if (!address_is_aligned(sc->addr, 1 << pv->page_log2))
            {
              err = -ENOENT;
              goto err;
            }
        }
      break;
    }
    case DEV_MEM_OP_PARTIAL_READ:
    case DEV_MEM_OP_PARTIAL_WRITE:
      break;

    default:
      err = -ENOTSUP;
      goto err;
    }

  dev_mem_rq_pushback(&pv->pending_rq, rq);

  if (pv->state == MEM_CHAR_IDLE)
    mem_char_start_next_rq(pv, 0);
 err:
  LOCK_RELEASE_IRQ(&dev->lock);

  if (err)
    {
      rq->error = err;
      dev_mem_rq_done(rq);
    }
}

static DEV_MEM_INFO(mem_char_info)
{
  struct device_s *dev = accessor->dev;
  struct mem_char_context_s *pv = dev->drv_pv;

  if (band_index > 0)
    return -ENOENT;

  if (accessor->number > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));
  info->type = DEV_MEM_FLASH;
  info->flags = DEV_MEM_PARTIAL_READ |
    DEV_MEM_PARTIAL_WRITE |
    DEV_MEM_ERASE_ONE |
    DEV_MEM_PAGE_READ |
    DEV_MEM_PAGE_WRITE |
    0;
  info->map_base = 0;
  info->size = pv->size;
  info->page_log2 = pv->page_log2;
  info->erase_log2 = pv->erase_log2;
  info->partial_log2 = 0;

  return 0;
}

static DEV_INIT(mem_char_init)
{
  struct mem_char_context_s *pv;
  uintptr_t num;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  dev->drv_pv = pv;

  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  /* 512bytes by default */
  if (device_get_param_uint(dev, "page_log2", &num))
    pv->page_log2 = 9;
  else
    pv->page_log2 = num;

  /* 512bytes by default */
  if (device_get_param_uint(dev, "erase_log2", &num))
    pv->erase_log2 = 9;
  else
    pv->erase_log2 = num;

  /* 1MBytes by default */
  if (device_get_param_uint(dev, "size", &num))
    pv->size = 2048;
  else
    pv->size = num;

  if (device_get_param_dev_accessor(dev, "io", &pv->io.base, DRIVER_CLASS_CHAR))
    goto err;

  dev_rq_queue_init(&pv->pending_rq);
  dev_char_rq_init(&pv->io_rq, mem_char_io_done);
  pv->io_rq.pvdata = dev;

  pv->state = MEM_CHAR_IDLE;

  return 0;

 err:
  mem_free(pv);
  return -EINVAL;
}

static DEV_CLEANUP(mem_char_cleanup)
{
  struct mem_char_context_s *pv = dev->drv_pv;

  if (pv->state != MEM_CHAR_IDLE ||
      dev_rq_queue_isempty(&pv->pending_rq))
    return -EBUSY;

  device_put_accessor(&pv->io.base);
  mem_free(pv);

  return 0;
}

#define mem_char_use dev_use_generic
#define mem_char_cancel (dev_mem_cancel_t*)&dev_driver_notsup_fcn

DRIVER_DECLARE(mem_char_drv, 0, "Mem Char", mem_char,
               DRIVER_MEM_METHODS(mem_char));

DRIVER_REGISTER(mem_char_drv,
                DEV_ENUM_FDTNAME_ENTRY("mem_char"));
