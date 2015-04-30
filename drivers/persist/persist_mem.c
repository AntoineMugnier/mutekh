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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
*/

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <device/driver.h>
#include <device/request.h>
#include <device/resources.h>
#include <device/class/persist.h>
#include <device/class/mem.h>

#define dprintk(k...) do {} while (0)
//#define dprintk printk

enum pmem_action_e
{
  PMEM_STORAGE_FOREACH_DONE,
  PMEM_STORAGE_FOREACH_CONTINUE,
  PMEM_STORAGE_FOREACH_BREAK,
  PMEM_STORAGE_FOREACH_POSTPONE,
};

enum pmem_desc_reserved_e
{
  PMEM_DESC_ERASED = 0,
  PMEM_DESC_WRITTEN = 1,
  PMEM_DESC_BUSY = 3,
  PMEM_DESC_FREE = 7,
};

#define PMEM_MAGIC 0x58a1fb3e

enum pmem_header_state_e
{
  PMEM_SLOT_HEADER_ERASED = 0,
  PMEM_SLOT_HEADER_MAIN = 1,
  PMEM_SLOT_HEADER_BUSY = 3,
  PMEM_SLOT_HEADER_FREE = 7,
};

enum pmem_slot_state_e
{
  PMEM_SLOT_CLEAN,
  PMEM_SLOT_EMPTY,
  PMEM_SLOT_MUST_PACK,
  PMEM_SLOT_BROKEN,
};

struct pmem_header_s
{
  uint32_t magic;
  uint32_t state : 3;
  uint32_t size : 29;
};

struct pmem_private_s;

typedef enum pmem_action_e pmem_foreach_func_t(struct pmem_private_s *pv,
                                 const struct dev_persist_descriptor_s *desc,
                                 const void *data);

typedef void pmem_cb_t(struct pmem_private_s *pv);

struct pmem_private_s
{
  struct dev_mem_rq_s mem_rq;
  struct device_mem_s storage;
  struct dev_mem_info_s info;

  dev_request_queue_root_t queue;
  union {
    struct device_s *dev;
    struct dev_persist_rq_s *current;
  };

  uint64_t addr;
  uint64_t size;

  uint8_t current_slot;
  uint8_t inited;

  union {
    struct pmem_header_s header;
    struct dev_persist_descriptor_s descriptor;
  } tmp;

  struct {
    uintptr_t point;
    uintptr_t size;
    pmem_foreach_func_t *func;
    pmem_cb_t *done;
  } foreach;

  pmem_cb_t *mem_rq_done;
};

static void pmem_storage_foreach_start(struct pmem_private_s *pv,
                                       uintptr_t start, uintptr_t size,
                                       pmem_foreach_func_t *func,
                                       pmem_cb_t *done);
static void pmem_storage_foreach_continue(struct pmem_private_s *pv);

static void pmem_request_run_first(struct device_s *dev);
static void pmem_request_callback(struct device_s *dev, struct dev_persist_rq_s *rq);
static void pmem_pack(struct pmem_private_s *pv);
static void pmem_pack_copy(struct pmem_private_s *pv);
static void pmem_item_add(struct pmem_private_s *pv,
                          const struct dev_persist_descriptor_s *desc,
                          const void *data,
                          pmem_cb_t *cb);
static enum pmem_action_e pmem_read_find(struct pmem_private_s *pv,
                                         const struct dev_persist_descriptor_s *desc,
                                         const void *data);
static void pmem_read_done(struct pmem_private_s *pv);
static void pmem_storage_foreach_continue(struct pmem_private_s *pv);
static enum pmem_action_e pmem_pack_copy_one(struct pmem_private_s *pv,
                               const struct dev_persist_descriptor_s *desc,
                               const void *data);
static void pmem_pack_done(struct pmem_private_s *pv);
static void pmem_slot_init(struct pmem_private_s *pv);
static void pmem_slot_init2(struct pmem_private_s *pv);

static KROUTINE_EXEC(pmem_mem_kr)
{
  struct pmem_private_s *pv = KROUTINE_CONTAINER(kr, *pv, mem_rq.base.kr);
  pv->mem_rq_done(pv);
}

static void pmem_storage_erase(struct pmem_private_s *pv,
                               uint32_t base, uint32_t size,
                               pmem_cb_t *cb)
{
  pv->mem_rq.type = 0
    | DEV_MEM_OP_CACHE_INVALIDATE
    | DEV_MEM_OP_PAGE_ERASE
    ;
  pv->mem_rq.size = size >> pv->info.erase_log2;
  pv->mem_rq.band_mask = 1;
  pv->mem_rq.addr = pv->addr + base;

  pv->mem_rq_done = cb;

  kroutine_init(&pv->mem_rq.base.kr, pmem_mem_kr, KROUTINE_IMMEDIATE);
  DEVICE_OP(&pv->storage, request, &pv->mem_rq);
}

static void pmem_storage_write(struct pmem_private_s *pv,
                               uint32_t base, const void *data, uint32_t size,
                               pmem_cb_t *cb)
{
  pv->mem_rq.type = 0
    | DEV_MEM_OP_PARTIAL_WRITE
    ;
  pv->mem_rq.size = size;
  pv->mem_rq.band_mask = 1;
  pv->mem_rq.addr = pv->addr + base;
  pv->mem_rq.data = (void *)data;

  pv->mem_rq_done = cb;

  kroutine_init(&pv->mem_rq.base.kr, pmem_mem_kr, KROUTINE_IMMEDIATE);
  DEVICE_OP(&pv->storage, request, &pv->mem_rq);
}

static void pmem_current_done(struct pmem_private_s *pv)
{
  struct device_s *dev;

  if (!pv->current) {
    assert(!pv->inited);
    pv->inited = 1;
    pmem_request_run_first(pv->dev);
    return;
  }

  assert(pv->current);
  dev = pv->current->rq.drvdata;

  pmem_request_callback(dev, pv->current);
}

static void pmem_erase(struct pmem_private_s *pv)
{
  pmem_storage_erase(pv, 0, pv->size, pmem_slot_init);
}

static void pmem_slot_init(struct pmem_private_s *pv)
{
  pv->tmp.header.magic = PMEM_MAGIC;
  pv->tmp.header.size = pv->size / 2;
  pv->tmp.header.state = PMEM_SLOT_HEADER_BUSY;

  pmem_storage_write(pv, 0, &pv->tmp.header, sizeof(pv->tmp.header), pmem_slot_init2);
}

static void pmem_slot_init2(struct pmem_private_s *pv)
{
  pv->tmp.header.magic = PMEM_MAGIC;
  pv->tmp.header.size = pv->size / 2;
  pv->tmp.header.state = PMEM_SLOT_HEADER_MAIN;

  pmem_storage_write(pv, 0, &pv->tmp.header, sizeof(pv->tmp.header), pmem_current_done);
}

static enum pmem_slot_state_e pmem_slot_state(struct pmem_private_s *pv, uint8_t slot)
{
  uintptr_t base = pv->info.map_base + pv->size / 2 * slot;
  uintptr_t end = base + pv->size / 2;
  const struct pmem_header_s *header = (const void *)base;
  enum pmem_slot_state_e state = PMEM_SLOT_CLEAN;
  uintptr_t point = base + sizeof(*header);

  if (header->magic != PMEM_MAGIC)
    return PMEM_SLOT_BROKEN;

  if (header->state != PMEM_SLOT_HEADER_MAIN)
    return PMEM_SLOT_BROKEN;

  if (header->size != pv->size / 2)
    return PMEM_SLOT_BROKEN;

  while (point < end) {
    const struct dev_persist_descriptor_s *desc = (const void *)point;

    switch (desc->state) {
    case DEV_PERSIST_STATE_FREE:
      for (const uint32_t *check = (const void *)(desc + 1);
           check < (const uint32_t *)end;
           ++check)
        if (*check != 0xffffffff)
          state = PMEM_SLOT_MUST_PACK;
      return state;

    case DEV_PERSIST_STATE_WRITTEN:
    case DEV_PERSIST_STATE_ERASED:
      break;

    case DEV_PERSIST_STATE_BUSY:
      state = PMEM_SLOT_MUST_PACK;
      break;
    }

    point += desc->size * 4;
    // We may have point == end if slot is full to the last byte
    if (point < base + sizeof(*header) || point > end)
      return PMEM_SLOT_BROKEN;
  }

  return state;
}

static void pmem_discover(struct device_s *dev, struct pmem_private_s *pv)
{
  enum pmem_slot_state_e state[2] = {PMEM_SLOT_BROKEN, PMEM_SLOT_BROKEN};

  for (uint8_t i = 0; i < 2; ++i) {
    state[i] = pmem_slot_state(pv, i);

    if (state[i] == PMEM_SLOT_CLEAN) {
      pv->current_slot = i;
      pv->inited = 1;
      pmem_request_run_first(pv->dev);
      return;
    }
  }

  // None of them is CLEAN, but we may have one with MUST_PACK ?
  for (uint8_t i = 0; i < 2; ++i) {
    if (state[i] == PMEM_SLOT_MUST_PACK) {
      pv->current_slot = i;
      pv->dev = dev;
      pmem_pack(pv);
      return;
    }
  }

  // None is CLEAN, nor MUST_PACK, erase first one
  pv->current_slot = 0;
  pv->dev = dev;
  pmem_erase(pv);
}

static void pmem_pack(struct pmem_private_s *pv)
{
  uint32_t start = 0;
  uint32_t size = pv->size / 2;

  // Erase other slot
  if (pv->current_slot == 0)
    start += size;

  pmem_storage_erase(pv, start, size, pmem_pack_copy);
}

static void pmem_pack_copy(struct pmem_private_s *pv)
{
  uint32_t start = sizeof(struct pmem_header_s);
  uint32_t size = pv->size / 2;

  // Swap slots
  pv->current_slot = !pv->current_slot;

  if (pv->current_slot == 0)
    start += size;

  pmem_storage_foreach_start(pv, start, size, pmem_pack_copy_one, pmem_pack_done);
}

static void pmem_pack_done(struct pmem_private_s *pv)
{
}

static void pmem_storage_foreach_start(struct pmem_private_s *pv,
                                       uintptr_t start, uintptr_t size,
                                       pmem_foreach_func_t *func,
                                       pmem_cb_t *done)
{
  pv->foreach.point = start;
  pv->foreach.size = size;
  pv->foreach.func = func;
  pv->foreach.done = done;

  pmem_storage_foreach_continue(pv);
}

static void pmem_storage_foreach_continue(struct pmem_private_s *pv)
{
  while (pv->foreach.size >= sizeof(struct dev_persist_descriptor_s)) {
    const struct dev_persist_descriptor_s *desc
      = (const void *)(pv->info.map_base + pv->foreach.point);
    uint32_t size = desc->size * 4 + sizeof(*desc);
    bool_t postpone = 0;

    if (size > pv->foreach.size)
      break;

    switch (desc->reserved) {
    case DEV_PERSIST_STATE_BUSY: // Should not happen
    case DEV_PERSIST_STATE_ERASED:
      break;

    case DEV_PERSIST_STATE_FREE:
      pv->foreach.size = 0;
      continue;

    case DEV_PERSIST_STATE_WRITTEN:
      switch (pv->foreach.func(pv, desc, desc + 1)) {
      case PMEM_STORAGE_FOREACH_POSTPONE:
        postpone = 1;
        break;

      case PMEM_STORAGE_FOREACH_CONTINUE:
        break;

      case PMEM_STORAGE_FOREACH_DONE:
        pv->foreach.size = 0;
        postpone = 1;
        break;

      case PMEM_STORAGE_FOREACH_BREAK:
        pv->foreach.size = 0;
        return;
      }
      break;
    }

    pv->foreach.point += size;
    pv->foreach.size -= size;

    if (postpone)
      return;
  }

 done:
  pv->foreach.done(pv);
  return;
}


static enum pmem_action_e pmem_pack_copy_one(struct pmem_private_s *pv,
                                             const struct dev_persist_descriptor_s *desc,
                                             const void *data)
{
  pmem_item_add(pv, desc, data, pmem_storage_foreach_continue);

  return PMEM_STORAGE_FOREACH_POSTPONE;
}

static void pmem_read(struct pmem_private_s *pv)
{
  uint32_t start = 0;
  uint32_t size = pv->size / 2;
  if (pv->current_slot == 1)
    start = size;

  pv->current->err = -ENOENT;
  pmem_storage_foreach_start(pv, start, size, pmem_read_find, pmem_read_done);
}

static enum pmem_action_e pmem_read_find(struct pmem_private_s *pv,
                                         const struct dev_persist_descriptor_s *desc,
                                         const void *data)
{
  if (pv->current->descriptor->uid + pv->current->uid_offset != desc->uid)
    return PMEM_STORAGE_FOREACH_CONTINUE;

  pv->current->data = data;
  return PMEM_STORAGE_FOREACH_BREAK;
}

static void pmem_read_done(struct pmem_private_s *pv)
{
  struct device_s *dev;

  assert(pv->current);
  dev = pv->current->rq.drvdata;
  pv->current->err = 0;

  pmem_request_callback(dev, pv->current);
}

static void pmem_write(struct pmem_private_s *pv)
{
  struct dev_persist_descriptor_s desc = *pv->current->descriptor;
  desc.uid += pv->current->uid_offset;

  pmem_item_add(pv, &desc, pv->current->data, pmem_current_done);
}

typedef void rq_starter(struct pmem_private_s *pv);
static rq_starter *starter[] = {
  [DEV_PERSIST_ERASE] = pmem_erase,
  [DEV_PERSIST_PACK] = pmem_pack,
  [DEV_PERSIST_READ] = pmem_read,
  [DEV_PERSIST_WRITE] = pmem_write,
};

static
void pmem_request_run_first(struct device_s *dev)
{
  struct pmem_private_s *pv = dev->drv_pv;

  assert(pv);

  if (pv->current)
    return;

  struct dev_persist_rq_s *rq = dev_persist_rq_s_from_rq(dev_request_queue_pop(&pv->queue));

  if (!rq)
    return;


  if (rq->op != DEV_PERSIST_ERASE && !pv->inited)
    return pmem_discover(dev, pv);

  pv->current = rq;
  rq->rq.drvdata = dev;
  starter[rq->op](pv);
}

static void pmem_request_callback(struct device_s *dev, struct dev_persist_rq_s *rq)
{
  struct pmem_private_s *pv = dev->drv_pv;

  assert(pv->current == rq);
  pv->current = NULL;
  pv->current->rq.drvdata = NULL;

  lock_release(&dev->lock);
  kroutine_exec(&rq->rq.kr);
  lock_spin(&dev->lock);

  pmem_request_run_first(dev);
}

static
DEV_PERSIST_REQUEST(pmem_request)
{
  struct device_s *dev = accessor->dev;
  struct pmem_private_s *pv = dev->drv_pv;

  LOCK_SPIN_IRQ(&dev->lock);

  dev_request_queue_pushback(&pv->queue, &rq->rq);
  pmem_request_run_first(dev);

  LOCK_RELEASE_IRQ(&dev->lock);
}

static DEV_INIT(pmem_init);
static DEV_CLEANUP(pmem_cleanup);

#define pmem_use dev_use_generic

DRIVER_DECLARE(persist_mem_drv, 0, "persistent -> mem", pmem,
               DRIVER_PERSIST_METHODS(pmem));

DRIVER_REGISTER(persist_mem_drv);

#define FLAGS_REQUIRED (DEV_MEM_MAPPED_READ | DEV_MEM_WRITABLE | DEV_MEM_ERASE_ONE | DEV_MEM_CROSS_WRITE)

static DEV_INIT(pmem_init)
{
  struct pmem_private_s *pv;
  uint32_t page_count;
  error_t err = 0;
  uintptr_t addr, size;

  dev->status = DEVICE_DRIVER_INIT_FAILED;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  memset(pv, 0, sizeof(*pv));
  if (!pv)
    return -ENOMEM;

  err = device_get_param_dev_accessor(dev, "storage", &pv->storage, DRIVER_CLASS_MEM);
  if (err)
    goto err_pv;

  err = device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, &size);
  if (err)
    goto err_pv;

  DEVICE_OP(&pv->storage, info, &pv->info, 0);

  // Keep truncated down size for a multiple of 2 pages.
  page_count = size >> pv->info.page_log2;
  page_count &= ~1;
  pv->size = page_count << pv->info.page_log2;
  pv->addr = addr;

  if ((pv->info.flags & FLAGS_REQUIRED) != FLAGS_REQUIRED
      || page_count < 2
      || ((pv->size / 2) & ((1 << pv->info.erase_log2) - 1))) {
    err = -ENOTSUP;
    goto err_pv;
  }

  dev_request_queue_init(&pv->queue);

  dev->drv_pv = pv;
  dev->drv = &persist_mem_drv;
  dev->status = DEVICE_DRIVER_INIT_DONE;

  return 0;

 err_pv:
  mem_free(pv);
  return err;
}

static DEV_CLEANUP(pmem_cleanup)
{
  struct pmem_private_s *pv = dev->drv_pv;

  device_put_accessor(&pv->storage);
  dev_request_queue_destroy(&pv->queue);
  mem_free(pv);
}
