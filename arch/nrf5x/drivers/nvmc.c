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
#include <hexo/iospace.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#if defined(CONFIG_DRIVER_NRF5X_NVMC) || defined(CONFIG_DRIVER_NRF5X_PERSIST)
# include <device/resources.h>
# include <device/device.h>
# include <device/class/mem.h>
# include <device/class/persist.h>
#endif

#include <arch/nrf5x/nvmc.h>
#include <arch/nrf5x/ficr.h>
#include <arch/nrf5x/uicr.h>

#define NVMC_ADDR NRF_PERIPHERAL_ADDR(NRF5X_NVMC)

enum nrf5x_flash_bank
{
  BANK_CODE,
  BANK_UICR,
};

#if 0
# define dprintk printk
# define dhexdumpk hexdumpk
#else
# define dprintk(k...) do {} while (0)
# define dhexdumpk(k...) do {} while (0)
#endif

#if defined(CONFIG_DRIVER_NRF5X_NVMC) || defined(CONFIG_DRIVER_NRF5X_PERSIST)
struct nrf5x_nvmc_private_s
{
  struct dev_request_dlqueue_s queue;

  uintptr_t addr;
  size_t slot_size;

  uint8_t current_slot;

  size_t used;
  size_t reclaimable;
  size_t available;
};
#endif

size_t nrf5x_flash_page_size(void)
{
  return cpu_mem_read_32(NRF_FICR_CODEPAGESIZE);
}

size_t nrf5x_flash_page_count(void)
{
  return cpu_mem_read_32(NRF_FICR_CODESIZE);
}

void nrf5x_flash_page_erase(uintptr_t page)
{
#if defined(CONFIG_ARCH_NRF51)
  uint32_t cr0_len = cpu_mem_read_32(NRF_FICR_CLENR0);
  if (cr0_len == 0xffffffff)
    cr0_len = 0;
  uint8_t reg = page < cr0_len ? NRF_NVMC_ERASEPCR0 : NRF_NVMC_ERASEPCR1;
#else
  uint8_t reg = NRF_NVMC_ERASEPAGE;
#endif

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  nrf_reg_set(NVMC_ADDR, NRF_NVMC_CONFIG, NRF_NVMC_CONFIG_ERASE);
  while (!nrf_reg_get(NVMC_ADDR, NRF_NVMC_READY))
    ;

  nrf_reg_set(NVMC_ADDR, reg, page);
  while (!nrf_reg_get(NVMC_ADDR, NRF_NVMC_READY))
    ;

  nrf_reg_set(NVMC_ADDR, NRF_NVMC_CONFIG, 0);
  while (!nrf_reg_get(NVMC_ADDR, NRF_NVMC_READY))
    ;
  CPU_INTERRUPT_RESTORESTATE;
}

void nrf5x_flash_write(
  uintptr_t dest, const void *data, size_t word_count)
{
  uint32_t i;
  volatile uint32_t *dst = (volatile uint32_t *)dest;
  const uint32_t *src = data;

  CPU_INTERRUPT_SAVESTATE_DISABLE;

  nrf_reg_set(NVMC_ADDR, NRF_NVMC_CONFIG, NRF_NVMC_CONFIG_WRITE);
  while (!nrf_reg_get(NVMC_ADDR, NRF_NVMC_READY))
    ;

  if (((uintptr_t)src & 3) == 0) {
    for (i = 0; i < word_count; ++i)
      dst[i] = src[i];
  } else {
    for (i = 0; i < word_count; ++i)
      dst[i] = endian_32_na_load(src + i);
  }
  while (!nrf_reg_get(NVMC_ADDR, NRF_NVMC_READY))
    ;

  nrf_reg_set(NVMC_ADDR, NRF_NVMC_CONFIG, 0);
  while (!nrf_reg_get(NVMC_ADDR, NRF_NVMC_READY))
    ;

  CPU_INTERRUPT_RESTORESTATE;
}

#ifdef CONFIG_DRIVER_NRF5X_PERSIST

#define NRF5X_PERSIST_MAGIC 0x58a1fb3e

enum slot_state_e
{
  SLOT_CLEAN,
  SLOT_EMPTY,
  SLOT_MUST_PACK,
  SLOT_BROKEN,
};

struct nrf5x_persist_header_s
{
  uint32_t magic;
  uint32_t size : 29;
  uint32_t state : 3;
};

struct nrf5x_persist_slot_state_s
{
  enum slot_state_e state;
  size_t used;
  size_t reclaimable;
  size_t available;
};

static error_t nrf5x_persist_erase(struct nrf5x_nvmc_private_s *pv,
                                   struct dev_persist_rq_s *rq);

static bool_t nrf5x_persist_storage_is_clean(uintptr_t base, size_t size)
{
  assert(IS_ALIGNED(base, 4) && IS_ALIGNED(size, 4));

  const uint32_t *begin = (const uint32_t *)base;
  const uint32_t *end = (const uint32_t *)(base + size);

  for (; begin < end; begin++)
    if (*begin != 0xffffffff)
      return 0;

  return 1;
}

static void nrf5x_persist_storage_erase(uintptr_t base, size_t size)
{
  uint32_t page_size = cpu_mem_read_32(NRF_FICR_CODEPAGESIZE);

  assert(IS_ALIGNED(base, page_size) && IS_ALIGNED(size, page_size));

  while (size) {
    nrf5x_flash_page_erase(base);
    base += page_size;
    size -= page_size;
  }
}

static void nrf5x_persist_storage_write(uintptr_t base,
                                        const void *data,
                                        size_t size)
{
  assert(IS_ALIGNED(base, 4) && IS_ALIGNED(data, 4) && IS_ALIGNED(size, 4));

  dprintk("Write to %p:\n", base);
  dhexdumpk(0, data, size);
  dprintk(" before:\n");
  dhexdumpk(base, (void*)base, size);

  nrf5x_flash_write(base, data, size / sizeof(uint32_t));

  dprintk(" after:\n");
  dhexdumpk(base, (void*)base, size);
}

static inline size_t item_size(const struct dev_persist_descriptor_s *desc)
{
  return sizeof(*desc) + ALIGN_VALUE_UP(desc->size, 4);
}

static void nrf5x_persist_item_invalidate(struct nrf5x_nvmc_private_s *pv,
                                          const struct dev_persist_descriptor_s *hdr)
{
  struct dev_persist_descriptor_s desc = *hdr;
  desc.state = DEV_PERSIST_STATE_ERASED;

  dprintk("%s %d %x %d at %p\n", __FUNCTION__, desc.type, desc.uid, desc.size, hdr);

  nrf5x_persist_storage_write((uintptr_t)hdr, &desc, sizeof(desc));

  pv->reclaimable += item_size(&desc);
}

static void nrf5x_persist_write_blob(struct nrf5x_nvmc_private_s *pv,
                                     uintptr_t writep,
                                     struct dev_persist_descriptor_s *desc,
                                     const void *data,
                                     const struct dev_persist_descriptor_s *to_invalidate)
{
  uint32_t size = item_size(desc);

  dprintk("%s %d %x %d at %08x\n", __FUNCTION__, desc->type, desc->uid, desc->size, writep);

  desc->state = DEV_PERSIST_STATE_BUSY;
  nrf5x_persist_storage_write(writep, desc, sizeof(*desc));

  nrf5x_persist_storage_write(writep + sizeof(*desc), data, desc->size);

  if (to_invalidate)
    nrf5x_persist_item_invalidate(pv, to_invalidate);

  desc->state = DEV_PERSIST_STATE_WRITTEN;
  nrf5x_persist_storage_write(writep, desc, sizeof(*desc));

  pv->used += size;
  pv->available -= size;
}

static void nrf5x_persist_write_counter(struct nrf5x_nvmc_private_s *pv,
                                        uintptr_t writep,
                                        struct dev_persist_descriptor_s *desc,
                                        uint64_t value,
                                        const struct dev_persist_descriptor_s *to_invalidate)
{
  uint32_t size = item_size(desc);

  assert(desc->size > sizeof(value));

  dprintk("%s %d %x %d at %08x\n", __FUNCTION__, desc->type, desc->uid, desc->size, writep);

  desc->state = DEV_PERSIST_STATE_BUSY;
  nrf5x_persist_storage_write(writep, desc, sizeof(*desc));

  nrf5x_persist_storage_write(writep + sizeof(*desc), &value, sizeof(value));

  if (to_invalidate)
    nrf5x_persist_item_invalidate(pv, to_invalidate);

  desc->state = DEV_PERSIST_STATE_WRITTEN;
  nrf5x_persist_storage_write(writep, desc, sizeof(*desc));

  pv->used += size;
  pv->available -= size;
}

static uint64_t nrf5x_persist_counter_base_get(const struct dev_persist_descriptor_s *desc)
{
  const uint64_t *valuep = (const void *)(desc + 1);
  return *valuep;
}

static uint32_t nrf5x_persist_counter_offset_get(const struct dev_persist_descriptor_s *desc)
{
  const uint32_t *begin = (uint32_t *)((const uint64_t *)(desc + 1) + 1);
  size_t total = (item_size(desc) - sizeof(*desc) - sizeof(uint64_t)) / sizeof(uint32_t);
  size_t low = 0, high = total;
  uint32_t zeroes = 0;

  // Search in [low:high)
  while (low < high) {
    size_t cur = (low + high) / 2;

    if (begin[cur] == 0)
      low = cur + 1;
    else
      high = cur;
  }

  if (low < total)
    zeroes = __builtin_ctz(begin[low]);
  return zeroes + low * sizeof(uint32_t) * 8;
}

static error_t nrf5x_persist_pack(struct nrf5x_nvmc_private_s *pv,
                               struct dev_persist_rq_s *rq)
{
  size_t size = pv->slot_size;
  uintptr_t current = pv->addr;
  uintptr_t other = pv->addr;
  uintptr_t writep = other;
  struct nrf5x_persist_header_s header;

  if (!pv->reclaimable)
    return 0;

  if (pv->current_slot == 0)
    other += size;
  else
    current += size;

  if (!nrf5x_persist_storage_is_clean(other, size))
    nrf5x_persist_storage_erase(other, size);

  header.magic = NRF5X_PERSIST_MAGIC;
  header.size = size;
  header.state = DEV_PERSIST_STATE_BUSY;
  nrf5x_persist_storage_write(other, &header, sizeof(header));

  writep = sizeof(header) + other;

  pv->used = sizeof(header);
  pv->available = pv->slot_size - sizeof(header);
  pv->reclaimable = 0;

  uintptr_t point = current + sizeof(struct nrf5x_persist_header_s);

  while (point > current && point < current + size) {
    const struct dev_persist_descriptor_s *curp = (const void*)point;
    struct dev_persist_descriptor_s cur = *curp;

    if (point + item_size(curp) > current + size)
      break;

    point += item_size(curp);

    if (curp->state != DEV_PERSIST_STATE_WRITTEN)
      continue;

    switch (cur.type) {
    case DEV_PERSIST_BLOB:
      nrf5x_persist_write_blob(pv, writep, &cur, curp + 1, NULL);
      break;

    case DEV_PERSIST_COUNTER:
      nrf5x_persist_write_counter(pv, writep, &cur,
                                  nrf5x_persist_counter_base_get(curp)
                                  + nrf5x_persist_counter_offset_get(curp),
                                  NULL);
      break;
    }

    writep += item_size(&cur);
  }

  header.state = DEV_PERSIST_STATE_WRITTEN;
  nrf5x_persist_storage_write(other, &header, sizeof(header));

  if (!nrf5x_persist_storage_is_clean(current, sizeof(header)))
    nrf5x_persist_storage_erase(current, size);

  // Swap slots
  pv->current_slot = !pv->current_slot;

  return 0;
}

static void nrf5x_persist_slot_state_read(struct nrf5x_nvmc_private_s *pv,
                                          struct nrf5x_persist_slot_state_s *state,
                                          uint8_t slot)
{
  uintptr_t base = pv->addr + pv->slot_size * slot;
  uintptr_t end = base + pv->slot_size;
  const struct nrf5x_persist_header_s *header = (const void *)base;
  uintptr_t point = base + sizeof(*header);

  state->available = 0;
  state->used = sizeof(*header);
  state->reclaimable = 0;

  dhexdumpk(base, header, sizeof(*header));
  if (header->magic != NRF5X_PERSIST_MAGIC
      || header->state != DEV_PERSIST_STATE_WRITTEN
      || header->size != pv->slot_size) {
    state->state = SLOT_BROKEN;
    dprintk("slot %d magic %08x size %d state %d\n", slot, header->magic, header->size, header->state);
    dprintk("slot %d bad header\n", slot);
    return;
  }

  state->state = SLOT_CLEAN;

  while (point < end) {
    const struct dev_persist_descriptor_s *desc = (const void *)point;

    switch (desc->state) {
    case DEV_PERSIST_STATE_FREE:
      if (nrf5x_persist_storage_is_clean(point, end - point)) {
        dprintk("slot %d clean end reached used %d\n", slot, state->used);
        state->available = pv->slot_size - sizeof(*header) - state->used;
        return;
      }

      dprintk("slot %d unclean end reached\n", slot);
      return;

    case DEV_PERSIST_STATE_BUSY:
      state->state = SLOT_MUST_PACK;
    case DEV_PERSIST_STATE_ERASED:
      state->reclaimable += item_size(desc);
      state->used += item_size(desc);
      break;

    case DEV_PERSIST_STATE_WRITTEN:
      state->used += item_size(desc);
      break;
    }

    point += item_size(desc);

    // We may have point == end if slot is full to the last byte
    if (point < base + sizeof(*header) || point > end) {
      state->state = SLOT_BROKEN;
      dprintk("slot %d overflow\n", slot);
      break;
    }
  }

  return;
}

static void nrf5x_persist_discover(struct nrf5x_nvmc_private_s *pv)
{
  struct nrf5x_persist_slot_state_s state[2];

  for (uint8_t i = 0; i < 2; ++i) {
    nrf5x_persist_slot_state_read(pv, &state[i], i);

    if (state[i].state == SLOT_CLEAN) {
      dprintk("slot %d clean\n", i);

      pv->current_slot = i;
      pv->used = state[i].used;
      pv->reclaimable = state[i].reclaimable;
      pv->available = state[i].available;
      return;
    }
  }

  // None of them is CLEAN, but we may have one with MUST_PACK ?
  for (uint8_t i = 0; i < 2; ++i) {
    if (state[i].state == SLOT_MUST_PACK) {
      dprintk("slot %d must pack\n", i);

      pv->current_slot = i;
      nrf5x_persist_pack(pv, NULL);
      return;
    }
  }

  dprintk("all broken\n");
  // None is CLEAN, nor MUST_PACK, erase first one
  pv->current_slot = 0;
  nrf5x_persist_erase(pv, NULL);
}

static
const struct dev_persist_descriptor_s *
nrf5x_persist_find(struct nrf5x_nvmc_private_s *pv,
                   const struct dev_persist_descriptor_s *ref)
{
  size_t size = pv->slot_size;
  uintptr_t current = pv->addr + pv->slot_size * pv->current_slot;
  uintptr_t point = current + sizeof(struct nrf5x_persist_header_s);

  struct dev_persist_descriptor_s desc = *ref;

  dprintk("Find %d %x\n", desc.type, desc.uid);

  while (point > current && point < current + size) {
    const struct dev_persist_descriptor_s *cur = (const void *)point;

    if (point + item_size(cur) > current + size)
      break;

    point += item_size(cur);

    dprintk(" %p %d %x %d size: %d (%d)...", cur, cur->type, cur->uid, cur->state,
           cur->size, item_size(cur));

    if (cur->state != DEV_PERSIST_STATE_WRITTEN) {
      dprintk(" bad state\n");
      continue;
    }

    if (cur->uid != desc.uid) {
      dprintk(" bad uid\n");
      continue;
    }

    if (cur->type != desc.type) {
      dprintk(" bad type\n");
      continue;
    }

    dprintk(" OK\n");

    return cur;
  }

  dprintk(" %p Not found\n", point);

  return NULL;
}

static error_t nrf5x_persist_read(struct nrf5x_nvmc_private_s *pv,
                               struct dev_persist_rq_s *rq)
{
  struct dev_persist_descriptor_s ref = *rq->descriptor;
  ref.uid += rq->uid_offset;

  const struct dev_persist_descriptor_s *found = nrf5x_persist_find(pv, &ref);

  if (!found)
    return -ENOENT;

  switch (found->type) {
  case DEV_PERSIST_BLOB:
    rq->data = (const void *)(found + 1);
    break;

  case DEV_PERSIST_COUNTER:
    rq->counter = nrf5x_persist_counter_base_get(found)
      + nrf5x_persist_counter_offset_get(found);
    break;
  }

  return 0;
}

static void nrf5x_persist_counter_zero_range(const struct dev_persist_descriptor_s *hdr,
                                             uint32_t first,
                                             uint32_t last)
{
  uintptr_t bits = (uintptr_t)(hdr + 1) + sizeof(uint64_t);

  uint32_t first_word = first / 32;
  uint32_t last_word = last / 32;
  uint32_t first_bit = first % 32;
  uint32_t last_bit = last % 32;
  uint32_t first_mask = ~((1 << first_bit) - 1);
  uint32_t last_mask = (1 << last_bit) - 1;
  uint32_t mask = first_mask;

  dprintk("zero range %d - %d\n", first_bit, last_bit);

  dprintk("first: %d@%d %08x\n", first_bit, first_word, first_mask);
  dprintk("last: %d@%d %08x\n", last_bit, last_word, last_mask);

  if (first_word == last_word)
    last_mask &= first_mask;
  else {
    uint32_t value = ~mask;

    for (uint32_t point = first_word; point < last_word; ++point) {
      nrf5x_persist_storage_write(bits + point * sizeof(uint32_t), &value, sizeof(value));
      value = 0;
    }
  }

  last_mask = ~last_mask;

  dprintk("last write: @%08x %08x\n", bits + last_word * sizeof(uint32_t), last_mask);

  nrf5x_persist_storage_write(bits + last_word * sizeof(uint32_t), &last_mask, sizeof(last_mask));
}

static error_t nrf5x_persist_erase(struct nrf5x_nvmc_private_s *pv,
                                   struct dev_persist_rq_s *rq)
{
  size_t size = pv->slot_size;
  uintptr_t current = pv->addr;
  struct nrf5x_persist_header_s header;

  if (!nrf5x_persist_storage_is_clean(current, size * 2))
    nrf5x_persist_storage_erase(current, size * 2);

  header.magic = NRF5X_PERSIST_MAGIC;
  header.size = size;
  header.state = DEV_PERSIST_STATE_BUSY;
  nrf5x_persist_storage_write(current, &header, sizeof(header));

  header.state = DEV_PERSIST_STATE_WRITTEN;
  nrf5x_persist_storage_write(current, &header, sizeof(header));

  pv->current_slot = 0;
  pv->used = sizeof(header);
  pv->reclaimable = 0;
  pv->available = pv->slot_size - sizeof(header);

  return 0;
}

static error_t nrf5x_persist_write(struct nrf5x_nvmc_private_s *pv,
                                   struct dev_persist_rq_s *rq)
{
  assert(rq);

  struct dev_persist_descriptor_s ref = *rq->descriptor;
  ref.uid += rq->uid_offset;

  dprintk("%s %d %x %d\n", __FUNCTION__, ref.type, ref.uid, ref.size);

  const struct dev_persist_descriptor_s *found = nrf5x_persist_find(pv, &ref);
  dprintk("Value found: %p\n", found);

  if (!found)
    goto append;

  switch (found->type) {
  case DEV_PERSIST_BLOB:
    if (!memcmp(found + 1, rq->data, found->size))
      return 0;
    break;

  case DEV_PERSIST_COUNTER: {
    if (rq->counter == 0)
      return 0;

    uint32_t bits = (item_size(found) - sizeof(*found) - sizeof(uint64_t)) * 8;
    uint32_t zeroes = nrf5x_persist_counter_offset_get(found);

    dprintk("%s counter %lld + %d/%d + %lld\n",
           __FUNCTION__, nrf5x_persist_counter_base_get(found),
           zeroes, bits, rq->counter);

    if (zeroes + rq->counter < bits) {
      nrf5x_persist_counter_zero_range(found, zeroes, zeroes + rq->counter);
      return 0;
    }

    rq->counter += nrf5x_persist_counter_base_get(found) + zeroes;
    break;
  }
  }

 append:;
  size_t needed = item_size(&ref);
  uintptr_t writep = pv->addr
    + pv->slot_size * pv->current_slot
    + pv->used;

  dprintk("Append %d %x. needed: %d, reclaimable: %d, available: %d, used: %d\n",
         ref.type, ref.uid,
         needed, pv->reclaimable, pv->available, pv->used);

  if (needed > pv->available + pv->reclaimable)
    return -ENOSPC;

  if (needed > pv->available) {
    nrf5x_persist_pack(pv, NULL);
    found = nrf5x_persist_find(pv, &ref);
    dprintk("Value found after repack: %p\n", found);
    writep = pv->addr
      + pv->slot_size * pv->current_slot
      + pv->used;
  }

  switch (ref.type) {
  case DEV_PERSIST_BLOB:
    nrf5x_persist_write_blob(pv, writep, &ref, rq->data, found);
    break;

  case DEV_PERSIST_COUNTER:
    nrf5x_persist_write_counter(pv, writep, &ref, rq->counter, found);
    break;
  }

  return 0;
}

static error_t nrf5x_persist_remove(struct nrf5x_nvmc_private_s *pv,
                                    struct dev_persist_rq_s *rq)
{
  assert(rq);

  struct dev_persist_descriptor_s ref = *rq->descriptor;
  ref.uid += rq->uid_offset;

  const struct dev_persist_descriptor_s *found = nrf5x_persist_find(pv, &ref);

  if (!found)
    return -ENOENT;

  nrf5x_persist_item_invalidate(pv, found);

  return 0;
}

static void nrf5x_persist_process(struct device_s *dev, struct dev_persist_rq_s *rq)
{
  struct nrf5x_nvmc_private_s *pv = dev->drv_pv;
  typedef error_t rq_starter(struct nrf5x_nvmc_private_s *pv,
                             struct dev_persist_rq_s *rq);
  static rq_starter *const starter[] = {
    [DEV_PERSIST_ERASE] = nrf5x_persist_erase,
    [DEV_PERSIST_PACK] = nrf5x_persist_pack,
    [DEV_PERSIST_READ] = nrf5x_persist_read,
    [DEV_PERSIST_WRITE] = nrf5x_persist_write,
    [DEV_PERSIST_REMOVE] = nrf5x_persist_remove,
  };

  rq->err = starter[rq->op](pv, rq);
}

static DEV_PERSIST_REQUEST(nrf5x_persist_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_nvmc_private_s *pv = dev->drv_pv;

  dev_request_delayed_push(device_persist_s_base(accessor),
                           &pv->queue, dev_persist_rq_s_base(rq), 1);
}

static DEV_PERSIST_INFO(nrf5x_persist_info)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_nvmc_private_s *pv = dev->drv_pv;

  info->storage_size = pv->slot_size;
  info->used_size = pv->used;
}
#endif

#ifdef CONFIG_DRIVER_NRF5X_NVMC

static DEV_MEM_INFO(nrf5x_mem_info)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_nvmc_private_s *pv = dev->drv_pv;

  if (band_index > 0)
    return -ENOENT;

  memset(info, 0, sizeof(*info));

  uint8_t page_log2 = __builtin_ctz(cpu_mem_read_32(NRF_FICR_CODEPAGESIZE));

  switch (accessor->number)
    {
    case BANK_CODE:
      info->size = cpu_mem_read_32(NRF_FICR_CODESIZE) - (pv->slot_size >> (page_log2 - 1));
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
        | DEV_MEM_WRITABLE
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

static void nrf5x_nvmc_flash_op(
  uintptr_t base,
  struct dev_mem_rq_s *rq)
{
  size_t i;
  uint32_t addr_mask = cpu_mem_read_32(NRF_FICR_CODEPAGESIZE) - 1;
  uint32_t page_log2 = __builtin_ctz(cpu_mem_read_32(NRF_FICR_CODEPAGESIZE));

  if (rq->type & (DEV_MEM_OP_PARTIAL_READ | DEV_MEM_OP_PAGE_READ))
    dev_mem_mapped_op_helper(base, page_log2, rq);

  if (rq->type & DEV_MEM_OP_PAGE_ERASE)
    for (i = 0; i < rq->size; i++)
      nrf5x_flash_page_erase(base + (~addr_mask & rq->addr) + (i << page_log2));

  if (rq->type & DEV_MEM_OP_PAGE_WRITE)
    {
      uintptr_t sc_mask = (1 << rq->sc_log2) - 1;
      uint32_t page_word_count = cpu_mem_read_32(NRF_FICR_CODEPAGESIZE) / 4;


      for (i = 0; i < rq->size; i++)
        nrf5x_flash_write(base + (addr_mask & rq->addr) + (i << page_log2),
                          (void*)(rq->sc_data[i >> rq->sc_log2]
                                  + ((i & sc_mask) << page_log2)),
                          page_word_count);
    }
  else if (rq->type & DEV_MEM_OP_PARTIAL_WRITE)
    nrf5x_flash_write(base + rq->addr, rq->data, rq->size / 4);
}

static void nrf5x_mem_process(struct device_accessor_s *accessor, struct dev_mem_rq_s *rq)
{
  rq->err = 0;
  switch (accessor->number) {
  case BANK_CODE:
    nrf5x_nvmc_flash_op(0x00000000, rq);
    break;

  case BANK_UICR:
    nrf5x_nvmc_flash_op(NRF_UICR_BASE, rq);
    break;

  default:
    rq->err = -EINVAL;
  }
}

static DEV_MEM_REQUEST(nrf5x_mem_request)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_nvmc_private_s *pv = dev->drv_pv;

  dev_request_delayed_push(device_mem_s_base(accessor),
                           &pv->queue, dev_mem_rq_s_base(rq), 1);
}

#endif

#if defined(CONFIG_DRIVER_NRF5X_NVMC) || defined(CONFIG_DRIVER_NRF5X_PERSIST)
static DEV_REQUEST_DELAYED_FUNC(nrf5x_nvmc_process)
{
  struct device_s *dev = accessor->dev;
  struct nrf5x_nvmc_private_s *pv = dev->drv_pv;

  switch ((__compiler_sint_t)accessor->api->class_) {
#ifdef CONFIG_DRIVER_NRF5X_NVMC
  case DRIVER_CLASS_MEM:
    nrf5x_mem_process(accessor, dev_mem_rq_s_cast(rq_));
    break;
#endif

#ifdef CONFIG_DRIVER_NRF5X_PERSIST
  case DRIVER_CLASS_PERSIST:
    nrf5x_persist_process(dev, dev_persist_rq_s_cast(rq_));
    break;
#endif
  }

  dev_request_delayed_end(&pv->queue, rq_);
}

static DEV_INIT(nrf5x_nvmc_init);
static DEV_CLEANUP(nrf5x_nvmc_cleanup);
static DEV_USE(nrf5x_nvmc_use);

DRIVER_DECLARE(nrf5x_nvmc_drv, 0, "nRF5x NVMC"
#ifdef CONFIG_DRIVER_NRF5X_NVMC
               ",MEM"
#endif
#ifdef CONFIG_DRIVER_NRF5X_PERSIST
               ",PERSIST"
#endif
               , nrf5x_nvmc
#ifdef CONFIG_DRIVER_NRF5X_NVMC
               , DRIVER_MEM_METHODS(nrf5x_mem)
#endif
#ifdef CONFIG_DRIVER_NRF5X_PERSIST
               , DRIVER_PERSIST_METHODS(nrf5x_persist)
#endif
               );

DRIVER_REGISTER(nrf5x_nvmc_drv);

static DEV_USE(nrf5x_nvmc_use)
{
  struct device_accessor_s *accessor = param;
  
  if (op == DEV_USE_LAST_NUMBER) {
    accessor->number = BANK_UICR;
    return 0;
  }

  return dev_use_generic(param, op);
}

static DEV_INIT(nrf5x_nvmc_init)
{
  struct nrf5x_nvmc_private_s *pv;
  error_t err = 0;
  uintptr_t addr;

  assert(!device_res_get_uint(dev, DEV_RES_MEM, 0, &addr, NULL)
         && NVMC_ADDR == addr);

#ifdef CONFIG_DRIVER_NRF5X_PERSIST
  uint32_t page_size = cpu_mem_read_32(NRF_FICR_CODEPAGESIZE);
  uintptr_t size;

  pv = mem_alloc(sizeof(*pv), mem_scope_sys);
  if (!pv)
    return -ENOMEM;

  memset(pv, 0, sizeof(*pv));

  err = device_res_get_uint(dev, DEV_RES_MEM, 1, &addr, &size);
  if (err)
    goto err_pv;

  printk("NVMC persist %p %p\n", addr, size);

  // Keep truncated down size for a multiple of 2 pages.
  pv->slot_size = (size / 2) & (~(page_size - 1));
  pv->addr = addr;

  printk("NVMC with persist driver on slots of %d bytes, pages of %d bytes, from %p and up\n",
         pv->slot_size, page_size, pv->addr);

  if (pv->slot_size < page_size) {
    err = -ENOTSUP;
    goto err_pv;
  }

  dev_request_delayed_init(&pv->queue, nrf5x_nvmc_process);

  nrf5x_persist_discover(pv);
#endif

  dev->drv_pv = pv;
  return 0;

#ifdef CONFIG_DRIVER_NRF5X_PERSIST
 err_pv:
  mem_free(pv);
  return err;
#endif
}

static DEV_CLEANUP(nrf5x_nvmc_cleanup)
{
#ifdef CONFIG_DRIVER_NRF5X_PERSIST
  struct nrf5x_nvmc_private_s *pv = dev->drv_pv;

  if (!dev_request_delayed_isidle(&pv->queue))
    return -EBUSY;

  dev_request_delayed_cleanup(&pv->queue);
  mem_free(pv);
#endif

  return 0;
}
#endif
