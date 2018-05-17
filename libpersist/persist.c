#define LOGK_MODULE_ID "prst"
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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2015
    Copyright Alexandre Becoulet <alexandre.becoulet@free.fr> (c) 2018
*/

#include <persist/persist.h>
#include <hexo/flash.h>
#include <mutek/printk.h>
#include <mutek/kroutine.h>

#define PERSIST_MAGIC 0x58a1fb3e

enum slot_state_e
{
  SLOT_CLEAN,
  SLOT_EMPTY,
  SLOT_MUST_PACK,
  SLOT_BROKEN,
};

struct persist_header_s
{
  uint32_t magic;
  uint32_t size: 29;
  uint32_t state: 3;
};

struct persist_slot_state_s
{
  enum slot_state_e state;
  size_t used;
  size_t reclaimable;
  size_t available;
};

static error_t persist_erase_process(struct persist_context_s *ctx);

static bool_t persist_storage_is_clean(uintptr_t base, size_t size)
{
  assert(address_is_aligned(base, 4) &&
         address_is_aligned(size, 4));

  const uint32_t *begin = (const uint32_t *)base;
  const uint32_t *end = (const uint32_t *)(base + size);

  for (; begin < end; begin++)
    if (*begin != 0xffffffff)
      return 0;

  return 1;
}

static void persist_storage_erase(struct persist_context_s *ctx,
                                  uintptr_t base, size_t size)
{
  uint32_t page_size = ctx->page_size;

  assert(address_is_aligned(base, page_size) &&
         address_is_aligned(size, page_size));

  while (size) {
    flash_page_erase(base);
    base += page_size;
    size -= page_size;
  }
}

static void persist_storage_write(uintptr_t base,
                                  const void *data,
                                  size_t size)
{
  assert(address_is_aligned(base, 4) &&
         address_is_aligned(data, 4) &&
         address_is_aligned(size, 4));

  flash_page_write(base, data, size);
}

static inline size_t item_size(const struct persist_descriptor_s *desc)
{
  return sizeof(*desc) + align_pow2_up(desc->size, 4);
}

static void persist_item_invalidate(struct persist_context_s *ctx,
                                    const struct persist_descriptor_s *hdr)
{
  struct persist_descriptor_s desc = *hdr;
  desc.state = PERSIST_STATE_ERASED;

  logk_trace("%s %d %x %d at %p\n", __FUNCTION__, desc.type, desc.uid, desc.size, hdr);

  persist_storage_write((uintptr_t)hdr, &desc, sizeof(desc));

  ctx->reclaimable += item_size(&desc);
}

static void persist_write_blob(struct persist_context_s *ctx,
                               uintptr_t writep,
                               struct persist_descriptor_s *desc,
                               const void *data,
                               const struct persist_descriptor_s *to_invalidate)
{
  uint32_t size = item_size(desc);

  logk_trace("%s %d %x %d at %08x\n", __FUNCTION__, desc->type, desc->uid, desc->size, writep);

  desc->state = PERSIST_STATE_BUSY;
  persist_storage_write(writep, desc, sizeof(*desc));

  persist_storage_write(writep + sizeof(*desc), data, desc->size);

  if (to_invalidate)
    persist_item_invalidate(ctx, to_invalidate);

  desc->state = PERSIST_STATE_WRITTEN;
  persist_storage_write(writep, desc, sizeof(*desc));

  ctx->used += size;
  ctx->available -= size;
}

static void persist_write_counter(struct persist_context_s *ctx,
                                  uintptr_t writep,
                                  struct persist_descriptor_s *desc,
                                  uint64_t value,
                                  const struct persist_descriptor_s *to_invalidate)
{
  uint32_t size = item_size(desc);

  assert(desc->size > sizeof(value));

  logk_trace("%s %d %x %d at %08x\n", __FUNCTION__, desc->type, desc->uid, desc->size, writep);

  desc->state = PERSIST_STATE_BUSY;
  persist_storage_write(writep, desc, sizeof(*desc));

  persist_storage_write(writep + sizeof(*desc), &value, sizeof(value));

  if (to_invalidate)
    persist_item_invalidate(ctx, to_invalidate);

  desc->state = PERSIST_STATE_WRITTEN;
  persist_storage_write(writep, desc, sizeof(*desc));

  ctx->used += size;
  ctx->available -= size;
}

static uint64_t persist_counter_base_get(const struct persist_descriptor_s *desc)
{
  const uint64_t *valuep = (const void *)(desc + 1);
  return *valuep;
}

static uint32_t persist_counter_offset_get(const struct persist_descriptor_s *desc)
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
    zeroes = bit_ctz(begin[low]);
  return zeroes + low * sizeof(uint32_t) * 8;
}

static error_t persist_pack_process(struct persist_context_s *ctx)
{
  size_t size = ctx->slot_size;
  uintptr_t current = ctx->addr;
  uintptr_t other = ctx->addr;
  uintptr_t writep = other;
  struct persist_header_s header;

  if (!ctx->reclaimable)
    return 0;

  if (ctx->current_slot == 0)
    other += size;
  else
    current += size;

  if (!persist_storage_is_clean(other, size))
    persist_storage_erase(ctx, other, size);

  header.magic = PERSIST_MAGIC;
  header.size = size;
  header.state = PERSIST_STATE_BUSY;
  persist_storage_write(other, &header, sizeof(header));

  writep = sizeof(header) + other;

  ctx->used = sizeof(header);
  ctx->available = ctx->slot_size - sizeof(header);
  ctx->reclaimable = 0;

  uintptr_t point = current + sizeof(struct persist_header_s);

  while (point > current && point < current + size) {
    const struct persist_descriptor_s *curp = (const void*)point;
    struct persist_descriptor_s cur = *curp;

    if (point + item_size(curp) > current + size)
      break;

    point += item_size(curp);

    if (curp->state != PERSIST_STATE_WRITTEN)
      continue;

    switch (cur.type) {
    case PERSIST_BLOB:
      persist_write_blob(ctx, writep, &cur, curp + 1, NULL);
      break;

    case PERSIST_COUNTER:
      persist_write_counter(ctx, writep, &cur,
                                  persist_counter_base_get(curp)
                                  + persist_counter_offset_get(curp),
                                  NULL);
      break;
    }

    writep += item_size(&cur);
  }

  header.state = PERSIST_STATE_WRITTEN;
  persist_storage_write(other, &header, sizeof(header));

  if (!persist_storage_is_clean(current, sizeof(header)))
    persist_storage_erase(ctx, current, size);

  // Swap slots
  ctx->current_slot = !ctx->current_slot;

  return 0;
}

void persist_pack(struct persist_context_s *ctx,
                  struct persist_rq_s *rq)
{
  persist_pack_process(ctx);
  rq->err = 0;
  kroutine_exec(&rq->kr);
}

static void persist_slot_state_read(struct persist_context_s *ctx,
                                    struct persist_slot_state_s *state,
                                    uint8_t slot)
{
  uintptr_t base = ctx->addr + ctx->slot_size * slot;
  uintptr_t end = base + ctx->slot_size;
  const struct persist_header_s *header = (const void *)base;
  uintptr_t point = base + sizeof(*header);

  state->available = 0;
  state->used = sizeof(*header);
  state->reclaimable = 0;

  if (header->magic != PERSIST_MAGIC
      || header->state != PERSIST_STATE_WRITTEN
      || header->size != ctx->slot_size) {
    state->state = SLOT_BROKEN;
    logk_debug("slot %d magic %08x size %d state %d\n", slot, header->magic, header->size, header->state);
    logk_debug("slot %d bad header\n", slot);
    return;
  }

  state->state = SLOT_CLEAN;

  while (point < end) {
    const struct persist_descriptor_s *desc = (const void *)point;

    switch (desc->state) {
    case PERSIST_STATE_FREE:
      if (persist_storage_is_clean(point, end - point)) {
        logk_trace("slot %d clean end reached used %d\n", slot, state->used);
        state->available = ctx->slot_size - sizeof(*header) - state->used;
        return;
      }

      logk_trace("slot %d unclean end reached\n", slot);
      return;

    case PERSIST_STATE_BUSY:
      state->state = SLOT_MUST_PACK;
    case PERSIST_STATE_ERASED:
      state->reclaimable += item_size(desc);
      state->used += item_size(desc);
      break;

    case PERSIST_STATE_WRITTEN:
      state->used += item_size(desc);
      break;
    }

    point += item_size(desc);

    // We may have point == end if slot is full to the last byte
    if (point < base + sizeof(*header) || point > end) {
      state->state = SLOT_BROKEN;
      logk_trace("slot %d overflow\n", slot);
      break;
    }
  }

  return;
}

static void persist_discover(struct persist_context_s *ctx)
{
  struct persist_slot_state_s state[2];

  for (uint8_t i = 0; i < 2; ++i) {
    persist_slot_state_read(ctx, &state[i], i);

    if (state[i].state == SLOT_CLEAN) {
      logk_trace("slot %d clean\n", i);

      ctx->current_slot = i;
      ctx->used = state[i].used;
      ctx->reclaimable = state[i].reclaimable;
      ctx->available = state[i].available;
      return;
    }
  }

  // None of them is CLEAN, but we may have one with MUST_PACK ?
  for (uint8_t i = 0; i < 2; ++i) {
    if (state[i].state == SLOT_MUST_PACK) {
      logk_trace("slot %d must pack\n", i);

      ctx->current_slot = i;
      persist_pack_process(ctx);
      return;
    }
  }

  logk_trace("all broken\n");
  // None is CLEAN, nor MUST_PACK, erase first one
  ctx->current_slot = 0;
  persist_erase_process(ctx);
}

static const struct persist_descriptor_s *
persist_find(struct persist_context_s *ctx,
             const struct persist_descriptor_s *ref)
{
  size_t size = ctx->slot_size;
  uintptr_t current = ctx->addr + ctx->slot_size * ctx->current_slot;
  uintptr_t point = current + sizeof(struct persist_header_s);

  struct persist_descriptor_s desc = *ref;

  logk_trace("Find %d %x\n", desc.type, desc.uid);

  while (point > current && point < current + size) {
    const struct persist_descriptor_s *cur = (const void *)point;

    if (point + item_size(cur) > current + size)
      break;

    point += item_size(cur);

    logk_trace(" %p %d %x %d size: %d (%d)...", cur, cur->type, cur->uid, cur->state,
           cur->size, item_size(cur));

    if (cur->state != PERSIST_STATE_WRITTEN) {
      logk_trace(" bad state\n");
      continue;
    }

    if (cur->uid != desc.uid) {
      logk_trace(" bad uid\n");
      continue;
    }

    if (cur->type != desc.type) {
      logk_trace(" bad type\n");
      continue;
    }

    logk_trace(" OK\n");

    return cur;
  }

  logk_trace(" %p Not found\n", point);

  return NULL;
}

static error_t persist_read_process(struct persist_context_s *ctx,
                                    struct persist_rq_s *rq)
{
  struct persist_descriptor_s ref = *rq->descriptor;
  ref.uid += rq->uid_offset;

  const struct persist_descriptor_s *found = persist_find(ctx, &ref);

  if (!found)
    return -ENOENT;

  switch (found->type) {
  case PERSIST_BLOB:
    rq->data = (const void *)(found + 1);
    break;

  case PERSIST_COUNTER:
    rq->counter = persist_counter_base_get(found)
      + persist_counter_offset_get(found);
    break;
  }

  return 0;
}

void persist_read(struct persist_context_s *ctx,
                  struct persist_rq_s *rq)
{
  rq->err = persist_read_process(ctx, rq);
  kroutine_exec(&rq->kr);
}

static void persist_counter_zero_range(const struct persist_descriptor_s *hdr,
                                             uint32_t first,
                                             uint32_t last)
{
  uintptr_t bits = (uintptr_t)(hdr + 1) + sizeof(uint64_t);

  uint32_t first_word = first / 32;
  uint32_t last_word = last / 32;
  uint32_t first_bit = first % 32;
  uint32_t last_bit = last % 32;
  uint32_t first_mask = ~bit_mask(0, first_bit);
  uint32_t last_mask = bit_mask(0, last_bit);
  uint32_t mask = first_mask;

  logk_trace("zero range %d - %d\n", first_bit, last_bit);

  logk_trace("first: %d@%d %08x\n", first_bit, first_word, first_mask);
  logk_trace("last: %d@%d %08x\n", last_bit, last_word, last_mask);

  if (first_word == last_word)
    last_mask &= first_mask;
  else {
    uint32_t value = ~mask;

    for (uint32_t point = first_word; point < last_word; ++point) {
      persist_storage_write(bits + point * sizeof(uint32_t), &value, sizeof(value));
      value = 0;
    }
  }

  last_mask = ~last_mask;

  logk_trace("last write: @%08x %08x\n", bits + last_word * sizeof(uint32_t), last_mask);

  persist_storage_write(bits + last_word * sizeof(uint32_t), &last_mask, sizeof(last_mask));
}

static error_t persist_erase_process(struct persist_context_s *ctx)
{
  size_t size = ctx->slot_size;
  uintptr_t current = ctx->addr;
  struct persist_header_s header;

  if (!persist_storage_is_clean(current, size * 2))
    persist_storage_erase(ctx, current, size * 2);

  header.magic = PERSIST_MAGIC;
  header.size = size;
  header.state = PERSIST_STATE_BUSY;
  persist_storage_write(current, &header, sizeof(header));

  header.state = PERSIST_STATE_WRITTEN;
  persist_storage_write(current, &header, sizeof(header));

  ctx->current_slot = 0;
  ctx->used = sizeof(header);
  ctx->reclaimable = 0;
  ctx->available = ctx->slot_size - sizeof(header);

  return 0;
}

void persist_erase(struct persist_context_s *ctx,
                   struct persist_rq_s *rq)
{
  rq->err = persist_erase_process(ctx);
  kroutine_exec(&rq->kr);
}

static error_t persist_write_process(struct persist_context_s *ctx,
                                     struct persist_rq_s *rq)
{
  struct persist_descriptor_s ref = *rq->descriptor;
  ref.uid += rq->uid_offset;

  logk_trace("%s %d %x %d\n", __FUNCTION__, ref.type, ref.uid, ref.size);

  const struct persist_descriptor_s *found = persist_find(ctx, &ref);
  logk_trace("Value found: %p\n", found);

  if (!found)
    goto append;

  switch (found->type) {
  case PERSIST_BLOB:
    if (!memcmp(found + 1, rq->data, found->size))
      return 0;
    break;

  case PERSIST_COUNTER: {
    if (rq->counter == 0)
      return 0;

    uint32_t bits = (item_size(found) - sizeof(*found) - sizeof(uint64_t)) * 8;
    uint32_t zeroes = persist_counter_offset_get(found);

    logk_trace("%s counter %lld + %d/%d + %lld\n",
           __FUNCTION__, persist_counter_base_get(found),
           zeroes, bits, rq->counter);

    if (zeroes + rq->counter < bits) {
      persist_counter_zero_range(found, zeroes, zeroes + rq->counter);
      return 0;
    }

    rq->counter += persist_counter_base_get(found) + zeroes;
    break;
  }
  }

 append:;
  size_t needed = item_size(&ref);
  uintptr_t writep = ctx->addr
    + ctx->slot_size * ctx->current_slot
    + ctx->used;

  logk_trace("Append %d %x. needed: %d, reclaimable: %d, available: %d, used: %d\n",
         ref.type, ref.uid,
         needed, ctx->reclaimable, ctx->available, ctx->used);

  if (needed > ctx->available + ctx->reclaimable)
    return -ENOSPC;

  if (needed > ctx->available) {
    persist_pack_process(ctx);
    found = persist_find(ctx, &ref);
    logk_trace("Value found after repack: %p\n", found);
    writep = ctx->addr
      + ctx->slot_size * ctx->current_slot
      + ctx->used;
  }

  switch (ref.type) {
  case PERSIST_BLOB:
    persist_write_blob(ctx, writep, &ref, rq->data, found);
    break;

  case PERSIST_COUNTER:
    persist_write_counter(ctx, writep, &ref, rq->counter, found);
    break;
  }

  return 0;
}

void persist_write(struct persist_context_s *ctx,
                   struct persist_rq_s *rq)
{
  rq->err = persist_write_process(ctx, rq);
  kroutine_exec(&rq->kr);
}

static error_t persist_remove_process(struct persist_context_s *ctx,
                                      struct persist_rq_s *rq)
{
  struct persist_descriptor_s ref = *rq->descriptor;
  ref.uid += rq->uid_offset;

  const struct persist_descriptor_s *found = persist_find(ctx, &ref);

  if (!found)
    return -ENOENT;

  persist_item_invalidate(ctx, found);

  return 0;
}

void persist_remove(struct persist_context_s *ctx,
                    struct persist_rq_s *rq)
{
  rq->err = persist_remove_process(ctx, rq);
  kroutine_exec(&rq->kr);
}

void persist_context_init(struct persist_context_s *ctx,
                          uintptr_t dev_addr, size_t dev_size,
                          size_t page_size)
{
  assert(dev_size >= (2 * page_size));

  ctx->addr = dev_addr;
  ctx->slot_size = (dev_size >> 1) & (~(page_size - 1));
  ctx->page_size = page_size;

  persist_discover(ctx);
}

#ifdef CONFIG_MUTEK_CONTEXT_SCHED

#include <mutek/scheduler.h>
#include <hexo/lock.h>

static KROUTINE_EXEC(persist_sched_done)
{
  struct persist_rq_s *rq = KROUTINE_CONTAINER(kr, *rq, kr);
  struct persist_status_s *status = rq->pvdata;

  LOCK_SPIN_IRQ(&status->lock);
  if (status->ctx != NULL)
    sched_context_start(status->ctx);
  status->done = 1;
  LOCK_RELEASE_IRQ(&status->lock);
}

void persist_sched_init(struct persist_rq_s *rq,
                        struct persist_status_s *status)
{
  status->done = 0;
  lock_init(&status->lock);
  status->ctx = NULL;
  rq->pvdata = status;
  kroutine_init_immediate(&rq->kr, &persist_sched_done);
}

void persist_sched_wait(struct persist_status_s *status)
{
  CPU_INTERRUPT_SAVESTATE_DISABLE;
  lock_spin(&status->lock);

  if (!status->done)
    {
      status->ctx = sched_get_current();
      sched_stop_unlock(&status->lock);
    }
  else
    lock_release(&status->lock);

  CPU_INTERRUPT_RESTORESTATE;

  lock_destroy(&status->lock);
}

extern inline error_t
persist_wait_read(struct persist_context_s *ctx,
                  const struct persist_descriptor_s *desc,
                  uint16_t uid_offset,
                  const void **data);

extern inline error_t
persist_wait_write(struct persist_context_s *ctx,
                   const struct persist_descriptor_s *desc,
                   uint16_t uid_offset,
                   const void *data);

extern inline error_t
persist_wait_remove(struct persist_context_s *ctx,
                    const struct persist_descriptor_s *desc,
                    uint16_t uid_offset);

extern inline error_t
persist_wait_inc(struct persist_context_s *ctx,
                 const struct persist_descriptor_s *desc,
                 uint16_t uid_offset);

extern inline error_t
persist_wait_counter_read(struct persist_context_s *ctx,
                          const struct persist_descriptor_s *desc,
                          uint16_t uid_offset,
                          uint64_t *value);

#endif
