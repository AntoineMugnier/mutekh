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
*/

/**
   @file
   @module {Libraries::Persistent configuration service}
   @short Persistent configuration library
*/

#ifndef __PERSIST_H__
#define __PERSIST_H__

#include <enums.h>

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/lock.h>

#include <mutek/kroutine.h>

/** backend choice */
#ifdef CONFIG_PERSIST_RAM_BACKEND
#define PERSIST_FLASH_ERASE_VALUE 0xff
#ifndef CONFIG_PERSIST_RAM_BACKEND_PAGESIZE
#define CONFIG_PERSIST_RAM_BACKEND_PAGESIZE 0x1000
#endif
#endif

/** @internal */
enum persist_state_e
{
  PERSIST_STATE_ERASED = 0,
  PERSIST_STATE_WRITTEN = 1,
  PERSIST_STATE_BUSY = 3,
  PERSIST_STATE_FREE = 7,
};

ENUM_DESCRIPTOR(persist_type_e, strip:PERSIST_, upper);

/**  Persist config */
struct persist_config
{
  uintptr_t dev_addr;
  size_t dev_size;              /* must be at least 2 * page_size */
  size_t page_size;
};

/** Persistent storage context */
struct persist_context_s
{
  /* backed device info */
  uintptr_t addr;
  size_t page_size;
  size_t slot_size;

  /* current state */
  uint8_t current_slot;

  size_t used;
  size_t reclaimable;
  size_t available;
};

/** Value type */
enum persist_type_e
{
  /** A blob may hold any value. It is reallocated when written to. */
  PERSIST_BLOB,

  /** A counter can only be incremented or reset. It is stored in two
      fields in backing storage: a counter part, and a bitfield part.
      Single increments changes single bits in the bitfield part as
      long as there are some left.  When all bytes got consumed, value
      is reallocated.  This minimizes wear on backing storage.

      Writes to counters actually increment counter with passed value.
  */
  PERSIST_COUNTER,
};

/** Value descriptor. */
struct persist_descriptor_s
{
  /** Platform-wide identifier. @ref persist_rq_s::uid_offset is added
      to this id when a requested is performed. */
  uint16_t uid;
  /** Type */
  uint16_t type:1;
  /** Reseved bits. @internal */
  uint16_t state:3;
  /** Byte size of blobs, byte size of bitmap for counters. */
  uint16_t size:12;
} __attribute__((packed));

/** Persist request object. */
struct persist_rq_s
{
  /** Executed when request done */
  struct kroutine_s kr;

  /** Reserved for application */
  void *pvdata;

  /** Value descriptor */
  const struct persist_descriptor_s *descriptor;

  /** UID offset to add to descriptor's base UID */
  uint16_t uid_offset;

  /** Access return status */
  error_t error;

  union {
    /**
       @list
       @item for blob write accesses, this holds buffer to copy to persist
             storage.
       @item for blob read accesses, this will be set to an address
             that points to actual copy of data.
       @end list
    */
    const void *data;

    /**
       @list
       @item for counter write accesses, this holds an offset to add
             to counter.  Most write operations will take @tt 1 here.

       @item for counter read accesses, this will contain actual counter
             value upon successful operation.
       @end list
     */
    uint64_t counter;
  };
};

STRUCT_COMPOSE(persist_rq_s, kr);

/** Initialize a context for access to a persistent storage in a
    memory area. */
void persist_context_init(struct persist_context_s *ctx,
                          uintptr_t dev_addr, size_t dev_size,
                          size_t page_size);

/** Read an entry. Caller will get data pointer set to start of value
    if present. */
config_depend(CONFIG_PERSIST)
void persist_read(struct persist_context_s *ctx,
                  struct persist_rq_s *rq);

/** Write an entry. Entry will be replaced if present already.  This
    may pack existing storage. */
config_depend(CONFIG_PERSIST)
void persist_write(struct persist_context_s *ctx,
                   struct persist_rq_s *rq);

/** Erase all entries, start fresh with a blank storage.  This does
    not check for previous storage contents. */
config_depend(CONFIG_PERSIST)
void persist_erase(struct persist_context_s *ctx,
                   struct persist_rq_s *rq);

/** Remove a given entry, if present. */
config_depend(CONFIG_PERSIST)
void persist_remove(struct persist_context_s *ctx,
                    struct persist_rq_s *rq);

/** Pack storage. Collect all erased entries. */
config_depend(CONFIG_PERSIST)
void persist_pack(struct persist_context_s *ctx,
                  struct persist_rq_s *rq);

#ifdef CONFIG_MUTEK_CONTEXT_SCHED
struct sched_context_s;

/** @internal */
struct persist_status_s
{
  lock_t lock;
  struct sched_context_s *ctx;
  bool_t done;
};

/** @internal */
void persist_sched_init(struct persist_rq_s *rq,
                        struct persist_status_s *status);

/** @internal */
void persist_sched_wait(struct persist_status_s *status);

#endif

/** Synchronous persist device operation function. This function use
    the scheduler api to put current context in wait state during the
    request. */
config_depend_and2_inline(CONFIG_PERSIST, CONFIG_MUTEK_CONTEXT_SCHED,
inline error_t
persist_wait_read(struct persist_context_s *ctx,
                  const struct persist_descriptor_s *desc,
                  uint16_t uid_offset,
                  const void **data),
{
  struct persist_rq_s rq = {
    .descriptor = desc,
    .uid_offset = uid_offset,
  };

  struct persist_status_s st;
  persist_sched_init(&rq, &st);
  persist_read(ctx, &rq);
  persist_sched_wait(&st);

  *data = rq.data;
  return rq.error;
});

/** Synchronous persist device operation function. This function use
    the scheduler api to put current context in wait state during the
    request. */
config_depend_and2_inline(CONFIG_PERSIST, CONFIG_MUTEK_CONTEXT_SCHED,
inline error_t
persist_wait_write(struct persist_context_s *ctx,
                   const struct persist_descriptor_s *desc,
                   uint16_t uid_offset,
                   const void *data),
{
  struct persist_rq_s rq = {
    .descriptor = desc,
    .uid_offset = uid_offset,
    .data = data,
  };

  struct persist_status_s st;
  persist_sched_init(&rq, &st);
  persist_write(ctx, &rq);
  persist_sched_wait(&st);

  return rq.error;
});

/** Synchronous persist device operation function. This function use
    the scheduler api to put current context in wait state during the
    request. */
config_depend_and2_inline(CONFIG_PERSIST, CONFIG_MUTEK_CONTEXT_SCHED,
inline error_t
persist_wait_remove(struct persist_context_s *ctx,
                    const struct persist_descriptor_s *desc,
                    uint16_t uid_offset),
{
  struct persist_rq_s rq = {
    .descriptor = desc,
    .uid_offset = uid_offset,
  };

  struct persist_status_s st;
  persist_sched_init(&rq, &st);
  persist_remove(ctx, &rq);
  persist_sched_wait(&st);

  return rq.error;
});

/** Synchronous persist device operation function. This function use
    the scheduler api to put current context in wait state during the
    request. */
config_depend_and2_inline(CONFIG_PERSIST, CONFIG_MUTEK_CONTEXT_SCHED,
inline error_t
persist_wait_pack(struct persist_context_s *ctx),
{
  struct persist_rq_s rq = {
  };

  struct persist_status_s st;
  persist_sched_init(&rq, &st);
  persist_pack(ctx, &rq);
  persist_sched_wait(&st);

  return rq.error;
 });

/** Synchronous persist device operation function. This function use
    the scheduler api to put current context in wait state during the
    request. */
config_depend_and2_inline(CONFIG_PERSIST, CONFIG_MUTEK_CONTEXT_SCHED,
inline error_t
persist_wait_erase(struct persist_context_s *ctx),
{
  struct persist_rq_s rq = {
  };

  struct persist_status_s st;
  persist_sched_init(&rq, &st);
  persist_erase(ctx, &rq);
  persist_sched_wait(&st);

  return rq.error;
 });

/** Synchronous persist device operation function. This function use
    the scheduler api to put current context in wait state during the
    request. The value of @tt counter must contain the increment and
    is replaced by the value of the counter after the operation. */
config_depend_and2_inline(CONFIG_PERSIST, CONFIG_MUTEK_CONTEXT_SCHED,
inline error_t
persist_wait_inc(struct persist_context_s *ctx,
                 const struct persist_descriptor_s *desc,
                 uint16_t uid_offset, uint64_t *counter),
{
  struct persist_rq_s rq = {
    .descriptor = desc,
    .uid_offset = uid_offset,
    .counter = *counter,
  };

  struct persist_status_s st;
  persist_sched_init(&rq, &st);
  persist_write(ctx, &rq);
  persist_sched_wait(&st);

  *counter = rq.counter;

  return rq.error;
});

/** Synchronous persist device operation function. This function use
    the scheduler api to put current context in wait state during the
    request. */
config_depend_and2_inline(CONFIG_PERSIST, CONFIG_MUTEK_CONTEXT_SCHED,
inline error_t
persist_wait_counter_read(struct persist_context_s *ctx,
                          const struct persist_descriptor_s *desc,
                          uint16_t uid_offset,
                          uint64_t *value),
{
  struct persist_rq_s rq = {
    .descriptor = desc,
    .uid_offset = uid_offset,
    .counter = 1,
  };

  struct persist_status_s st;
  persist_sched_init(&rq, &st);
  persist_read(ctx, &rq);
  persist_sched_wait(&st);

  *value = rq.counter;
  return rq.error;
});

#endif
