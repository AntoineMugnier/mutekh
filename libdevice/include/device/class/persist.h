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
   @module {Core::Devices support library}
   @short Persistent configuration device driver
   @index {Persistent configuration device} {Device classes}
   @csee DRIVER_CLASS_PERSIST

   @section {Description}

   @tt persist driver class exists to offer a basic and simple
   configuration storage backend based on the usual request pattern.

   Operations involve:
   @list
   @item Read a value,
   @item Write a value,
   @item Delete a value,
   @item Pack storage backend (reclaim unused space),
   @item Erase storage.
   @end list

   @tt persist devices can hold up to 2^16 values, distinguished
   through their UID (unique identifier).

   Stored values must have a static declaration (@see
   dev_persist_descriptor_s).  Values may be of one of two types:
   @list
   @item Blobs, binary objects read/write verbatim;
   @item Counters, 64 bit values that can either be added to or erased.
   @end list

   In order to be able to share a declaration among various similar
   values, requests may set an @tt uid_offset that gets added to the
   descriptor's UID when looking up value in storage backend.

   Storage backend takes all possible measures to ensure value
   validity and durability.  In particular, special care is taken to
   ensure no data is lost in case of power outage during write phases.

   Size of blobs is known in advance and is part of their declaration.
   They cannot extend beyond their preallocated size.

   Size of counters must be at least 8 bytes, They make no interest
   under 9 bytes.  Any byte beyond 8-th serves as an optimization on
   increment operations: increments are done in-place using spare bits
   as an unary value that adds to the counter.  This makes sense on
   flash-based storage backends where a page-erase operation is
   required before writing new data.  Optimal size to allocate to a
   counter is a usage-dependant tradeof.

   @end section
 */

#ifndef __DEVICE_PERSIST_H__
#define __DEVICE_PERSIST_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <mutek/kroutine.h>

#include <device/driver.h>
#include <device/request.h>

#include <gct_platform.h>
#include <gct/container_clist.h>

struct device_s;
struct driver_s;
struct device_persist_s;
struct driver_persist_s;

/** @internal */
enum dev_persist_state_e
{
  DEV_PERSIST_STATE_ERASED = 0,
  DEV_PERSIST_STATE_WRITTEN = 1,
  DEV_PERSIST_STATE_BUSY = 3,
  DEV_PERSIST_STATE_FREE = 7,
};

ENUM_DESCRIPTOR(dev_persist_type_e, strip:DEV_PERSIST_, upper);

/**
   Value type
*/
enum dev_persist_type_e
{
  /** A blob may hold any value. It is reallocated when written to. */
  DEV_PERSIST_BLOB,

  /** A counter can only be incremented or reset. It is stored in two
      fields in backing storage: a counter part, and a bitfield part.
      Single increments changes single bits in the bitfield part as
      long as there are some left.  When all bytes got consumed, value
      is reallocated.  This minimizes wear on backing storage.

      Writes to counters actually increment counter with passed value.
  */
  DEV_PERSIST_COUNTER,
};


/**
   Value descriptor.
 */
struct dev_persist_descriptor_s
{
  /** Platform-wise identifier. @tt rq->uid_offset is added to this
      id. */
  uint16_t uid;
  /** Type */
  uint16_t type : 1;
  /** Reseved bits. @internal */
  uint16_t state : 3;
  /** Byte size for blobs, size of integer part for counters. */
  uint16_t size : 12;
} __attribute__((packed));

ENUM_DESCRIPTOR(dev_persist_op_e, strip:DEV_PERSIST_, upper);

/**
   Request access types.
 */
enum dev_persist_op_e
{
  /**
     Erase all entries, start fresh with a blank storage.  This does
     not check for previous storage contents.
   */
  DEV_PERSIST_ERASE,

  /**
     Pack storage. Collect all erased entries.
   */
  DEV_PERSIST_PACK,

  /**
     Read an entry. Caller will get data pointer set to start of value
     if present.
   */
  DEV_PERSIST_READ,

  /**
     Write an entry. Entry will be replaced if present already.  This
     may pack existing storage.
   */
  DEV_PERSIST_WRITE,

  /**
     Remove a given entry, if present.
   */
  DEV_PERSIST_REMOVE,
};

/**
   Persist class request.
 */
struct dev_persist_rq_s
{
  struct dev_request_s rq;

  /** Access return status */
  error_t err;

  /** Value descriptor */
  const struct dev_persist_descriptor_s *descriptor;

  /** Type of access */
  enum dev_persist_op_e op : 3;
  /** UID offset to add to descriptor's base UID */
  uint16_t uid_offset;

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

/**
   @this contains statistics about device's backing storage.

   Sizes, used and total, may not actually equal the sum of all stored
   values.  There may be some overhead in storage of each value.
 */
struct dev_persist_info_s
{
  /** Total theorical size */
  size_t storage_size;
  /** Total bytes used */
  size_t used_size;
};

STRUCT_INHERIT(dev_persist_rq_s, dev_request_s, rq);

/**
   @this retrieves statistics about persist device.
 */
#define DEV_PERSIST_INFO(n) void (n)(struct device_persist_s *accessor, \
                                     struct dev_persist_info_s *info)
typedef DEV_PERSIST_INFO(devpersist_info_t);

/**
   @this requests operations on a persist device.
 */
#define DEV_PERSIST_REQUEST(n) void  (n)(struct device_persist_s *accessor,   \
                                        struct dev_persist_rq_s *rq)
typedef DEV_PERSIST_REQUEST(devpersist_request_t);

DRIVER_CLASS_TYPES(DRIVER_CLASS_PERSIST, persist,
                   devpersist_info_t *f_info;
                   devpersist_request_t *f_request;
                   );

#define DRIVER_PERSIST_METHODS(prefix)                            \
  ((const struct driver_class_s*)&(const struct driver_persist_s){      \
    .class_ = DRIVER_CLASS_PERSIST,                               \
    .f_info = prefix ## _info,                                    \
    .f_request = prefix ## _request,                              \
  })


#if defined(CONFIG_DEVICE_PERSIST)

BUSY_WAITING_FUNCTION
config_depend(CONFIG_DEVICE_PERSIST)
inline error_t
dev_persist_spin_op(struct device_persist_s *accessor,
                    struct dev_persist_rq_s *rq)
{
  struct dev_request_status_s st;
  dev_request_spin_init(&rq->rq, &st);
  DEVICE_OP(accessor, request, rq);
  dev_request_spin_wait(&st);
  return rq->err;
}

/** Synchronous persist device operation function. This function use
    the scheduler api to put current context in wait state during the
    request. */
config_depend_and2(CONFIG_DEVICE_PERSIST, CONFIG_MUTEK_CONTEXT_SCHED)
inline error_t
dev_persist_wait_op(struct device_persist_s *accessor,
                    struct dev_persist_rq_s *rq)
{
  struct dev_request_status_s st;
  dev_request_sched_init(&rq->rq, &st);
  DEVICE_OP(accessor, request, rq);
  dev_request_sched_wait(&st);
  return rq->err;
}

config_depend_and2(CONFIG_DEVICE_PERSIST, CONFIG_MUTEK_CONTEXT_SCHED)
inline error_t
dev_persist_wait_read(struct device_persist_s *accessor,
                      const struct dev_persist_descriptor_s *desc,
                      uint16_t uid_offset,
                      const void **data)
{
  struct dev_persist_rq_s rq = {
    .descriptor = desc,
    .op = DEV_PERSIST_READ,
    .uid_offset = uid_offset,
  };

  error_t err = dev_persist_wait_op(accessor, &rq);

  *data = rq.data;

  return err;
}

config_depend_and2(CONFIG_DEVICE_PERSIST, CONFIG_MUTEK_CONTEXT_SCHED)
inline error_t
dev_persist_wait_write(struct device_persist_s *accessor,
                       const struct dev_persist_descriptor_s *desc,
                       uint16_t uid_offset,
                       const void *data)
{
  struct dev_persist_rq_s rq = {
    .descriptor = desc,
    .op = DEV_PERSIST_WRITE,
    .uid_offset = uid_offset,
    .data = data,
  };

  return dev_persist_wait_op(accessor, &rq);
}

config_depend_and2(CONFIG_DEVICE_PERSIST, CONFIG_MUTEK_CONTEXT_SCHED)
inline error_t
dev_persist_wait_remove(struct device_persist_s *accessor,
                        const struct dev_persist_descriptor_s *desc,
                        uint16_t uid_offset)
{
  struct dev_persist_rq_s rq = {
    .descriptor = desc,
    .op = DEV_PERSIST_REMOVE,
    .uid_offset = uid_offset,
  };

  return dev_persist_wait_op(accessor, &rq);
}

config_depend_and2(CONFIG_DEVICE_PERSIST, CONFIG_MUTEK_CONTEXT_SCHED)
inline error_t
dev_persist_wait_inc(struct device_persist_s *accessor,
                     const struct dev_persist_descriptor_s *desc,
                     uint16_t uid_offset)
{
  struct dev_persist_rq_s rq = {
    .descriptor = desc,
    .op = DEV_PERSIST_WRITE,
    .uid_offset = uid_offset,
    .counter = 1,
  };

  return dev_persist_wait_op(accessor, &rq);
}

config_depend_and2(CONFIG_DEVICE_PERSIST, CONFIG_MUTEK_CONTEXT_SCHED)
inline error_t
dev_persist_wait_counter_read(struct device_persist_s *accessor,
                              const struct dev_persist_descriptor_s *desc,
                              uint16_t uid_offset,
                              uint64_t *value)
{
  struct dev_persist_rq_s rq = {
    .descriptor = desc,
    .op = DEV_PERSIST_READ,
    .uid_offset = uid_offset,
    .counter = 1,
  };

  error_t err;

  err = dev_persist_wait_op(accessor, &rq);

  *value = rq.counter;

  return err;
}

#endif
#endif

