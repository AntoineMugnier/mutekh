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

    Copyright (c) 2021, Nicolas Pouillon <nipo@ssji.net>
*/

/**
   @file
   @module {Core::Devices support library}
   @short 1-Wire driver class

   @section{Purpose}

   The @em {1-Wire} class gives access to iButton /
   1-Wire devices from Dallas/Maxim

   @end section
 */

#ifndef __DEVICE_ONEWIRE_H__
#define __DEVICE_ONEWIRE_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/device.h>
#include <device/driver.h>
#include <device/resources.h>
#include <device/request.h>

#include <hexo/enum.h>

struct device_s;
struct driver_s;
struct device_onewire_s;
struct driver_onewire_s;
struct dev_onewire_drv_ctx_s;
struct dev_onewire_rq_s;

/*
  Request datatypes
 */

ENUM_DESCRIPTOR(dev_onewire_rq_type_e, strip:DEV_ONEWIRE_, upper);
ENUM_DESCRIPTOR(dev_onewire_transfer_dir_e, strip:DEV_ONEWIRE_, upper);

/** 1-Wire "ROM", i.e. device unique ID */
struct dev_onewire_rom_s
{
  union {
    /** Little-endian */
    uint64_t raw;
    struct {
      uint8_t family;
      /** Little-endian */
      uint8_t uid[6];
      uint8_t crc;
    } structured;
  };
};

enum dev_onewire_rq_type_e
{
  /** Targetted data transfer */
  DEV_ONEWIRE_DATA,
  /** Raw data transfer */
  DEV_ONEWIRE_RAW,
  /** Enumeration */
  DEV_ONEWIRE_SEARCH,
};

enum dev_onewire_transfer_dir_e
{
  /** Transfer data to device */
  DEV_ONEWIRE_WRITE,
  /** Transfer data from device */
  DEV_ONEWIRE_READ,
};

struct dev_onewire_transfer_s
{
  enum dev_onewire_transfer_dir_e direction;
  size_t size;
  uint8_t *data;
};

struct dev_onewire_data_s
{
  /**
     Only used for targetted data transfer.
     If NULL, skip device selection and use Skip ROM command
  */
  const struct dev_onewire_rom_s *rom;
  struct dev_onewire_transfer_s *transfer;
  size_t transfer_count;
};

/**
   Search requests may end with:
   @list
   @item -EIO: Nobody drove the bus after some given bit during
   selection. This is most probably a device detach during
   enumeration.
   @item 0: Enumeration happened, collision may have happened. If so, caller should check for @tt collision bit.
   @end list
 */
struct dev_onewire_search_s
{
  /**
     Target ROM
   */
  struct dev_onewire_rom_s rom, collision;

  uint8_t discover_after;

  /** Search for devices with alarm condition only */
  bool_t alarm_only;
};

struct dev_onewire_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  /** Request type */
  enum dev_onewire_rq_type_e  BITFIELD(type,2);

  union {
    struct dev_onewire_data_s data;
    struct dev_onewire_search_s search;
  };
};

DEV_REQUEST_INHERIT(onewire);
DEV_REQUEST_QUEUE_OPS(onewire);

/*
  Driver API
 */

/** @see dev_onewire_request_t */
#define DEV_ONEWIRE_REQUEST(n) void (n) (                              \
    const struct device_onewire_s *accessor,                           \
    struct dev_onewire_rq_s *rq)

/** @This enqueues a request.

   The kroutine of the request may be executed from within this
   function. Please read @xref {Nested device request completion}. 

   @xsee {Purpose} @see dev_onewire_request_type_e
*/
typedef DEV_ONEWIRE_REQUEST(dev_onewire_request_t);

DRIVER_CLASS_TYPES(DRIVER_CLASS_ONEWIRE, onewire,
    dev_onewire_request_t *f_request;
);

/** @see driver_onewire_s */
#define DRIVER_ONEWIRE_METHODS(prefix)                                    \
  ((const struct driver_class_s*)&(const struct driver_onewire_s){        \
    .class_ = DRIVER_CLASS_ONEWIRE,                                       \
    .f_request = prefix ## _request,                                  \
  })

/*
  Client API helpers
 */

DEV_REQUEST_WAIT_FUNC(onewire);

/** @This performs a write/read operation. */
config_depend_and2_inline(CONFIG_DEVICE_ONEWIRE, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_onewire_wait_write_read(
    const struct device_onewire_s *accessor,
    const struct dev_onewire_rom_s *rom,
    const uint8_t *wdata, size_t wsize,
    uint8_t *rdata, size_t rsize),
{
  struct dev_onewire_transfer_s transfers[2] = {
    {
      .direction = DEV_ONEWIRE_WRITE,
      .data = (void*)wdata,
      .size = wsize,
    },
    {
      .direction = DEV_ONEWIRE_READ,
      .data = rdata,
      .size = rsize,
    },
  };

  struct dev_onewire_rq_s rq = {
    .type = DEV_ONEWIRE_DATA,
    .data.rom = rom,
    .data.transfer = transfers,
    .data.transfer_count = 2,
  };

  return dev_onewire_wait_rq(accessor, &rq);
});

/** @This performs a write operation. */
config_depend_and2_inline(CONFIG_DEVICE_ONEWIRE, CONFIG_MUTEK_CONTEXT_SCHED,
error_t dev_onewire_wait_write(
    const struct device_onewire_s *accessor,
    const struct dev_onewire_rom_s *rom,
    const uint8_t *wdata, size_t wsize),
{
  struct dev_onewire_transfer_s transfers[1] = {
    {
      .direction = DEV_ONEWIRE_WRITE,
      .data = (void*)wdata,
      .size = wsize,
    },
  };

  struct dev_onewire_rq_s rq = {
    .type = DEV_ONEWIRE_DATA,
    .data.rom = rom,
    .data.transfer = transfers,
    .data.transfer_count = 1,
  };

  return dev_onewire_wait_rq(accessor, &rq);
});

/*
  Resource helpers
 */

#ifdef CONFIG_DEVICE_ONEWIRE
# define DEV_STATIC_RES_DEV_ONEWIRE(path_)                          \
  DEV_STATIC_RES_DEVCLASS_PARAM("onewire", path_, DRIVER_CLASS_ONEWIRE)
#else
# define DEV_STATIC_RES_DEV_ONEWIRE(path_) { .type = DEV_RES_UNUSED, }
#endif

/**
   Standard 1-Wire opcodes
 */
enum dev_onewire_opcode_e
{
  DEV_ONEWIRE_OPCODE_SEARCH_ROM   = 0xf0,
  DEV_ONEWIRE_OPCODE_READ_ROM     = 0x33,
  DEV_ONEWIRE_OPCODE_MATCH_ROM    = 0x55,
  DEV_ONEWIRE_OPCODE_SKIP_ROM     = 0xcc,
  DEV_ONEWIRE_OPCODE_ALARM_SEARCH = 0xec,
};

#endif
