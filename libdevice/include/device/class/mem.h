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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009
*/

/**
  @file
  @module {Core::Devices support library}
  @short Memory device driver API
  @index {Memory device} {Device classes}
  @csee DRIVER_CLASS_MEM
*/

#ifndef __DEVICE_MEM_H__
#define __DEVICE_MEM_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <mutek/kroutine.h>
#include <hexo/enum.h>

#include <device/driver.h>
#include <device/request.h>

struct device_s;
struct driver_s;
struct device_mem_s;
struct driver_mem_s;

ENUM_DESCRIPTOR(dev_mem_type_e, strip:DEV_MEM_, upper);

enum dev_mem_type_e
{
  /* RAM memory */
  DEV_MEM_RAM,
  /* ROM memory */
  DEV_MEM_ROM,
  /* Flash or EEPROM memory */
  DEV_MEM_FLASH,
  /* Disk drive memory */
  DEV_MEM_DISK,
  /* Out of band Flash memory */
  DEV_MEM_OOB,
  /* Hardware ECC */
  DEV_MEM_ECC,
};

ENUM_DESCRIPTOR(dev_mem_flags_e, strip:DEV_MEM_, upper, or);

enum dev_mem_flags_e
{
  /** Memory is mapped in the processor address space and can be read
      by regular processor load instructions. */
  DEV_MEM_MAPPED_READ   = 0x0001,
  /** Memory is mapped in the processor address space and can be
      written by regular processor store instructions. */
  DEV_MEM_MAPPED_WRITE  = 0x0002,
  /** Data are fetched through a non-consistent cache and may not
      match actual content of memory. */
  DEV_MEM_CACHED_READ   = 0x0004,
  /** Data are stored in a write back cache and my not be written
      immediately to the memory. */
  DEV_MEM_CACHED_WRITE  = 0x0008,
  /** Support partial read of a page. */
  DEV_MEM_PARTIAL_READ  = 0x0010,
  /** Support partial write of a page. */
  DEV_MEM_PARTIAL_WRITE = 0x0020,
  /** Partial read can cross page boundary. */
  DEV_MEM_CROSS_READ    = 0x0040,
  /** Partial write can cross page boundary. */
  DEV_MEM_CROSS_WRITE   = 0x0080,
  /** Support page reading operations. */
  DEV_MEM_PAGE_READ  = 0x0100,
  /** Support page writing operations. */
  DEV_MEM_PAGE_WRITE = 0x0200,
  /** Memory content is lost when the device is stopped. */
  DEV_MEM_VOLATILE      = 0x0400,
  /** Memory needs an erase operation in order to switch bits from 0 to 1. */
  DEV_MEM_ERASE_ONE     = 0x0800,
  /** Memory needs an erase operation in order to switch bits from 1 to 0. */
  DEV_MEM_ERASE_ZERO    = 0x1000,
};

struct dev_mem_info_s
{
  /** size of memory in number of pages according to @ref page_log2. */
  uint64_t size;

  /** mapped address of memory, valid when @ref DEV_MEM_MAPPED_READ or
      @ref DEV_MEM_MAPPED_WRITE is set. */
  uintptr_t map_base;

  /** number of allowed reads cycles between two erase operations. @em
      {read_cycles_m*2^read_cycles_p}. 0 means infinite. */
  uint8_t BITFIELD(read_cycles_p,5);
  /** @see read_cycles_p */
  uint8_t BITFIELD(read_cycles_m,3);

  /** number of allowed erase cycles. @em
      {erase_cycles_m*2^erase_cycles_p}. 0 means infinite. */
  uint8_t BITFIELD(erase_cycles_p,5);
  /** @see erase_cycles_p */
  uint8_t BITFIELD(erase_cycles_m,3);

  /** log2 of default page size in bytes used for read and write
      operations. */
  uint8_t page_log2;

  /** log2 of default page size in bytes used for erase operation,
      Only valid when either @ref DEV_MEM_ERASE_ONE or @ref
      DEV_MEM_ERASE_ZERO is set. */
  uint8_t erase_log2;

  /** log2 alignment of partial access. */
  uint8_t partial_log2;

  /** number of allowed partial write between two erase
      operations. 0=infinite. Only valid when either
      @ref DEV_MEM_ERASE_ONE or @ref DEV_MEM_ERASE_ZERO is set. */
  uint8_t partial_write;

  /** type of memory */
  enum dev_mem_type_e  BITFIELD(type,3);

  enum dev_mem_flags_e BITFIELD(flags,13);
};

/** @internal */
#define _DEV_MEM_CACHE 0x00
/** @internal */
#define _DEV_MEM_READ 0x01
/** @internal */
#define _DEV_MEM_WRITE 0x02
/** @internal */
#define _DEV_MEM_ERASE 0x04
/** @internal */
#define _DEV_MEM_OP_MASK 0x07

/** @internal */
#define _DEV_MEM_PAGE 0x08
/** @internal */
#define _DEV_MEM_PARTIAL 0x10
/** @internal */
#define _DEV_MEM_ALL 0x20

/** @internal */
#define _DEV_MEM_FLUSH 0x40
/** @internal */
#define _DEV_MEM_INVAL 0x80

/* @This specifies available memory device operations. When combined,
   operations are performed in declaration order. */
enum dev_mem_rq_type_e
{
  DEV_MEM_OP_NOP                    = _DEV_MEM_CACHE,
  /** Partial page read. All fields in @ref dev_mem_rq_s::partial must
      be set. Read operations crossing page boundaries may not be supported. */
  DEV_MEM_OP_PARTIAL_READ           = _DEV_MEM_READ | _DEV_MEM_PARTIAL,
  /** Same as @ref DEV_MEM_OP_PARTIAL_READ, do not rely on cache content. */
  DEV_MEM_OP_PARTIAL_READ_UNCACHED  = _DEV_MEM_READ | _DEV_MEM_PARTIAL | _DEV_MEM_FLUSH,
  /** Partial page write. All fields in @ref dev_mem_rq_s::partial
      must be set. Write operations crossing page boundaries may not
      be supported. */
  DEV_MEM_OP_PARTIAL_WRITE          = _DEV_MEM_WRITE | _DEV_MEM_PARTIAL,
  /** Same as DEV_MEM_OP_PARTIAL_WRITE, do not keep data in the write cache. */
  DEV_MEM_OP_PARTIAL_WRITE_THROUGH  = _DEV_MEM_WRITE | _DEV_MEM_PARTIAL | _DEV_MEM_FLUSH,
  /** Pages read with scattered data buffers. All fields in @ref
      dev_mem_rq_s::page must be set. */
  DEV_MEM_OP_PAGE_READ              = _DEV_MEM_READ | _DEV_MEM_PAGE,
  /** Same as @ref DEV_MEM_OP_PAGE_READ, do not rely on cache content. */
  DEV_MEM_OP_PAGE_READ_UNCACHED     = _DEV_MEM_READ | _DEV_MEM_PAGE | _DEV_MEM_FLUSH,
  /** Pages write with scattered data buffers. All fields in @ref
      dev_mem_rq_s::page must be set. */
  DEV_MEM_OP_PAGE_WRITE             = _DEV_MEM_WRITE | _DEV_MEM_PAGE,
  /** Same as DEV_MEM_OP_PAGE_WRITE, do not keep data in the write cache. */
  DEV_MEM_OP_PAGE_WRITE_THROUGH     = _DEV_MEM_WRITE | _DEV_MEM_PAGE | _DEV_MEM_FLUSH,
  /** Erase pages on a flash device or perform a TRIM on a disk
      device. All fields in @ref dev_mem_rq_s::page must be set except
      @tt sc_data. */
  DEV_MEM_OP_PAGE_ERASE             = _DEV_MEM_ERASE | _DEV_MEM_PAGE,
  /** Erase the whole device. */
  DEV_MEM_OP_ERASE                  = _DEV_MEM_ERASE | _DEV_MEM_ALL,
  /** Flush the whole write cache. Always succeed if there is no cache. */
  DEV_MEM_OP_CACHE_FLUSH            = _DEV_MEM_CACHE | _DEV_MEM_FLUSH | _DEV_MEM_ALL,
  /** Invalidate the whole read cache. Always succeed if there is no cache. */
  DEV_MEM_OP_CACHE_INVALIDATE       = _DEV_MEM_CACHE | _DEV_MEM_INVAL | _DEV_MEM_ALL,
  /** Partially flush the write cache. All fields in @ref dev_mem_rq_s::partial
      must be set. Always succeed if there is no cache. */
  DEV_MEM_OP_CACHE_PARTIAL_FLUSH    = _DEV_MEM_CACHE | _DEV_MEM_FLUSH,
  /** Partially invalidates the read cache. All fields in @ref
      dev_mem_rq_s::partial must be set. Always succeed if there is no
      cache. */
  DEV_MEM_OP_CACHE_PARTIAL_INVAL    = _DEV_MEM_CACHE | _DEV_MEM_INVAL,
};

struct dev_mem_page_sc_s
{
  /** Byte address of accessed page on the device. The address has to be
      aligned on the default page size as reported in @ref dev_mem_info_s. */
  uint64_t            addr;

  /* Buffers used to store the accessed pages */
  uint8_t             *data;
};

struct dev_mem_rq_s
{
  union {
    struct dev_request_s base;
    FIELD_USING(struct dev_request_s, error);
    FIELD_USING(struct dev_request_s, pvdata);
  };

  /** Requested operation */
  enum dev_mem_rq_type_e        BITFIELD(type,8);

  /** Mask of bands present in data */
  uint8_t                       band_mask;

  union {
    struct {
      /** Byte address of access */
      uint64_t                  addr;

      /** Buffers used to store accessed data */
      uint8_t                   *data;

      /** Bytes count */
      uint32_t                  size;
    } partial;

    struct {
      /** Array of source and destination page addresses. */
      const struct dev_mem_page_sc_s *sc;

      /** Number of entries in the the @tt sc array. */
      uint8_t                   sc_count:4;

      /** Log2 of page size used for the access. It can not be less
          than the default page size reported in @ref dev_mem_info_s. */
      uint8_t                   page_log2;
    } page;
  };
};

DEV_REQUEST_INHERIT(mem); DEV_REQUEST_QUEUE_OPS(mem);

/** @see dev_mem_info_t */
#define DEV_MEM_INFO(n)	error_t  (n) (const struct device_mem_s *accessor, \
                                      struct dev_mem_info_s *info,      \
                                      uint8_t band_index)

/** @This fills a @ref dev_mem_info_s object with information about
    a memory band of the device.

    Bands are numbered from 0. The band 0 is always
    present. Additional bands may be present in order to handle out of
    band data, hardware ECC... @This function returns @tt -ENOENT if
    the band index is not valid.

    This function must check the @tt number field of the device
    accessor and report the @tt -ENOENT error for sub-devices which
    are not implemented.

    @see #DEV_MEM_INFO.
*/
typedef DEV_MEM_INFO(dev_mem_info_t);


/** @see dev_mem_request_t */
#define DEV_MEM_REQUEST(n)	void  (n) (const struct device_mem_s *accessor,   \
                                           struct dev_mem_rq_s *rq)

/** @This enqueues a memory device operation request.

    The request kroutine is executed on completion. The @tt err
    indicates the status of the request. Other fields of the request
    are not modified during processing. Here are possible error values:

    @list
      @item @tt 0 : Success
      @item @tt 1 : at least one corrected ECC error on read (not fatal)
      @item @tt -ERANGE : addres or size out of range or not properly aligned
      @item @tt -ENOTSUP : operation not supported on this device
      @item @tt -EIO : IO error
      @item @tt -EBADDATA : uncorrectable ECC error on read
    @end list

    The kroutine of the request may be executed from within this
    function. Please read @xref {Nested device request completion}.

    @see #DEV_MEM_REQUEST
*/
typedef DEV_MEM_REQUEST(dev_mem_request_t);


DRIVER_CLASS_TYPES(DRIVER_CLASS_MEM, mem,
                   dev_mem_info_t *f_info;
                   dev_mem_request_t *f_request;
                   );

/** @see driver_mem_s */
#define DRIVER_MEM_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_mem_s){   \
    .class_ = DRIVER_CLASS_MEM,                                  \
    .f_info = prefix ## _info,                                   \
    .f_request = prefix ## _request,                             \
  })

/** @internal @This handles read/write operations to mapped memories
    using the @ref memcpy function. It can be used in device drivers. */
config_depend(CONFIG_DEVICE_MEM)
error_t dev_mem_mapped_op_helper(uintptr_t base, uintptr_t end,
                                 struct dev_mem_rq_s * __restrict__ rq);

/** @internal @This handles read/write operations to platform flash
    using the @ref flash_erase and @ref flash_write functions. */
config_depend(CONFIG_DEVICE_MEM)
error_t dev_mem_flash_op(uintptr_t base, uintptr_t end,
                         uint_fast8_t page_log2,
                         struct dev_mem_rq_s * __restrict__ rq);

DEV_REQUEST_WAIT_FUNC(mem);

#endif

