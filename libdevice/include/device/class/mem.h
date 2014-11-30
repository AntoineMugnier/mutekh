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
 * @file
 * @module{Devices support library}
 * @short Memory device driver API
 */

#ifndef __DEVICE_MEM_H__
#define __DEVICE_MEM_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <mutek/kroutine.h>

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
  /** Memory is writable */
  DEV_MEM_WRITABLE      = 0x0001,
  /** Memory is mapped in the processor address space and can be read
      by regular processor load instructions. */
  DEV_MEM_MAPPED_READ   = 0x0002,
  /** Memory is mapped in the processor address space and can be
      written by regular processor store instructions. */
  DEV_MEM_MAPPED_WRITE  = 0x0004,
  /** Data are fetched through a non-consistent cache and may not
      match actual content of memory. */
  DEV_MEM_CACHED_READ   = 0x0008,
  /** Data are stored in a write back cache and my not be written
      immediately to the memory. */
  DEV_MEM_CACHED_WRITE  = 0x0010,
  /** Support partial read of a page. */
  DEV_MEM_PARTIAL_READ  = 0x0020,
  /** Support partial write of a page. */
  DEV_MEM_PARTIAL_WRITE = 0x0040,
  /** Partial read can cross page boundary. */
  DEV_MEM_CROSS_READ    = 0x0080,
  /** Partial write can cross page boundary. */
  DEV_MEM_CROSS_WRITE   = 0x0100,
  /** Memory content is lost when the device is stopped. */
  DEV_MEM_VOLATILE      = 0x0200,
  /** Memory needs an erase operation in order to switch bits from 0 to 1. */
  DEV_MEM_ERASE_ONE     = 0x0400,
  /** Memory needs an erase operation in order to switch bits from 1 to 0. */
  DEV_MEM_ERASE_ZERO    = 0x0800,
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
  uint8_t read_cycles_p:5;
  /** @see read_cycles_p */
  uint8_t read_cycles_m:3;

  /** number of allowed erase cycles. @em
      {erase_cycles_m*2^erase_cycles_p}. 0 means infinite. */
  uint8_t erase_cycles_p:5;
  /** @see erase_cycles_p */
  uint8_t erase_cycles_m:3;

  /** log2 of byte size of a page, 0=no paging */
  uint8_t page_log2;

  /** log2 byte size of a page erase, 0=no paging. Only valid when
      either DEV_MEM_ERASE_ONE @ref or @ref DEV_MEM_ERASE_ZERO is set. */
  uint8_t erase_log2;

  /** log2 alignment of partial access. */
  uint8_t partial_log2;

  /** number of allowed partial write between two erase
      operations. 0=infinite. Only valid when either
      DEV_MEM_ERASE_ONE @ref or @ref DEV_MEM_ERASE_ZERO is set. */
  uint8_t partial_write;

  /** type of memory */
  enum dev_mem_type_e  type:3;

  enum dev_mem_flags_e flags:12;
};

ENUM_DESCRIPTOR(dev_mem_rq_type_e, strip:DEV_MEM_OP_, upper, or);

/* @This specifies available memory device operations. When combined,
   operations are performed in declaration order. */
enum dev_mem_rq_type_e
{
  /** Invalidate read cache. Always succeed if there is no cache. Can
      be combined with @ref DEV_MEM_OP_PARTIAL_READ or @ref
      DEV_MEM_OP_PAGE_READ. The @tt count field of the request indicates
      the number of bytes unless it is combined with a @ref
      DEV_MEM_OP_PAGE_READ operation. */
  DEV_MEM_OP_CACHE_INVALIDATE   = 0x0001,
  /** Erase a page on a flash device, perform a TRIM on a disk device.
      Can be combined with @ref DEV_MEM_OP_PAGE_WRITE. The @tt count
      field of the request indicates the number of erased pages. The
      address must be page aligned. This may no be supported by all devices. */
  DEV_MEM_OP_PAGE_ERASE         = 0x0002,
  /** Partial page read. The @tt count field of the request indicates
      the number of bytes. The transfer must not cross page
      boundaries. This may no be supported by all devices. */
  DEV_MEM_OP_PARTIAL_READ       = 0x0004,
  /** Partial page write. The @tt count field of the request indicates
      the number of bytes. The transfer must not cross page
      boundaries. This may no be supported by all devices. */
  DEV_MEM_OP_PARTIAL_WRITE      = 0x0008,
  /** Pages read with scattered data buffers. The @tt count field of
      the request indicates the number of written pages and the @tt
      sc_log2 field indicates the number of pages per buffer. The
      address must be page aligned. */
  DEV_MEM_OP_PAGE_READ          = 0x0010,
  /** Pages write with scattered data buffers. The @tt count field of
      the request indicates the number of read pages and the @tt
      sc_log2 field indicates the number of pages per buffer. The
      address must be page aligned.  */
  DEV_MEM_OP_PAGE_WRITE         = 0x0020,
  /** Flush write cache. Always succeed if there is no cache.  Can be
      combined with @ref DEV_MEM_OP_PARTIAL_WRITE or @ref
      DEV_MEM_OP_PAGE_WRITE. The @tt count field of the request
      indicates the number of bytes unless it is combined with a @ref
      DEV_MEM_OP_PAGE_WRITE operation. */
  DEV_MEM_OP_CACHE_FLUSH        = 0x0040,
  DEV_MEM_OP_CONFIG             = 0x0080,
};

struct dev_mem_config_s;

struct dev_mem_rq_s
{
  struct dev_request_s          base;

  /* Requested operation */
  enum dev_mem_rq_type_e        type:8;

  error_t                       err;

  /* Mask of bands present in data */
  uint8_t                       band_mask;

  /* log2 of the number of pages in a single @ref sc_data buffer. */
  uint8_t                       sc_log2;

  /* Size of data, granularity depends on the requested operation. */
  size_t                        size;

  /* Access byte address */
  uint64_t                      addr;

  union {
    /* buffer for @ref DEV_MEM_OP_PARTIAL_READ and @ref
       DEV_MEM_OP_PARTIAL_WRITE operations */
    uint8_t *data;
    /* array of page buffers for @ref DEV_MEM_OP_PAGE_READ and @ref
       DEV_MEM_OP_PAGE_WRITE operations */
    uint8_t **sc_data;

    struct dev_mem_config_s    *cfg;
  };
};

STRUCT_INHERIT(dev_mem_rq_s, dev_request_s, base);

/** Memory device info() function tempate. @see dev_mem_info_t */
#define DEV_MEM_INFO(n)	error_t  (n) (struct device_mem_s *accessor, \
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


/* Memory device request function template. @see dev_mem_request_t */
#define DEV_MEM_REQUEST(n)	void  (n) (struct device_mem_s *accessor,   \
                                           struct dev_mem_rq_s *rq)

/** @This enqueues a memory device operation request.

    The request kroutine is executed on completion. The @tt err
    indicates the status of the request. Here are possible values:

    @list
      @item @tt 0 : Success
      @item @tt 1 : at least one corrected ECC error on read (not fatal)
      @item @tt -EINVAL : invalid request parameter
      @item @tt -ERANGE : page index out of range
      @item @tt -ENOTSUP : operation not supported on this device
      @item @tt -EIO : IO error
      @item @tt -EBADDATA : uncorrectable ECC error on read
    @end list

    An ending request object can be enqueued again from within the
    kroutine. Other fields of the request are not modified during
    processing.

    @see #DEV_MEM_REQUEST
*/
typedef DEV_MEM_REQUEST(dev_mem_request_t);


DRIVER_CLASS_TYPES(mem,
                   dev_mem_info_t *f_info;
                   dev_mem_request_t *f_request;
                   );

/** Synchronous memory device operation function. This function use a
    busy wait loop during the request. @see dev_mem_wait_op */
config_depend(CONFIG_DEVICE_MEM)
inline error_t dev_mem_spin_op(struct device_mem_s *accessor,
                               struct dev_mem_rq_s *rq)
{
  struct dev_request_status_s st;
  dev_request_spin_init(&rq->base, &st);
  DEVICE_OP(accessor, request, rq);
  dev_request_spin_wait(&st);
  return rq->err;
}

/** Synchronous memory device operation function. This function use
    the scheduler api to put current context in wait state during the
    request.

    This function take care of initializing some fields of the
    request, the @tt type, @tt band_mask, @tt page_count,
    @tt sc_log2 and @tt page_index fields must be initialized by the
    caller. */
config_depend_and2(CONFIG_DEVICE_MEM, CONFIG_MUTEK_SCHEDULER)
inline error_t dev_mem_wait_op(struct device_mem_s *accessor,
                               struct dev_mem_rq_s *rq)
{
  struct dev_request_status_s st;
  dev_request_sched_init(&rq->base, &st);
  DEVICE_OP(accessor, request, rq);
  dev_request_sched_wait(&st);
  return rq->err;
}

/** @internal @This handles read/write operations to mapped memories
    using the @ref memcpy function. */
config_depend(CONFIG_DEVICE_MEM)
void dev_mem_mapped_op_helper(uintptr_t base, uint_fast8_t page_log2,
                              struct dev_mem_rq_s *rq);

#endif

