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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2013-2014

*/

#ifndef __DEVICE_RESOURCES_H__
#define __DEVICE_RESOURCES_H__

/**
   @file
   @module {Core::Devices support library}
   @short Device resources
   @xsee {Device resources}
*/

#include <assert.h>
#include <hexo/types.h>
#include <hexo/decls.h>

#include <device/types.h>

struct device_s;
struct device_accessor_s;
enum driver_class_e;

/** @This specifies the types of device resource entries.
    @xcsee {Device resources} */
enum dev_resource_type_e
{
    /** Mark a resource entry as not used. */
    DEV_RES_UNUSED = 0,
    /** Memory address range. */
    DEV_RES_MEM,
    /** IO space address range. */
    DEV_RES_IO,
    /** This entry specifies the connection of an irq output of the
        device to an interrupt controller. @see device_res_add_irq */
    DEV_RES_IRQ,
    /** This entry specifies a pin label name along with a range of
        associated contiguous pin ids. It used to define a set of GPIO
        pins which are needed to drive an external device. The label
        specifies a driver specific role for the pin range. A link to
        the GPIO controller for which the pin ids are relevant must be
        specified in a separate @ref DEV_RES_DEV_PARAM resource entry
        named @tt gpio. */
    DEV_RES_GPIO,
    /** This entry specifies a pin label name along with the associated
        pin muxing configuration. It used to define which pins of the
        chip must be dedicated to IOs of an internal device. */
    DEV_RES_IOMUX,
    /** Default serial line configuration for an UART. */
    DEV_RES_UART,
    /** Numerical identifier which uniquely identify an instance of
        the device. Processor devices must use this resource in order
        to specifies the cpu id. */
    DEV_RES_ID,
    /** Vendor ID. Both a numerical and a string values can be
        specified. The exact meaning of the value depends on the
        parent enumerator device. */
    DEV_RES_VENDOR,
    /** Product ID. Both a numerical and a string values can be
        specified. The exact meaning of the value depends on the
        parent enumerator device. */
    DEV_RES_PRODUCT,
    /** Hardware revision number. */
    DEV_RES_REVISION,
    /** Constant frequency value. */
    DEV_RES_FREQ,
    /** CMU Clock multiplexer and scaler configuration entry.
        @see device_res_add_cmu_mux */
    DEV_RES_CMU_MUX,
    /** CMU Clock oscillator configuration entry.
        @see device_res_add_cmu_osc */
    DEV_RES_CMU_OSC,
    /** Device clock source mapping. @see device_res_add_clock_src */
    DEV_RES_CLK_SRC,
    /** @experimental Device clock throttling modes. @see #DEV_STATIC_RES_CLOCK_MODES */
    DEV_RES_CLK_MODES,
    /** Generic string parameters. The exact meaning of the value is
        driver dependent. */
    DEV_RES_STR_PARAM,
    /** Generic binary object parameter. The exact meaning of the
        value is driver dependent. */
    DEV_RES_BLOB_PARAM,
    /** Generic integer parameter. The exact meaning of the value is
        driver dependent. */
    DEV_RES_UINT_PARAM,
    /** Named dependency on an other device. The exact meaning of the
        value may be device class dependent or driver dependent. The
        device initialization will not occur until the specified path
        points to an existing and properly initialized device. The
        class information is used to check for availability of a
        specific API implementation by the driver associated to the
        device, it may be @ref DRIVER_CLASS_NONE. */
    DEV_RES_DEV_PARAM,
    /** Generic integer array parameter. The exact meaning of the
        value is driver dependent. */
    DEV_RES_UINT_ARRAY_PARAM,
    /** I2C device address. */
    DEV_RES_I2C_ADDR,
    /** DMA channel mapping. The @tt config parameter is driver
        specific. */
    DEV_RES_DMA,
    /** Number of resource types */
    DEV_RES_TYPES_COUNT,
};

enum dev_resource_flags_e
{
  /** first resource field is a pointer which must be freeed on cleanup */
  DEVICE_RES_FLAGS_FREE_PTR0 = 1,
  /** second resource field is a pointer which must be freeed on cleanup */
  DEVICE_RES_FLAGS_FREE_PTR1 = 2,
  /** reserved for use by enumerator driver */
  DEVICE_RES_FLAGS_ENUM_RESERVED0 = 16,
};

/** Device resource entry. @see dev_resource_table_s */
struct dev_resource_s
{
  union {
    /** @internal */
    const void *                ptr[2];
    /** @internal */
    uintptr_t                   uint[2];
    /** @internal */
    uint64_t                    uint64;

    /** @see DEV_RES_MEM */
    struct {
      uintptr_t                 start;
      uintptr_t                 end;
    }                           mem;

    /** @see DEV_RES_IO */
    struct {
      uintptr_t                 start;
      uintptr_t                 end;
    }                           io;

    /** @see DEV_RES_IRQ */
    struct {
      /** id of irq source endpoint of the device */
      uintptr_t                 BITFIELD(src_id,CONFIG_DEVICE_IRQ_MAX_OUTPUT_ID);
      /** id of irq sink endpoint of interrupt controller */
      uintptr_t                 BITFIELD(sink_id,CONFIG_DEVICE_IRQ_MAX_INPUT_ID);
      /** initial irq trigger mode, @see dev_irq_sense_modes_e */
      uintptr_t                 BITFIELD(trig_mode,8);
      /** logical irq id, used when @tt trig_mode is @ref DEV_IRQ_SENSE_ID_BUS */
      uintptr_t                 BITFIELD(irq_id,CONFIG_DEVICE_IRQ_MAX_LOGICAL_ID);
      /** specifies source endpoints of the controller that can be used to forward this irq. */
      uintptr_t                 BITFIELD(route_mask,CONFIG_DEVICE_IRQ_MAX_ROUTES);
    }                           irq;

    /** @see DEV_RES_GPIO */
    struct {
      const char                *label;
      uintptr_t                 BITFIELD(id,CONFIG_DEVICE_GPIO_MAX_ID);
      uintptr_t                 BITFIELD(width,CONFIG_DEVICE_GPIO_MAX_WIDTH);
    }                           gpio;

    /** @see DEV_RES_IOMUX */
    struct {
      const char                *label;
      uintptr_t                 BITFIELD(demux,CONFIG_DEVICE_IOMUX_MAX_DEMUX);
      uintptr_t                 BITFIELD(io_id,CONFIG_DEVICE_IOMUX_MAX_ID);
      uintptr_t                 BITFIELD(mux,CONFIG_DEVICE_IOMUX_MAX_MUX);
      uintptr_t                 BITFIELD(config,CONFIG_DEVICE_IOMUX_MAX_CONFIG);
    }                           iomux;

    /** @see DEV_RES_DMA */
    struct {
 char                *label;
      uintptr_t                 config;
      uintptr_t                 BITFIELD(channel,5);
    }                           dma;

    /** @see DEV_RES_UART */
    struct {
      uintptr_t                 BITFIELD(baudrate,26);
      uintptr_t                 BITFIELD(data_bits,4);
      uintptr_t                 BITFIELD(stop_bits,2);
      uintptr_t                 BITFIELD(parity,2);
      uintptr_t                 BITFIELD(flow_ctrl,1);
      uintptr_t                 BITFIELD(half_duplex,1);
    }                           uart;

    /** @ref DEV_RES_ID */
    struct {
      uintptr_t                 major;          //< dynamic numeric id
      uintptr_t                 minor;          //< dynamic numeric id
    }                           id;

    /** @see DEV_RES_REVISION */
    struct {
      uintptr_t                 major;
      uintptr_t                 minor;
    }                           revision;

    /** @see DEV_RES_VENDOR */
    struct {
      /** optional vendor numeric id, may be -1 */
      uintptr_t                 id;
      /** optional vendor string id, may be NULL */
      const char                *name;
    }                           vendor;

    /** @see DEV_RES_PRODUCT */
    struct {
      /** optional device numeric id, may be -1 */
      uintptr_t                 id;
      /** optional device string, may be NULL */
      const char                *name;
    }                           product;

    /** @see DEV_RES_CMU_MUX */
    struct {
      /** node id of the input clock signal */
      uint64_t                  BITFIELD(parent,CONFIG_DEVICE_CLOCK_MAX_ID);
      /** node id of the output clock signal */
      uint64_t                  BITFIELD(node,CONFIG_DEVICE_CLOCK_MAX_ID);
      /** numerator of the frequency scaling */
      uint64_t                  BITFIELD(num,CONFIG_DEVICE_CLOCK_FRAC_WIDTH);
      /** denominator of the frequency scaling */
      uint64_t                  BITFIELD(denom,CONFIG_DEVICE_CLOCK_FRAC_WIDTH);
      /** mask of associated configurations */
      uint64_t                  BITFIELD(config,CONFIG_DEVICE_CLOCK_MAX_CONFIG);
    }                           cmu_mux;

    /** @see DEV_RES_CMU_OSC */
    struct __attribute__((packed)) {
      /** numerator of the frequency fractional part */
      uint64_t                  BITFIELD(num,CONFIG_DEVICE_CLOCK_OSCN_WIDTH);
      /** denominator of the frequency fractional part */
      uint64_t                  BITFIELD(denom,CONFIG_DEVICE_CLOCK_OSCD_WIDTH);
      /** node id */
      uint32_t                  BITFIELD(node,CONFIG_DEVICE_CLOCK_MAX_ID);
      /** mask of associated configurations */
      uint32_t                  BITFIELD(config,CONFIG_DEVICE_CLOCK_MAX_CONFIG);
      /** accuracy, @see dev_freq_s */
      uint32_t                  BITFIELD(acc_m,3);
      /** accuracy, @see dev_freq_s */
      uint32_t                  BITFIELD(acc_e,5);
    }                           cmu_osc;

    /** @see DEV_RES_CLK_SRC */
    struct {
      /** path to the clock source device */
      const char                *src;
      /** node id of the source endpoint, relevant to the device
          pointed to by @tt src. */
      uintptr_t                 BITFIELD(src_ep,CONFIG_DEVICE_CLOCK_MAX_ID);
      /** node id of the sink endpoint, relevant to the device which
          holds this resource. */
      uintptr_t                 BITFIELD(sink_ep,CONFIG_DEVICE_CLOCK_MAX_ID);
    }                           clock_src;

#ifdef CONFIG_DEVICE_CLOCK_VARFREQ
    /** @see DEV_RES_STR_PARAM */
    struct {
      uintptr_t                 BITFIELD(sink_ep,CONFIG_DEVICE_CLOCK_MAX_ID);
      /** device driver throttling modes to clock provider mask bits */
      uintptr_t                 BITFIELD(modes,CONFIG_DEVICE_CLOCK_MODES*CONFIG_DEVICE_CLOCK_MASKB);
    }                           clock_modes;
#endif

    /** @see DEV_RES_FREQ */
    struct __attribute__((packed)) {
      /** numerator of the frequency fractional part */
      uint64_t                  BITFIELD(num,CONFIG_DEVICE_CLOCK_OSCN_WIDTH);
      /** denominator of the frequency fractional part */
      uint64_t                  BITFIELD(denom,CONFIG_DEVICE_CLOCK_OSCD_WIDTH);
      /** accuracy, @see dev_freq_s */
      uint32_t                  BITFIELD(acc_m,3);
      /** accuracy, @see dev_freq_s */
      uint32_t                  BITFIELD(acc_e,5);
    }                           freq;

    /** @see DEV_RES_STR_PARAM */
    struct {
      const char                *name;
      const char                *value;
    }                           str_param;

    /** @see DEV_RES_BLOB_PARAM */
    struct {
      const char                *name;
      const void                *value;
    }                           blob_param;

    /** @see DEV_RES_UINT_PARAM */
    struct {
      const char                *name;
      uintptr_t                 value;
    }                           uint_param;

    /** @see DEV_RES_DEV_PARAM */
    struct {
      const char                *name;
      const char                *dev;
      uint8_t                   class_;
    }                           dev_param;

    /** @see DEV_RES_UINT_ARRAY_PARAM */
    struct {
      const char                *name;
      const uintptr_t           *array;
      uint16_t                  count;
    }                           uint_array_param;

#ifndef CONFIG_COMPILE_NOBITFIELD /* mkdoc:skip */
    uint8_t                     _align[4 * sizeof(uintptr_t) - 2];
  } __attribute__((packed))
#else
  }
#endif
    u;

  /** resource descriptor type @see dev_resource_type_e */
  enum dev_resource_type_e BITFIELD(type,8);
  enum dev_resource_flags_e BITFIELD(flags,8);
};

#ifndef CONFIG_COMPILE_NOBITFIELD /* mkdoc:skip */
STATIC_ASSERT(resource_entry_size_exceeded, sizeof(struct dev_resource_s)
              == 4 * sizeof(uintptr_t));
#endif

enum dev_resource_table_flags_e
{
  DEVICE_RES_TBL_FLAGS_ALLOCATED = 1,
  DEVICE_RES_TBL_FLAGS_STATIC_CONST = 2,
};

/** Table of device resources */
struct dev_resource_table_s
{
#ifdef CONFIG_DEVICE_RESOURCE_ALLOC
  /** chaining of resource entry block */
  struct dev_resource_table_s     *next;
#endif
  enum dev_resource_table_flags_e BITFIELD(flags,8);
  /** number of entries in the block */
  uint8_t                         count;
  __attribute__((aligned(8)))
  struct dev_resource_s           table[0];
};

/** @internal */
#define DEV_STATIC_RESOURCES_ARRAY(args_...)                    \
  ((struct dev_resource_s[]){                          \
    args_                                                       \
  })

/** @internal @This yields a static pointer to an static initialized
    @ref dev_resource_s object.
    @see #DEV_DECLARE_STATIC */
#define DEV_STATIC_RESOURCES(args_...)                          \
  (const struct dev_resource_table_s *)&(const struct {         \
      struct dev_resource_table_s _table;                       \
      struct dev_resource_s _entries[ARRAY_SIZE(DEV_STATIC_RESOURCES_ARRAY(args_))]; \
  }){                                                           \
    ._table = {                                                 \
      .flags = DEVICE_RES_TBL_FLAGS_STATIC_CONST,               \
      .count = ARRAY_SIZE(DEV_STATIC_RESOURCES_ARRAY(args_)),   \
    },                                                          \
    ._entries = {                                               \
      args_                                                     \
    }                                                           \
  }

#ifdef CONFIG_DEVICE_RESOURCE_ALLOC
/** @This iterates over resources entries of a device. */
# define DEVICE_RES_FOREACH(dev, rvar, ... /* loop body */ )            \
  do {                                                                  \
    struct dev_resource_table_s *_tbl;                                  \
    for (_tbl = (dev)->res_tbl; _tbl != NULL; _tbl = _tbl->next)        \
      {                                                                 \
        uint_fast8_t _i;                                                \
        for (_i = 0; ; _i++)                                            \
          {                                                             \
            if (_i >= _tbl->count)                                      \
              goto _cont;  /* make break/continue work in body */       \
            struct dev_resource_s *rvar = &_tbl->table[_i];             \
            { __VA_ARGS__ }                                             \
          }                                                             \
        goto _end;                                                      \
      _cont:;                                                           \
      }                                                                 \
  _end:;                                                                \
  } while(0)
#else
/** @This iterates over resources entries of a device. */
# define DEVICE_RES_FOREACH(dev, rvar, ... /* loop body */ )            \
  do {                                                                  \
    struct dev_resource_table_s *_tbl = (dev)->res_tbl;                 \
    uint_fast8_t _i;                                                    \
    if (_tbl == NULL)                                                   \
      break;                                                            \
    for (_i = 0; ; _i++)                                                \
      {                                                                 \
        if (_i >= _tbl->count)                                          \
          break;  /* make break/continue work in body */                \
        struct dev_resource_s *rvar = &_tbl->table[_i];                 \
        { __VA_ARGS__ }                                                 \
      }                                                                 \
  } while(0)
#endif

/********************************************************************************/

/** @internal @This returns a poiner to next unused resource slot. */
config_depend(CONFIG_DEVICE_RESOURCE_ALLOC)
error_t device_res_alloc(struct device_s *dev, struct dev_resource_s **res,
			 enum dev_resource_type_e type);

/** @internal @This releases the resource values and mark the resource
    entry as @ref DEV_RES_UNUSED. */
config_depend(CONFIG_DEVICE_RESOURCE_ALLOC)
void device_res_cleanup(struct dev_resource_s *r);

/** @internal @This looks up a resource entry with given type. The @tt
    index parameter can be used to find different resources of the
    same type. */
struct dev_resource_s * device_res_get(const struct device_s *dev,
                                       enum dev_resource_type_e type,
                                       uint_fast8_t index);

/** @internal @This looks up a resource entry with given type and
    name. The first resource field must be a string. The @tt index
    parameter can be used to find different resources of the same
    type. The comparison stops on the first character which does not
    match @tt {[-_A-Za-z0-9]} in @tt name. */
struct dev_resource_s * device_res_get_from_name(const struct device_s *dev,
                                                 enum dev_resource_type_e type,
                                                 uint_fast8_t index, const char *name);

/** @internal @This allocates a new device resource and setups 2
    integer or pointer fields. */
config_depend_inline(CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_alloc_uint(struct device_s *dev,
                              enum dev_resource_type_e type,
                              uintptr_t a, uintptr_t b, struct dev_resource_s **r_),
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc(dev, &r, type);
  if (err)
    return err;

  r->u.uint[0] = a;
  r->u.uint[1] = b;

  if (r_)
    *r_ = r;
  return 0;
})

/** @internal @This looks up a resource entry and reads one or two
    integer fields. @tt a and @tt b pointers may be @tt NULL. */
inline error_t device_res_get_uint(const struct device_s *dev,
                                   enum dev_resource_type_e type,
                                   uint_fast8_t id, uintptr_t *a, uintptr_t *b)
{
  const struct dev_resource_s *r;

  if (!(r = device_res_get(dev, type, id)))
    return -ENOENT;

  if (a)
    *a = r->u.uint[0];
  if (b)
    *b = r->u.uint[1];
  return 0;
}

/** @internal @This allocates a new device resource and setups a 64
    bits integer field. */
config_depend_inline(CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_alloc_uint64(struct device_s *dev,
                                enum dev_resource_type_e type,
                                uint64_t a, struct dev_resource_s **r_),
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc(dev, &r, type);
  if (err)
    return err;

  r->u.uint64 = a;

  if (r_)
    *r_ = r;
  return 0;
})

/** @internal @This looks up a resource entry and reads a 64 bits
    integer resource field. */
inline error_t device_res_get_uint64(const struct device_s *dev,
                                     enum dev_resource_type_e type,
                                     uint_fast8_t id, uint64_t *a)
{
  struct dev_resource_s *r;

  if (!(r = device_res_get(dev, type, id)))
    return -ENOENT;

  *a = r->u.uint64;
  return 0;
}

/** @internal @This allocates a new device resource and setup one or
    two string fields. The @tt a and @tt b parameters may be @tt
    NULL. Strings are duplicated. */
config_depend(CONFIG_DEVICE_RESOURCE_ALLOC)
error_t device_res_alloc_str(struct device_s *dev,
			     enum dev_resource_type_e type,
			     const char *a, const char *b,
			     struct dev_resource_s **r_);

/********************************************************************************/


/** @This appends an IO space address range to the device resources
    table.  @csee DEV_RES_IO */
config_depend_alwaysinline(CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_io(struct device_s *dev, uintptr_t start, uintptr_t end),
{
  assert(start < end);
  return device_res_alloc_uint(dev, DEV_RES_IO, start, end, NULL);
})

/** @This specifies an IO space address range entry in a static
    device resources table declaration. @csee DEV_RES_IO
    @see #DEV_DECLARE_STATIC */
#define DEV_STATIC_RES_IO(start_, end_)         \
  {                                             \
    .type = DEV_RES_IO,                         \
      .u = { .io = {                            \
        .start = (start_),                      \
        .end = (end_),                          \
      } }                                       \
  }

/** @This looks up an IO resource entry and reads either
    fields. @tt start and @tt end pointers may be @tt NULL.
    @csee DEV_RES_IO */
ALWAYS_INLINE error_t device_res_get_io(const struct device_s *dev,
                                        uint_fast8_t id, uintptr_t *start, uintptr_t *end)
{
  return device_res_get_uint(dev, DEV_RES_IO, id, start, end);
}


/** @This adds a memory space address range to the device resources
    table. @csee DEV_RES_MEM */
config_depend_alwaysinline(CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_mem(struct device_s *dev, uintptr_t start, uintptr_t end),
{
  assert(start < end);
  return device_res_alloc_uint(dev, DEV_RES_MEM, start, end, NULL);
})

/** @This specifies a memory range entry in a static device resources
    table declaration. @csee DEV_RES_MEM @see #DEV_DECLARE_STATIC */
#define DEV_STATIC_RES_MEM(start_, end_)        \
  {                                             \
    .type = DEV_RES_MEM,                        \
      .u = { .mem = {                           \
        .start = (start_),                      \
        .end = (end_),                          \
      } }                                       \
  }

/** @This looks up a memory range resource entry and reads either
    fields. @tt start and @tt end pointers may be @tt NULL.
    @csee DEV_RES_MEM */
ALWAYS_INLINE error_t device_res_get_mem(const struct device_s *dev,
                                         uint_fast8_t id, uintptr_t *start, uintptr_t *end)
{
  return device_res_get_uint(dev, DEV_RES_MEM, id, start, end);
}


/** @This adds a device instance identifier to the device resource
   table. @csee DEV_RES_ID */
config_depend_alwaysinline(CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_id(struct device_s *dev, uintptr_t major, uintptr_t minor),
{
  return device_res_alloc_uint(dev, DEV_RES_ID, major, minor, NULL);
})

/** @This specifies a numerical instance identifier in a static device
    resources table declaration. @csee DEV_RES_ID
    @see #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_ID(major_, minor_)      \
  {                                             \
    .type = DEV_RES_ID,                         \
      .u = { .id = {                            \
        .major = (major_),                      \
        .minor = (minor_),                      \
      } }                                       \
  }

/** @internal @This looks up an instance identifier resource entry and
    reads either fields. @tt start and @tt end pointers may be @tt
    NULL. @csee DEV_RES_ID */
ALWAYS_INLINE error_t device_res_get_id(const struct device_s *dev,
                                        uint_fast8_t id, uintptr_t *major, uintptr_t *minor)
{
  return device_res_get_uint(dev, DEV_RES_ID, id, major, minor);
}


/** @This adds a revision entry for the device. @csee DEV_RES_REVISION */
config_depend_alwaysinline(CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_revision(struct device_s *dev, uintptr_t major, uintptr_t minor),
{
  return device_res_alloc_uint(dev, DEV_RES_REVISION, major, minor, NULL);
})

/** @This specifies a revision resource entry in a static device
    resources table declaration. @csee DEV_RES_REVISION
    @see #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_REVISION(major_, minor_)                        \
  {                                                                     \
    .type = DEV_RES_REVISION,                                           \
      .u = { .revision = {                                              \
        .major = (major_),                                              \
        .minor = (minor_),                                              \
      } }                                                               \
  }

/** @This looks up a revision resource entry and reads either
    fields. @tt start and @tt end pointers may be @tt NULL.
    @csee DEV_RES_ID */
ALWAYS_INLINE error_t device_res_get_rev(const struct device_s *dev,
                                         uint_fast8_t id, uintptr_t *major, uintptr_t *minor)
{
  return device_res_get_uint(dev, DEV_RES_ID, id, major, minor);
}


/**  @This attaches a vendor identifier resource entry to the
     device. The string will be duplicated if not @tt NULL.
     @csee DEV_RES_VENDOR */
config_depend_alwaysinline(CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_vendor(struct device_s *dev, uintptr_t id, const char *name),
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc_str(dev, DEV_RES_VENDOR, NULL, name, &r);
  if (err)
    return err;

  r->u.vendor.id = id;
  return 0;
})

/** @This specifies a vendor id resource entry in a static device
    resources table declaration. @csee DEV_RES_VENDOR
    @see #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_VENDOR(id_, name_)      \
  {                                             \
    .type = DEV_RES_VENDOR,                     \
      .u = { .vendor = {                        \
        .id = (id_),                            \
        .name = (name_),                        \
      } }                                       \
  }


/**  @This attaches a product identifier resource entry to the
     device. The string will be duplicated if not @tt NULL.
     @csee DEV_RES_PRODUCT */
config_depend_alwaysinline(CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_product(struct device_s *dev, uintptr_t id, const char *name),
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc_str(dev, DEV_RES_PRODUCT, NULL, name, &r);
  if (err)
    return err;

  r->u.product.id = id;
  return 0;
})

/** @This specifies a product id resource entry in a static device
    resources table declaration. @csee DEV_RES_PRODUCT
    @see #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_PRODUCT(id_, name_)     \
  {                                             \
    .type = DEV_RES_PRODUCT,                    \
      .u = { .product = {                       \
        .id = (id_),                            \
        .name = (name_),                        \
      } }                                       \
  }


/** @This attaches a frequency resource to the device.
    @csee DEV_RES_FREQ */
config_depend_alwaysinline(CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_freq(struct device_s *dev,
                            const struct dev_freq_s *freq),
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc(dev, &r, DEV_RES_FREQ);
  if (err)
    return err;

  r->u.freq.num = freq->num;
  r->u.freq.denom = freq->denom;
  r->u.freq.acc_m = freq->acc_m;
  r->u.freq.acc_e = freq->acc_e;
  return 0;
})

/** @This specifies a frequency resource entry in a static device
    resources table declaration. A default value is used for frequency
    accuracy.  @csee DEV_RES_FREQ @see #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_FREQ(num_, denom_)      \
  {                                             \
    .type = DEV_RES_FREQ,                       \
      .u = { .freq = {                          \
        .num = (num_),                          \
        .denom = (denom_),                      \
        .acc_m = 7,                             \
        .acc_e = 31,                            \
      } }                                       \
  }

/** @This is similar to @cref #DEV_STATIC_RES_FREQ, with a specified
    value for the accuracy. @see dev_freq_s */
# define DEV_STATIC_RES_FREQ_ACC(num_, denom_, _acc_m, _acc_e)      \
  {                                             \
    .type = DEV_RES_FREQ,                       \
      .u = { .freq = {                          \
        .num = (num_),                          \
        .denom = (denom_),                      \
        .acc_m = (_acc_m),                      \
        .acc_e = (_acc_e),                      \
      } }                                       \
  }

/** @This looks up a frequency resource entry.
    @csee DEV_RES_FREQ */
ALWAYS_INLINE error_t device_get_res_freq(const struct device_s *dev,
                                          struct dev_freq_s *freq,
                                          uint_fast8_t index)
{
  struct dev_resource_s *r;

  r = device_res_get(dev, DEV_RES_FREQ, index);
  if (r == NULL)
    return -ENOENT;

  freq->num = r->u.freq.num;
  freq->denom = r->u.freq.denom;
  freq->acc_m = r->u.freq.acc_m;
  freq->acc_e = r->u.freq.acc_e;

  return 0;
}


/** @This attaches a generic string parameter resource to the
    device. @csee DEV_RES_STR_PARAM */
config_depend_alwaysinline(CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_str_param(struct device_s *dev, const char *name, const char *value),
{
  return device_res_alloc_str(dev, DEV_RES_STR_PARAM, name, value, NULL);
})

/** @This specifies a generic string parameter resource in a static
    device resources table declaration. @csee DEV_RES_STR_PARAM @see
    #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_STR_PARAM(name_, value_)        \
  {                                                     \
    .type = DEV_RES_STR_PARAM,                          \
      .u = { .str_param = {                             \
        .name = (name_),                                \
        .value = (value_),                              \
      } }                                               \
  }

/** @This retrieves the value of a generic string parameter resource
    entry from the associated parameter name. @csee DEV_RES_STR_PARAM */
ALWAYS_INLINE error_t device_get_param_str(const struct device_s *dev,
                                           const char *name, const char **a)
{
  struct dev_resource_s *r;

  if (!(r = device_res_get_from_name(dev, DEV_RES_STR_PARAM, 0, name)))
    return -ENOENT;

  if (a)
    *a = (const char*)r->u.str_param.value;
  return 0;
}

/** @This is similar to @cref device_get_param_str. A default value is
    returned instead of an error when the entry is not found. */
ALWAYS_INLINE void device_get_param_str_default(const struct device_s *dev, const char *name,
                                                const char **a, const char *def)
{
  struct dev_resource_s *r;

  if (!(r = device_res_get_from_name(dev, DEV_RES_STR_PARAM, 0, name)))
    *a = def;
  else
    *a = (const char*)r->u.str_param.value;
}

/** @This specifies a generic blob parameter resource in a static
    device resources table declaration. @csee DEV_RES_BLOB_PARAM @see
    #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_BLOB_PARAM(name_, value_)       \
  {                                                     \
  .type = DEV_RES_BLOB_PARAM,                           \
  .u = { .blob_param = {                                \
  .name = (name_),                                      \
  .value = (value_),                                    \
  } }                                                   \
  }

/** @This retrieves the value of a blob parameter resource entry from
    the associated parameter name. @csee DEV_RES_BLOB_PARAM */
ALWAYS_INLINE error_t device_get_param_blob(const struct device_s *dev,
                                            const char *name, uint8_t id,
                                            const void **a)
{
  struct dev_resource_s *r;

  if (!(r = device_res_get_from_name(dev, DEV_RES_BLOB_PARAM, id, name)))
    return -ENOENT;

  if (a)
    *a = (const void*)r->u.blob_param.value;
  return 0;
}


/** @This attaches an integer parameter resource to the device. The
    name string will be duplicated. @csee DEV_RES_UINT_PARAM */
config_depend_alwaysinline(CONFIG_DEVICE_RESOURCE_ALLOC,
error_t device_res_add_uint_param(struct device_s *dev, const char *name, uintptr_t value),
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc_str(dev, DEV_RES_UINT_PARAM, name, NULL, &r);
  if (err)
    return err;

  r->u.uint_param.value = value;
  return 0;
})

/** @This specifies a generic integer parameter resource in a static
    device resources table declaration. @csee DEV_RES_UINT_PARAM @see
    #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_UINT_PARAM(name_, value_)       \
  {                                                     \
    .type = DEV_RES_UINT_PARAM,                         \
      .u = { .uint_param = {                            \
        .name = (name_),                                \
        .value = (value_),                              \
      } }                                               \
  }

/** @This retrieves the value of a integer parameter resource entry
    from the associated parameter name. @csee DEV_RES_UINT_PARAM */
ALWAYS_INLINE error_t device_get_param_uint(const struct device_s *dev,
                                            const char *name, uintptr_t *a)
{
  struct dev_resource_s *r;

  if (!(r = device_res_get_from_name(dev, DEV_RES_UINT_PARAM, 0, name)))
    return -ENOENT;

  if (a)
    *a = r->u.uint_param.value;
  return 0;
}

/** @This is similar to @cref device_get_param_uint. A default value is
    returned instead of an error when the entry is not found. */
ALWAYS_INLINE void device_get_param_uint_default(const struct device_s *dev, const char *name,
                                                 uintptr_t *a, uintptr_t def)
{
  struct dev_resource_s *r;

  if (!(r = device_res_get_from_name(dev, DEV_RES_UINT_PARAM, 0, name)))
    *a = def;
  else
    *a = r->u.uint_param.value;
}


/** @This attaches a device path parameter resource to the device.
    @csee DEV_RES_DEV_PARAM */
config_depend(CONFIG_DEVICE_RESOURCE_ALLOC)
error_t device_res_add_dev_param(struct device_s *dev, const char *name,
                                 const char *path, enum driver_class_e cl);

/** @This specifies a named device dependency resource in a static
    device resources table declaration. The requested class is @ref
    DRIVER_CLASS_NONE. @csee #DEV_STATIC_RES_DEVCLASS_PARAM */
# define DEV_STATIC_RES_DEV_PARAM(name_, path_)         \
  {                                                     \
    .type = DEV_RES_DEV_PARAM,                          \
      .u = { .dev_param = {                             \
        .name = (name_),                                \
        .dev = (path_),                                 \
        .class_ = DRIVER_CLASS_NONE,                    \
      } }                                               \
  }

/** @This specifies a named device dependency resource in a static
    device resources table declaration. @csee DEV_RES_DEV_PARAM @see
    #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_DEVCLASS_PARAM(name_, path_, class__) \
  {                                                     \
    .type = DEV_RES_DEV_PARAM,                          \
      .u = { .dev_param = {                             \
        .name = (name_),                                \
        .dev = (path_),                                 \
        .class_ = (class__),                            \
      } }                                               \
  }

/** @This initializes a device accessor object from a device path
    parameter resource of the device tree. This actually perform the
    resource lookup from the provided name and call the @ref
    device_get_accessor_by_path function with the device path
    specified in the device resource.  @csee DEV_RES_DEV_PARAM */
error_t device_get_param_dev_accessor(struct device_s *dev,
                                      const char *name,
                                      struct device_accessor_s *acc,
                                      enum driver_class_e cl);

/** @This specifies a generic integer array parameter resource in a
    static device resources table declaration. @csee
    DEV_RES_UINT_ARRAY_PARAM @see #DEV_DECLARE_STATIC */
# define DEV_STATIC_RES_UINT_ARRAY_PARAM(name_, ...)         \
  {                                                          \
    .type = DEV_RES_UINT_ARRAY_PARAM,                        \
      .u = { .uint_array_param = {                           \
        .name = (name_),                                     \
        .count = ARRAY_SIZE(((const uintptr_t[]){ __VA_ARGS__ })),      \
        .array = (const uintptr_t[]){ __VA_ARGS__ },         \
      } }                                                    \
  }

/** @This attaches an integer array parameter resource to the
    device. The name string will be duplicated. @csee
    DEV_RES_UINT_ARRAY_PARAM */
config_depend(CONFIG_DEVICE_RESOURCE_ALLOC)
error_t device_res_add_uint_array_param(struct device_s *dev, const char *name,
                                        uint16_t count, uintptr_t values[]);

/** @This retrieves the value of a integer parameter resource entry
    from the associated parameter name. @csee DEV_RES_UINT_ARRAY_PARAM */
ALWAYS_INLINE error_t device_get_param_uint_array(const struct device_s *dev,
                                                  const char *name, uint16_t *count, const uintptr_t **a)
{
  struct dev_resource_s *r;

  if (!(r = device_res_get_from_name(dev, DEV_RES_UINT_ARRAY_PARAM, 0, name)))
    return -ENOENT;

  *count = r->u.uint_array_param.count;
  *a = r->u.uint_array_param.array;
  return 0;
}


#endif
