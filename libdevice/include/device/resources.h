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
 * @file
 * @module{Devices support library}
 * @short Device resources
 */

#include <assert.h>
#include <hexo/types.h>
#include <hexo/decls.h>

#include <device/types.h>

enum driver_class_e;

/** @This specifies the types of device resource entries. */
enum dev_resource_type_e
{
    DEV_RES_UNUSED = 0,
    DEV_RES_MEM,
    DEV_RES_IO,
    DEV_RES_IRQ,
    DEV_RES_GPIO,
    DEV_RES_IOMUX,
    DEV_RES_UART,
    DEV_RES_ID,
    DEV_RES_VENDOR,
    DEV_RES_PRODUCT,
    DEV_RES_REVISION,
    DEV_RES_FREQ,
    DEV_RES_CLOCK_RTE,
    DEV_RES_CLOCK_OSC,
    DEV_RES_CLOCK_SRC,
    DEV_RES_STR_PARAM,
    DEV_RES_BLOB_PARAM,
    DEV_RES_UINT_PARAM,
    DEV_RES_DEV_PARAM,
    DEV_RES_UINT_ARRAY_PARAM,
    DEV_RES_I2C_ADDR,
    DEV_RES_DMA,

    DEV_RES_TYPES_COUNT,              //< Number of resource types
};

enum dev_resource_flags_e
{
  /** first resource field is a pointer which must be freeed on cleanup */
  DEVICE_RES_FLAGS_FREE_PTR0 = 1,
  /** second resource field is a pointer which must be freeed on cleanup */
  DEVICE_RES_FLAGS_FREE_PTR1 = 2,
  /** first resource field is a path string to an other device which
      must be initialized before initialization of this device. */
  DEVICE_RES_FLAGS_DEPEND = 4,
  /** reserved for use by enumerator driver */
  DEVICE_RES_FLAGS_ENUM_RESERVED0 = 8,
};

struct dev_resource_s
{
  /** resource descriptor type @see dev_resource_type_e */
  enum dev_resource_type_e type:8;
  enum dev_resource_flags_e flags:8;

  union {
    /** @internal */
    uintptr_t                   uint[2];
    /** @internal */
    uint64_t                    uint64;

    /** @see #DEV_STATIC_RES_MEM @see device_res_add_mem */
    struct {
      uintptr_t                 start;
      uintptr_t                 end;
    }                           mem;

    /** @see #DEV_STATIC_RES_IO @see device_res_add_io */
    struct {
      uintptr_t                 start;
      uintptr_t                 end;
    }                           io;

    /** @see #DEV_STATIC_RES_IRQ @see device_res_add_irq */
    struct {
      const char                *icu;
      /** id of physical irq link going out of the device */
      uintptr_t                 dev_out_id:CONFIG_DEVICE_IRQ_MAX_OUTPUT_ID;
      /** id of physical irq link going into the interrupt controller */
      uintptr_t                 icu_in_id:CONFIG_DEVICE_IRQ_MAX_INPUT_ID;
      /** logical irq id, used when multiple irqs are multiplexed on one physical link */
      uintptr_t                 irq_id:CONFIG_DEVICE_IRQ_MAX_LOGICAL_ID;
      /** device tree path to interrupt controller, relative to device */
    }                           irq;

    /** @see #DEV_STATIC_RES_GPIO @see device_res_add_gpio */
    struct {
      uintptr_t                 id:CONFIG_DEVICE_GPIO_MAX_ID;
      uintptr_t                 width:CONFIG_DEVICE_GPIO_MAX_WIDTH;
      const char                *label;
    }                           gpio;

    /** @see #DEV_STATIC_RES_IOMUX @see device_res_add_iomux */
    struct {
      uintptr_t                 demux:CONFIG_DEVICE_IOMUX_MAX_DEMUX;
      uintptr_t                 io_id:CONFIG_DEVICE_IOMUX_MAX_ID;
      uintptr_t                 mux:CONFIG_DEVICE_IOMUX_MAX_MUX;
      uintptr_t                 config:CONFIG_DEVICE_IOMUX_MAX_CONFIG;
      const char                *label;
    }                           iomux;

    /** @see #DEV_STATIC_RES_DMA @see device_res_add_dma */
    struct {
      const char                *label;
      uintptr_t                 channel:5;
      uintptr_t                 config;
    }                           dma;
    /** @see #DEV_STATIC_RES_UART @see device_res_add_uart */
    struct {
      uintptr_t                 baudrate:26;
      uintptr_t                 data_bits:4;
      uintptr_t                 stop_bits:2;
      uintptr_t                 parity:2;
      uintptr_t                 flow_ctrl:1;
      uintptr_t                 half_duplex:1;
    }                           uart;

    /** @see #DEV_STATIC_RES_ID @see device_res_add_id */
    struct {
      uintptr_t                 major;          //< dynamic numeric id
      uintptr_t                 minor;          //< dynamic numeric id
    }                           id;

    /** @see #DEV_STATIC_RES_REVISION @see device_res_add_revision */
    struct {
      uintptr_t                 major;
      uintptr_t                 minor;
    }                           revision;

    /** @see #DEV_STATIC_RES_VENDOR @see device_res_add_vendor */
    struct {
      /** optional vendor numeric id, may be -1 */
      uintptr_t                 id;
      /** optional vendor string id, may be NULL */
      const char                *name;
    }                           vendor;

    /** @see #DEV_STATIC_RES_PRODUCT @see device_res_add_product */
    struct {
      /** optional device numeric id, may be -1 */
      uintptr_t                 id;
      /** optional device string, may be NULL */
      const char                *name;
    }                           product;

    /** @see #DEV_STATIC_RES_CLK_RTE @see device_add_res_clock_route */
    struct {
      /** node id of the input clock signal */
      uint64_t                  parent:CONFIG_DEVICE_CLOCK_MAX_ID;
      /** node id of the output clock signal */
      uint64_t                  node:CONFIG_DEVICE_CLOCK_MAX_ID;
      /** numerator of the frequency scaling */
      uint64_t                  num:CONFIG_DEVICE_CLOCK_FRAC_WIDTH;
      /** denominator of the frequency scaling */
      uint64_t                  denom:CONFIG_DEVICE_CLOCK_FRAC_WIDTH;
      /** mask of associated configurations */
      uint64_t                  config:CONFIG_DEVICE_CLOCK_MAX_CONFIG;
    }                           clock_rte;

    /** @see #DEV_STATIC_RES_CLK_OSC @see device_add_res_clock_osc */
    struct {
      /** node id */
      uint64_t                  node:CONFIG_DEVICE_CLOCK_MAX_ID;
      /** numerator of the frequency fractional part */
      uint64_t                  num:CONFIG_DEVICE_CLOCK_OSCN_WIDTH;
      /** denominator of the frequency fractional part */
      uint64_t                  denom:CONFIG_DEVICE_CLOCK_OSCD_WIDTH;
      /** mask of associated configurations */
      uint64_t                  config:CONFIG_DEVICE_CLOCK_MAX_CONFIG;
    }                           clock_osc;

    /** @see #DEV_STATIC_RES_CLK_SRC @see device_add_res_clock_src */
    struct {
      /** path to the clock source device */
      const char                *src;
      /** node id of the source end-point, relevant to the device
          pointed to by @ref src. */
      uintptr_t                 src_ep:CONFIG_DEVICE_CLOCK_MAX_ID;
      /** node id of the sink end-point, relevant to the device which
          holds this resource. */
      uintptr_t                 sink_ep:CONFIG_DEVICE_CLOCK_MAX_ID;
    }                           clock_src;

    /** @see #DEV_STATIC_RES_FREQ @see device_add_res_freq */
    struct {
      /** numerator of the frequency fractional part */
      uint64_t                  num:64-CONFIG_DEVICE_CLOCK_OSCD_WIDTH;
      /** denominator of the frequency fractional part */
      uint64_t                  denom:CONFIG_DEVICE_CLOCK_OSCD_WIDTH;
    }                           freq;

    /** @see #DEV_STATIC_RES_STR_PARAM @see device_res_add_str_param */
    struct {
      const char                *value;
      const char                *name;
    }                           str_param;

    /** @see #DEV_STATIC_RES_BLOB_PARAM */
    struct {
      const void                *value;
      const char                *name;
    }                           blob_param;

    /** @see #DEV_STATIC_RES_UINT_PARAM @see device_res_add_uint_param */
    struct {
      uintptr_t                 value;
      const char                *name;
    }                           uint_param;

    /** @see #DEV_STATIC_RES_DEV_PARAM @see device_res_add_dev_param */
    struct {
      const char                *dev;
      const char                *name;
    }                           dev_param;

    /** @see device_res_add_uint_array_param */
    struct {
      uintptr_t                 *value;
      const char                *name;
    }                           uint_array_param;

  }                             u;
};

enum dev_resource_table_flags_e
{
  DEVICE_RES_TBL_FLAGS_ALLOCATED = 1,
  DEVICE_RES_TBL_FLAGS_STATIC_CONST = 2,
};

struct dev_resource_table_s
{
  struct dev_resource_table_s     *next;
  enum dev_resource_table_flags_e flags;
  uint_fast8_t                    count;
  struct dev_resource_s           table[0];
};

#define DEV_STATIC_RESOURCES_ARRAY(args_...)                    \
  ((struct dev_resource_s[]){                          \
    args_                                                       \
  })

/** @This yields a static pointer to an static initialized @ref
    dev_resource_s object.

    @see #DEV_DECLARE_STATIC
*/
#define DEV_STATIC_RESOURCES(args_...)                          \
  (const struct dev_resource_table_s *)&(const struct {         \
      struct dev_resource_table_s _table;                       \
      struct dev_resource_s _entries[ARRAY_SIZE(DEV_STATIC_RESOURCES_ARRAY(args_))]; \
  }){                                                           \
    ._table = {                                                 \
      .next = NULL,                                             \
      .flags = DEVICE_RES_TBL_FLAGS_STATIC_CONST,               \
      .count = ARRAY_SIZE(DEV_STATIC_RESOURCES_ARRAY(args_)),   \
    },                                                          \
    ._entries = {                                               \
      args_                                                     \
    }                                                           \
  }

/** @This iterates over resources entries of a device. */
#define DEVICE_RES_FOREACH(dev, rvar, ... /* loop body */ )             \
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

/********************************************************************************/

/** @internal @This returns a poiner to next unused resource slot. */
error_t device_res_alloc(struct device_s *dev, struct dev_resource_s **res,
			 enum dev_resource_type_e type);

/** @internal @This releases the resource values and mark the resource
    entry as @ref DEV_RES_UNUSED. */
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
    integer or pointers fields. */
ALWAYS_INLINE error_t device_res_alloc_uint(struct device_s *dev,
					    enum dev_resource_type_e type,
					    uintptr_t a, uintptr_t b, struct dev_resource_s **r_)
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
}

/** @internal @This looks up a resource entry and reads one or two
    integer fields. @tt a and @tt b pointers may be @tt NULL. */
ALWAYS_INLINE error_t device_res_get_uint(const struct device_s *dev,
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
ALWAYS_INLINE error_t device_res_alloc_uint64(struct device_s *dev,
					      enum dev_resource_type_e type,
					      uint64_t a, struct dev_resource_s **r_)
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc(dev, &r, type);
  if (err)
    return err;

  r->u.uint64 = a;

  if (r_)
    *r_ = r;
  return 0;
}

/** @internal @This looks up a resource entry and reads a 64 bits
    integer resource field. */
ALWAYS_INLINE error_t device_res_get_uint64(const struct device_s *dev,
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
error_t device_res_alloc_str(struct device_s *dev,
			     enum dev_resource_type_e type,
			     const char *a, const char *b,
			     struct dev_resource_s **r_);

/********************************************************************************/


/** @This adds an IO space address range to the device resources list.
    @see #DEV_STATIC_RES_IO */
ALWAYS_INLINE error_t device_res_add_io(struct device_s *dev, uintptr_t start, uintptr_t end)
{
  assert(start < end);
  return device_res_alloc_uint(dev, DEV_RES_IO, start, end, NULL);
}

/** @This can be used to include a memory range entry in a static
    device resources table declaration. @see device_res_add_mem
    @see #DEV_DECLARE_STATIC_RESOURCES */
#define DEV_STATIC_RES_IO(start_, end_)         \
  {                                             \
    .type = DEV_RES_IO,                         \
      .u = { .io = {                            \
        .start = (start_),                      \
        .end = (end_),                          \
      } }                                       \
  }


/** @This adds an memory space address range to the device resources list.
    @see #DEV_STATIC_RES_MEM */
ALWAYS_INLINE error_t device_res_add_mem(struct device_s *dev, uintptr_t start, uintptr_t end)
{
  assert(start < end);
  return device_res_alloc_uint(dev, DEV_RES_MEM, start, end, NULL);
}

/** @This can be used to include a memory range entry in a static
    device resources table declaration. @see device_res_add_mem
    @see #DEV_DECLARE_STATIC_RESOURCES */
#define DEV_STATIC_RES_MEM(start_, end_)        \
  {                                             \
    .type = DEV_RES_MEM,                        \
      .u = { .mem = {                           \
        .start = (start_),                      \
        .end = (end_),                          \
      } }                                       \
  }


/** @This adds an IRQ binding to the device resources list.

    The entry specifies how the physical output irq link of the device
    must be connected to the specified input link of the given
    interrupt controller and gives the logical irq number associated
    to the device. The logical irq number is used when the link is a
    bus carrying an interrupt identifier; zero is used if the link is a
    single wire.

    A device tree path to the interrupt controller relative to @tt dev
    is expected for the @tt icu parameter. The path string will be
    duplicated.

    @see #DEV_STATIC_RES_IRQ
*/
ALWAYS_INLINE error_t device_res_add_irq(struct device_s *dev, uint_fast8_t dev_out_id,
					 uint_fast8_t icu_in_id, uint_fast16_t irq_id,
					 const char *icu)
{
#ifdef CONFIG_DEVICE_IRQ
  struct dev_resource_s *r;
  error_t err = device_res_alloc_str(dev, DEV_RES_IRQ, icu, NULL, &r);
  if (err)
    return err;

  r->flags |= DEVICE_RES_FLAGS_DEPEND;

  r->u.irq.dev_out_id = dev_out_id;
  r->u.irq.icu_in_id = icu_in_id;
  r->u.irq.irq_id = irq_id;

  return 0;
#else
  return -EINVAL;
#endif
}

#ifdef CONFIG_DEVICE_IRQ
/** @This can be used to include an irq resource entry in a static
    device resources table declaration. The icu path name must be a static
    string. @see device_res_add_irq @see #DEV_DECLARE_STATIC_RESOURCES */
# define DEV_STATIC_RES_IRQ(dev_out_id_, icu_in_id_, irq_id_, icu_)     \
  {                                                                     \
    .flags = DEVICE_RES_FLAGS_DEPEND,                                   \
      .type = DEV_RES_IRQ,                                              \
      .u = { .irq = {                                                   \
        .icu = (icu_),                                                  \
        .dev_out_id = (dev_out_id_),                                    \
        .icu_in_id = (icu_in_id_),                                      \
        .irq_id = (irq_id_),                                            \
      } }                                                               \
  }
#else
# define DEV_STATIC_RES_IRQ(dev_out_id_, icu_in_id_, irq_id_, icu_)     \
  {                                                                     \
    .type = DEV_RES_UNUSED,                                             \
  }
#endif


/**
   @This adds a numerical identifier which uniquely identify an
   instance of the device. This is generally used by device which are
   commonly referred to by using a number. Processor devices must
   use this resource for the cpu id. @see #DEV_STATIC_RES_ID
*/
ALWAYS_INLINE error_t device_res_add_id(struct device_s *dev, uintptr_t major, uintptr_t minor)
{
  return device_res_alloc_uint(dev, DEV_RES_ID, major, minor, NULL);
}

/** @This can be used to include a numerical indentifier resource
    entry in a static device resources table declaration. @see
    #DEV_DECLARE_STATIC_RESOURCES @see device_res_add_id */
# define DEV_STATIC_RES_ID(major_, minor_)      \
  {                                             \
    .type = DEV_RES_ID,                         \
      .u = { .id = {                            \
        .major = (major_),                      \
        .minor = (minor_),                      \
      } }                                       \
  }


/**
   @This adds a revision information for the device.
*/
ALWAYS_INLINE error_t device_res_add_revision(struct device_s *dev, uintptr_t major, uintptr_t minor)
{
  return device_res_alloc_uint(dev, DEV_RES_REVISION, major, minor, NULL);
}

/** @This can be used to include a revision resource
    entry in a static device resources table declaration. @see
    #DEV_DECLARE_STATIC_RESOURCES @see device_res_add_revision */
# define DEV_STATIC_RES_REVISION(major_, minor_)                        \
  {                                                                     \
    .type = DEV_RES_REVISION,                                           \
      .u = { .revision = {                                              \
        .major = (major_),                                              \
        .minor = (minor_),                                              \
      } }                                                               \
  }


/**
   @This attaches a vendor identifier resource to the device. Both a
   numerical and a string values can be specified. The exact meaning
   of the value depends on the parent enumerator device. The string
   will be duplicated if not @tt NULL.
*/
ALWAYS_INLINE error_t device_res_add_vendor(struct device_s *dev, uintptr_t id, const char *name)
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc_str(dev, DEV_RES_VENDOR, NULL, name, &r);
  if (err)
    return err;

  r->u.vendor.id = id;
  return 0;
}

/** @This can be used to include a vendor id resource entry in a
    static device resources table declaration. When not @tt NULL, the
    name must be a static string. @see #DEV_DECLARE_STATIC_RESOURCES
    @see device_res_add_vendor */
# define DEV_STATIC_RES_VENDOR(id_, name_)      \
  {                                             \
    .type = DEV_RES_VENDOR,                     \
      .u = { .vendor = {                        \
        .id = (id_),                            \
        .name = (name_),                        \
      } }                                       \
  }


/**
   @This attaches a product identifier resource to the device. Both a
   numerical and a string values can be specified. The exact meaning
   of the value depends on the parent enumerator device.  The string
   will be duplicated if not @tt NULL.
*/
ALWAYS_INLINE error_t device_res_add_product(struct device_s *dev, uintptr_t id, const char *name)
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc_str(dev, DEV_RES_PRODUCT, NULL, name, &r);
  if (err)
    return err;

  r->u.product.id = id;
  return 0;
}

/** @This can be used to include a product id resource entry in a
    static device resources table declaration. When not @tt NULL, the
    name must be a static string. @see #DEV_DECLARE_STATIC_RESOURCES
    @see device_res_add_product */
# define DEV_STATIC_RES_PRODUCT(id_, name_)     \
  {                                             \
    .type = DEV_RES_PRODUCT,                    \
      .u = { .product = {                       \
        .id = (id_),                            \
        .name = (name_),                        \
      } }                                       \
  }


/** @This attaches a frequency resource to the device. */
ALWAYS_INLINE error_t device_res_add_freq(struct device_s *dev,
                                          const struct dev_freq_s *freq)
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc(dev, &r, DEV_RES_FREQ);
  if (err)
    return err;

  r->u.freq.num = freq->num;
  r->u.freq.denom = freq->denom;
  return 0;
}

/** @see #DEV_DECLARE_STATIC_RESOURCES @see device_res_add_freq */
# define DEV_STATIC_RES_FREQ(num_, denom_)      \
  {                                             \
    .type = DEV_RES_FREQ,                       \
      .u = { .freq = {                          \
        .num = (num_),                          \
        .denom = (denom_),                      \
      } }                                       \
  }

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

  return 0;
}


/** @This attaches a string parameter resource to the device. The
    exact meaning of the value is driver dependent. */
ALWAYS_INLINE error_t device_res_add_str_param(struct device_s *dev, const char *name, const char *value)
{
  return device_res_alloc_str(dev, DEV_RES_STR_PARAM, value, name, NULL);
}

# define DEV_STATIC_RES_STR_PARAM(name_, value_)        \
  {                                                     \
    .type = DEV_RES_STR_PARAM,                          \
      .u = { .str_param = {                             \
        .name = (name_),                                \
        .value = (value_),                              \
      } }                                               \
  }

/** @This retrieves the value of a string parameter resource entry
    from the associated parameter name. */
ALWAYS_INLINE error_t device_get_param_str(const struct device_s *dev,
                                           const char *name, const char **a)
{
  struct dev_resource_s *r;

  if (!(r = device_res_get_from_name(dev, DEV_RES_STR_PARAM, 0, name)))
    return -ENOENT;

  if (a)
    *a = (const char*)r->u.uint[0];
  return 0;
}

/** @This retrieves the value of a string parameter resource entry
    from the associated parameter name. A default value is returned
    when not found. */
ALWAYS_INLINE void device_get_param_str_default(const struct device_s *dev, const char *name,
                                                const char **a, const char *def)
{
  struct dev_resource_s *r;

  if (!(r = device_res_get_from_name(dev, DEV_RES_STR_PARAM, 0, name)))
    *a = def;
  else
    *a = (const char*)r->u.uint[0];
}


# define DEV_STATIC_RES_BLOB_PARAM(name_, value_)       \
  {                                                     \
  .type = DEV_RES_BLOB_PARAM,                           \
  .u = { .blob_param = {                                \
  .name = (name_),                                      \
  .value = (value_),                                    \
  } }                                                   \
  }

/** @This retrieves the value of a blob parameter resource entry from
    the associated parameter name. */
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
    exact meaning of the value is driver dependent. The name string
    will be duplicated. */
ALWAYS_INLINE error_t device_res_add_uint_param(struct device_s *dev, const char *name, uintptr_t value)
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc_str(dev, DEV_RES_UINT_PARAM, NULL, name, &r);
  if (err)
    return err;

  r->u.uint_param.value = value;
  return 0;
}

# define DEV_STATIC_RES_UINT_PARAM(name_, value_)       \
  {                                                     \
    .type = DEV_RES_UINT_PARAM,                         \
      .u = { .uint_param = {                            \
        .name = (name_),                                \
        .value = (value_),                              \
      } }                                               \
  }

/** @This retrieves the value of a integer parameter resource entry
    from the associated parameter name. */
ALWAYS_INLINE error_t device_get_param_uint(const struct device_s *dev,
                                            const char *name, uintptr_t *a)
{
  struct dev_resource_s *r;

  if (!(r = device_res_get_from_name(dev, DEV_RES_UINT_PARAM, 0, name)))
    return -ENOENT;

  if (a)
    *a = r->u.uint[0];
  return 0;
}

/** @This retrieves the value of an integer parameter resource entry
    from the associated parameter name. A default value is returned
    when not found. */
ALWAYS_INLINE void device_get_param_uint_default(const struct device_s *dev, const char *name,
                                                 uintptr_t *a, uintptr_t def)
{
  struct dev_resource_s *r;

  if (!(r = device_res_get_from_name(dev, DEV_RES_UINT_PARAM, 0, name)))
    *a = def;
  else
    *a = r->u.uint[0];
}


/** @This attaches a device path parameter resource to the device. The
    exact meaning of the value is driver dependent. The driver
    initialization will not take place until the device path points to
    an exisiting and properly initialized device. */
ALWAYS_INLINE error_t device_res_add_dev_param(struct device_s *dev, const char *name, const char *path)
{
  struct dev_resource_s *r;
  error_t err = device_res_alloc_str(dev, DEV_RES_DEV_PARAM, path, name, &r);
  if (err)
    return err;

  r->flags |= DEVICE_RES_FLAGS_DEPEND;

  return 0;
}

# define DEV_STATIC_RES_DEV_PARAM(name_, path_)          \
  {                                                     \
    .flags = DEVICE_RES_FLAGS_DEPEND,                   \
    .type = DEV_RES_DEV_PARAM,                          \
      .u = { .dev_param = {                             \
        .name = (name_),                                \
        .dev = (path_),                                 \
      } }                                               \
  }

/** @This initializes a device accessor object from a device path
    parameter resource of the device tree.
    @see device_get_accessor_by_path
 */
error_t device_get_param_dev_accessor(struct device_s *dev,
                                      const char *name, void *accessor,
                                      enum driver_class_e cl);


/** @This attaches an integer array parameter resource to the
    device. The first value of the array must indicate the number of
    subsequent entries in the array. The exact meaning of the value is
    driver dependent.

    The name string and the array will be duplicated. */
error_t device_res_add_uint_array_param(struct device_s *dev, const char *name, uintptr_t *value);

#endif
