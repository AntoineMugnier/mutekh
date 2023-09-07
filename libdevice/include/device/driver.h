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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

/**
 * @file
 * @module {Core::Devices support library}
 * @short Driver structures and driver API classes
 */

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/enum.h>
#include <device/device.h>

ENUM_DESCRIPTOR(driver_class_e, strip:DRIVER_CLASS_, upper);

/** @This specifies identifiers of device driver API classes.
    @xsee {Device classes} */
enum driver_class_e
{
  DRIVER_CLASS_NONE = 0,
  DRIVER_CLASS_BLOCK,
  DRIVER_CLASS_CHAR,
  DRIVER_CLASS_ENUM,
  DRIVER_CLASS_FB,
  /** Interrupt Controller Unit */
  DRIVER_CLASS_ICU,
  DRIVER_CLASS_DMA,
  DRIVER_CLASS_INPUT,
  DRIVER_CLASS_NET,
  DRIVER_CLASS_PCM,
  DRIVER_CLASS_TIMER,
  DRIVER_CLASS_PWM,
  DRIVER_CLASS_SPI_CTRL,
  DRIVER_CLASS_SPI_SLAVE,
  DRIVER_CLASS_LCD,
  /** Clock Management Unit */
  DRIVER_CLASS_CMU,
  DRIVER_CLASS_GPIO,
  DRIVER_CLASS_IOMUX,
  DRIVER_CLASS_I2C_CTRL,
  DRIVER_CLASS_I2C_SLAVE,
  DRIVER_CLASS_MEM,
  DRIVER_CLASS_RFPACKET,
  DRIVER_CLASS_CRYPTO,
  DRIVER_CLASS_CPU,
  DRIVER_CLASS_VALIO,
  DRIVER_CLASS_USBDEV,
  DRIVER_CLASS_DISPLAY,
  DRIVER_CLASS_SMI,
  DRIVER_CLASS_PHY,
  DRIVER_CLASS_BITBANG,
  DRIVER_CLASS_ONEWIRE,

  /* Custom driver class IDs should be registered in @ref
      #CONFIG_DEVICE_CUSTOM_CLASS_COUNT enum config token.
   */
};

ENUM_DESCRIPTOR(dev_enum_type_e, strip:DEV_ENUM_TYPE_, upper);

/** @This specifies types of enumeration data
    @see dev_enum_ident_s */
enum dev_enum_type_e
{
  DEV_ENUM_TYPE_INVALID,
  DEV_ENUM_TYPE_GENERIC,
  DEV_ENUM_TYPE_PCI,
  DEV_ENUM_TYPE_ISA,
  DEV_ENUM_TYPE_ATA,
  DEV_ENUM_TYPE_FDTNAME,
  DEV_ENUM_TYPE_GAISLER,
};

/** device structure identification informations. wildcard values are
    enum driver dependent */
struct dev_enum_ident_s
{
  enum dev_enum_type_e type;

  union {
    struct {
      uint16_t vendor;
      uint16_t device;
      uint16_t rev_minor;
      uint16_t rev_major;
    } generic;
    struct {
      uint16_t vendor;
      uint16_t device;
      uint32_t class;
    } pci;
    struct {
      uint16_t vendor;
      uint16_t device;
    } grlib;
    struct {
      uint16_t vendor;
    } isa;
    struct {
      const char *name;
    } fdtname;
    struct {
      const char *str;
    } ata;
  };
};


/**
   Shortcut for creating a PCI entry in a static @cref dev_enum_ident_s
   array.

   @param _vendor the vendor id to match, -1 for wildcard
   @param _device the device id to match, -1 for wildcard
   @param _class the class to match, -1 for wildcard
 */
#define DEV_ENUM_PCI_ENTRY(_vendor, _device, _class)		\
	{ .type = DEV_ENUM_TYPE_PCI, { .pci = {				\
				.vendor = _vendor, .device = _device,	\
				.class = _class } } }

/**
   Shortcut for creating an ISA entry in a static @cref dev_enum_ident_s
   array.

   @param _vendor the vendor id to match
 */
#define DEV_ENUM_ISA_ENTRY(_vendor)						\
	{ .type = DEV_ENUM_TYPE_PCI, { .isa = {				\
				.vendor = _vendor } } }

/**
   Shortcut for creating an ATA entry in a static @cref dev_enum_ident_s
   array.

   @param _str the string to match from the device
 */
#define DEV_ENUM_ATA_ENTRY(_str)							\
	{ .type = DEV_ENUM_TYPE_ATA, { .ata = {				\
				.str = _str } } }

/**
   Shortcut for creating a flat-device-tree entry in a static
   @cref dev_enum_ident_s array.

   @param _name The string to match from the device-tree
 */
#define DEV_ENUM_FDTNAME_ENTRY(_name)	\
	{ .type = DEV_ENUM_TYPE_FDTNAME, { .fdtname = {		\
				.name = _name } } }

/**
   Shortcut for creating a Gaisler entry in a static @cref dev_enum_ident_s
   array.

   @param _vendor the vendor id to match, -1 for wildcard
   @param _device the device id to match, -1 for wildcard
 */
#define DEV_ENUM_GAISLER_ENTRY(_vendor, _device)		\
	{ .type = DEV_ENUM_TYPE_GAISLER, { .grlib = {				\
				.vendor = _vendor, .device = _device } } }

/**
   Shortcut for creating a Generic with vendor/device ids and version
   number in a static @cref dev_enum_ident_s array.

   @param _vendor the vendor id to match, -1 for wildcard
   @param _device the device id to match, -1 for wildcard
   @param _rev_major the device major revision, -1 for wildcard
   @param _rev_minor the minimum supported minor revision, -1 for wildcard
 */
#define DEV_ENUM_GENERIC_ENTRY(_vendor, _device, _rev_major, _rev_minor)	\
  { .type = DEV_ENUM_TYPE_GENERIC, { .generic = {                        \
        .vendor = _vendor,                                              \
        .device = _device,                                              \
        .rev_minor = _rev_minor,                                        \
        .rev_major = _rev_major,                                        \
      } } }


/** @see dev_init_t */
#define DEV_INIT(n)	error_t (n) (struct device_s *dev, uint32_t cl_missing)

/**
   @This is the device driver initialization function type. This
   function allocates device private data and initialize the
   hardware. It will be called before using any other functions on the
   device.

   The device initialization status will be updated depending on the
   return value of this function:

   @list
     @item If 0 is returned, the device status changes to @ref
        DEVICE_INIT_DONE.

     @item The first call to this function might not be able to fully
       complete the initialization of the device. In this case, it
       should return @tt -EAGAIN to indicate that initialization will
       complete later. The device status will then be changed to @ref
       DEVICE_INIT_ONGOING. The initialization may complete either
       when the @ref device_async_init_done is called by the driver or
       when the dev_init_t function is called again. The later occurs
       when some progress are made on initialization of related
       devices and the @ref DRIVER_FLAGS_INIT_RETRY flag of the driver
       is set. The @ref device_init_is_partial function should be used
       to test for the initial invocation.

     @item If an error is returned and the driver has not invoked the
       @ref device_init_enable_api function, the status is changed
       to @ref DEVICE_INIT_FAILED. The driver must have released all
       resource associated to the device in this case as the @ref
       dev_cleanup_t function will not be called.

     @item If an error is returned but some classes have been
       reported as initialized, the status is changed to @ref
       DEVICE_INIT_PARTIAL.
   @end list

   The @ref DRIVER_FLAGS_NO_DEPEND flag can be used so that this
   function is called even if some resource dependencies are not
   satisfied. The driver is then responsible for testing missing
   dependencies. In this case the @tt cl_missing mask parameter
   indicates any driver classes related to a missing resource
   dependencies.

   The @ref DRIVER_FLAGS_EARLY_INIT flag can be used so that this
   function is called early during startup. Some kernel service can
   not be used in this case, especially, asynchronous initialization
   can not be used.

   @xsee {Device status}
*/
typedef DEV_INIT(dev_init_t);



/** @see dev_cleanup_t */
#define DEV_CLEANUP(n)	__attribute__((unused)) error_t    (n) (struct device_s *dev)

/**
   @This is the device cleanup function type. @This tries to release
   all ressources allocated by the @ref dev_init_t function.

   This function will only be called these conditions are met:
   @list
     @item the device status is either @ref DEVICE_INIT_DONE,
       @ref DEVICE_INIT_PARTIAL or @ref DEVICE_INIT_DECLINE.
     @item The @ref device_s::ref_count field is zero.
     @item The @ref device_s::start_count field is zero.
     @item The device has no children.
   @end list

   This function is called with the device lock held.
   @This function must return @tt -EBUSY if it is not possible to
   release the device yet.

   @This function may also return @tt -EAGAIN if cleanup has been
   initiated but requires completion of some asynchronous operations.
   The cleanup will complete when the @ref device_async_cleanup_done
   function is called from a deferred kroutine. This requires
   definition of the @ref #CONFIG_DEVICE_INIT_ASYNC token.
*/
typedef DEV_CLEANUP(dev_cleanup_t);

/** @This specifies device usage operations. @see dev_use_t */
enum dev_use_op_e
{
  /** Get and Put operations are used when the @ref
      device_get_accessor and @ref device_put_accessor functions are
      called. An error code may be returned on @ref
      DEV_USE_GET_ACCESSOR in order to prevent the caller from
      acquiring an accessor. The @tt param argument is a pointer to a
      @ref device_accessor_s object. */
  DEV_USE_GET_ACCESSOR,
  /* @see DEV_USE_GET_ACCESSOR */
  DEV_USE_PUT_ACCESSOR,

  /** This operation is used when @ref device_last_number is
      called. The @tt param argument is a pointer to a @ref
      device_accessor_s structure with the @tt dev and @tt api field
      properly initialized. The default implementation reports 0. */
  DEV_USE_LAST_NUMBER,

  /** Start and stop operations are invoked from the @ref device_start
      and @ref device_stop functions. An error code may be returned on
      @ref DEV_USE_START. The @tt param argument is a pointer to a
      @ref device_accessor_s object. The default implementation does
      nothing and always succeed.

      The driver may test if the value of @ref device_s::start_count
      is zero. This field is updated by the @ref device_start and @ref
      device_stop functions. The driver may also manage the lower bits
      of the value directly if they were reserved by providing a
      suitable value for the @ref #CONFIG_DEVICE_START_LOG2INC
      token. The driver may still maintain some other internal start
      counters if sub-devices do require separate counters. */
  DEV_USE_START,
  /** @see DEV_USE_START */
  DEV_USE_STOP,

#ifdef CONFIG_DEVICE_SLEEP
  /** This operation is only used when the driver has previously
      called the @ref device_sleep_schedule function. The device power
      must be reduced if the device is not currently in use. The @tt
      param argument is a pointer to a @ref device_s object.

      What is actually performed by the driver is device
      dependent. This may include releasing clock and power
      endpoints, calling @ref device_stop on associated spi or i2c
      bus controllers or enabling a device specific low power mode.

      When this operation is performed on multiple devices, driver
      initialization order is used to determine the call order. This
      ensure a bus controller is not disabled then re-enabled in order
      to put a dependent device in low power mode. */
  DEV_USE_SLEEP,
#endif

#ifdef CONFIG_DEVICE_CLOCK
  /** This operation is used when a clock or power enable operation
      requested by the driver has been satisfied. The @tt param
      argument is a pointer to the sink endpoint. @see
      dev_clock_sink_gate */
  DEV_USE_CLOCK_SINK_GATE_DONE,

# ifdef CONFIG_DEVICE_CLOCK_VARFREQ
  /** This operation is used when a clock frequency change occurred on
      a clock sink endpoint and the change notifications is
      enabled. The change might not have been solicited by the
      driver. The @tt param argument is a pointer to a @ref
      dev_clock_notify_s object. */
  DEV_USE_CLOCK_SINK_FREQ_CHANGED,
# endif
#endif

#ifdef CONFIG_DEVICE_ENUM
  /** This operation is used when a device child initialization completes.
      The @tt param argument is a pointer to the child device. */
  DEV_USE_ENUM_CHILD_INIT,
#endif
};

struct device_accessor_s;

/** @see dev_use_t */
#define DEV_USE(n) error_t (n) (void *param, enum dev_use_op_e op)

/** @This is called when the usage status of a device changes. The
    @ref dev_use_generic function provides a default implementation of
    this driver API function.

    This function is called with the device lock held.
    Valid operations are defined in @ref dev_use_op_e. */
typedef DEV_USE(dev_use_t);

extern DEV_USE(dev_use_generic);

/** @This specifies @ref driver_s flags */
enum driver_flags_e
{
  /** Perform initialization of the device during the @tt
     INIT_BOOTSTRAP phase instead of @tt INIT_SMP. This is needed
     mainly for processor and memory devices. It is not possible to
     rely on some kernel features (scheduler, irq, kroutines) being
     functional during the early initialization phase. */
  DRIVER_FLAGS_EARLY_INIT = 1,
  /** Do not test for missing resource dependencies before calling the
      driver initialization function on a device. */
  DRIVER_FLAGS_NO_DEPEND = 2,
  /** The dev_init_t function of the driver will be called again when
      the @tt -EAGAIN error code is returned. */
  DRIVER_FLAGS_RETRY_INIT = 4,
};

/** device driver object structure */
struct driver_s
{
#if defined(CONFIG_DEVICE_DRIVER_DESC)
  /** driver description string */
  const char *desc;
#endif

  dev_init_t	*f_init;
#if defined(CONFIG_DEVICE_CLEANUP)
  dev_cleanup_t	*f_cleanup;
#endif
  dev_use_t     *f_use;

  enum driver_flags_e flags:8;
  uint16_t      pv_size;

  /** NULL terminated array of pointers to driver classes structs */
  const struct driver_class_s *classes[];
};

#if defined(CONFIG_DEVICE_DRIVER_DESC)
# define DRIVER_DECLARE_DESC(x) .desc = (x),
#else
# define DRIVER_DECLARE_DESC(x)
#endif

#if defined(CONFIG_DEVICE_CLEANUP)
# define DRIVER_DECLARE_CLEANUP(x) .f_cleanup = (x),
#else
# define DRIVER_DECLARE_CLEANUP(x)
#endif

/** @This is used to declare a driver private data structure */
#define DRIVER_PV(...) \
typedef __VA_ARGS__ driver_pv_t;

/** @This is used to declare a driver private data local variable from
    dev */
#define DEVICE_PV(pvdata, device)               \
  driver_pv_t *pvdata = (device)->drv_pv

/** @This declares a @ref driver_s object. Implemented device
    classes must be specified as extra parameters. */
#define DRIVER_DECLARE(symbol_, flags_, pretty_, prefix_, ...)   \
  const struct driver_s symbol_ = {                    \
    DRIVER_DECLARE_DESC(pretty_)                       \
    .pv_size = sizeof(driver_pv_t),                    \
    .f_init = prefix_ ## _init,                        \
    DRIVER_DECLARE_CLEANUP(prefix_ ## _cleanup)        \
    .f_use = prefix_ ## _use,                          \
    .classes = { __VA_ARGS__, 0 },                     \
    .flags = flags_                                    \
  }

/** @internal */
struct driver_registry_s
{
  const struct driver_s *driver;  
  const struct dev_enum_ident_s	*id_table;
  size_t id_count;
};

/** @internal */
extern const struct driver_registry_s driver_registry_table[];
/** @internal */
extern const struct driver_registry_s driver_registry_table_end[];

#if defined(CONFIG_ARCH_EMU_DARWIN)
# define DRIVER_REGISTRY_SECTION(x) __attribute__((section ("__DATA, __drivers")))
#else
# define DRIVER_REGISTRY_SECTION(x) __attribute__((section (".drivers."#x)))
#endif

#ifdef CONFIG_DEVICE_DRIVER_REGISTRY
/** @This registers a @ref driver_s object in the @ref
    driver_registry_table table. This enables lookup and dynamic
    binding of the driver to a device.

    When this macro is not used, the driver can't be used for
    enumerated devices. In this case, a pointer to the driver must be
    passed to the @ref #DEV_DECLARE_STATIC macro or to the @ref
    device_bind_driver function. */
# define DRIVER_REGISTER(driver_, ...)                             \
  DRIVER_REGISTRY_SECTION(driver_)                                 \
  const struct driver_registry_s driver_##_driver_registry = {     \
    .driver = &driver_,                                            \
    .id_count = ARRAY_SIZE(((const struct dev_enum_ident_s[]){     \
      __VA_ARGS__                                                  \
    })),                                                           \
    .id_table = (const struct dev_enum_ident_s[]){                 \
      __VA_ARGS__                                                  \
    },                                                             \
  }
#else
# define DRIVER_REGISTER(driver_, ...)                             \
  __attribute__((used)) const struct driver_s driver_
#endif

/** @internal @This declares a driver class.
    @see driver_class_e
    @xsee {Device classes} */
#define DRIVER_CLASS_TYPES(id, cl, ...)                                 \
  /** @This is the driver API descriptor for the @tt cl device class.       \
      @see id @csee driver_class_s */                                  \
struct driver_##cl##_s                                                  \
{                                                                       \
  uint16_t class_; /* enum driver_class_e */                            \
  uint16_t ctx_offset; /* offsetof(driver_pv_s, libdevice_class_context) */ \
  __VA_ARGS__                                                           \
};                                                                      \
                                                                        \
/** @This is the device accessor object type for the @tt cl device class.   \
    This accessor must be initialized by calling the                    \
    @ref device_get_accessor function.                                  \
    @see id @csee device_accessor_s @xsee {Device accessor} */         \
struct device_##cl##_s                                                  \
{                                                                       \
  union {                                                               \
    struct device_accessor_s base;                                      \
    struct {                                                            \
      struct device_s *dev;                                             \
      struct driver_##cl##_s *api;                                      \
      uint_fast8_t number;                                              \
    };                                                                  \
  };                                                                    \
};                                                                      \
                                                                        \
/** @This casts a generic device accessor to a cl device accessor */    \
ALWAYS_INLINE struct device_##cl##_s *                                  \
device_##cl##_s_cast(struct device_accessor_s *x)                       \
{                                                                       \
  return (void*)x;                                                      \
}                                                                       \
                                                                        \
/** @This casts a cl device accessor to a generic device accessor */    \
ALWAYS_INLINE struct device_accessor_s *                                \
device_##cl##_s_base(struct device_##cl##_s *x)                         \
{                                                                       \
  return (void*)x;                                                      \
}

/** @internal @This declares a driver class which uses a libdevice context.
    @see #DRIVER_CLASS_TYPES */
#define DRIVER_CTX_CLASS_TYPES(id, cl, ...)                             \
DRIVER_CLASS_TYPES(id, cl, __VA_ARGS__)                                 \
                                                                        \
struct dev_##cl##_context_s;                                            \
                                                                        \
/** @This returns a pointer to the public context stored in the device  \
    private data. This is used by the libdevice generic code of the     \
    class, not relevant for all classes. */                             \
ALWAYS_INLINE struct dev_##cl##_context_s *                             \
device_##cl##_context(const struct device_##cl##_s *x)                  \
{                                                                       \
  return (void*)((uint8_t*)x->dev->drv_pv + x->api->ctx_offset);        \
}

/** @internal @This is the generic device driver class struct header. */
struct driver_class_s
{
  uint16_t class_;  /* enum driver_class_e */
  uint16_t ctx_offset; /* offsetof(driver_pv_s, libdevice_class_context) */
  void *functions[];
};

/** @This is the generic device accessor.
    @see device_get_accessor
    @xcsee {Device accessor} */
struct device_accessor_s
{
  struct device_s *dev;
  const struct driver_class_s *api;
  uint_fast8_t number;
};

/** @This invokes the requested operation on a device accessor object.
    The function must be provided by the driver.
    @xcsee {Device accessor} */
#define DEVICE_OP(dev_accessor, op, ...)        \
({                                            \
  typeof(dev_accessor) __a__ = (dev_accessor);  \
  __a__->api->f_##op(__a__, ## __VA_ARGS__);       \
})

/** @This tells whether device defines said function.
    @xcsee {Device accessor} */
#define DEVICE_HAS_OP(dev_accessor, op)        \
({                                            \
  typeof(dev_accessor) __a__ = (dev_accessor);  \
  ((void*)__a__->api->f_##op != (void*)dev_driver_notsup_fcn);      \
})

/** @This initializes a @ref device_accessor_s object to the null
    accessor. */
#define DEVICE_ACCESSOR_INIT { .dev = NULL, .api = NULL }

/** @internal */
error_t device_get_api(struct device_s *dev,
                       enum driver_class_e cl,
                       const struct driver_class_s **api);

/** @This function may be called from a deferred kroutine handler of
    the device driver when the @ref dev_init_t has previously returned
    @tt -EAGAIN and some progress has been made. It must be called
    with the device lock held.

    This is used to notify waiters, start a new pass on the device
    tree for initialization of depending devices and optionally update
    the device status.

    @list
      @item If no error is reported, the device status changes to @ref
        DEVICE_INIT_DONE.

      @item If the @tt -EAGAIN error code is used, the device is left
        in the @ref DEVICE_INIT_ONGOING state.

      @item If an error is reported and the driver has not invoked the
        @ref device_init_enable_api function yet, the status is changed
        to @ref DEVICE_INIT_FAILED. The driver must have released all
        resource associated to the device in this case as the @ref
        dev_cleanup_t function will not be called.

      @item If an error is reported and some classes have been
        reported as initialized, the status is changed to @ref
        DEVICE_INIT_PARTIAL.
    @end list */
config_depend(CONFIG_DEVICE_INIT_ASYNC)
void device_async_init_done(struct device_s *dev, error_t error);

/** @This function may be called from a deferred kroutine handler of
    the device driver when the @ref dev_cleanup_t has previously
    returned @tt -EAGAIN and the cleanup eventually terminates.
    It must be called with the device lock held. */
config_depend_and2(CONFIG_DEVICE_CLEANUP, CONFIG_DEVICE_INIT_ASYNC)
void device_async_cleanup_done(struct device_s *dev);

/** @This marks an API of the driver as available during partial
    initialization. The @tt index specifies an API as passed to the
    @ref #DRIVER_DECLARE macro of the driver.  @see
    DEVICE_INIT_ONGOING */
config_depend_alwaysinline(CONFIG_DEVICE_INIT_PARTIAL,
void device_init_enable_api(struct device_s *dev, uint_fast8_t index),
{
  dev->init_mask |= 1 << index;
})

/** @This can be used during the initialization phase of a device to
    test if a specified API is initialized. The @tt index specifies
    an API as passed to the @ref #DRIVER_DECLARE macro of the driver.
    @This always returns true if @ref #CONFIG_DEVICE_INIT_PARTIAL is
    not defined. */
ALWAYS_INLINE bool_t device_init_test_api(struct device_s *dev, uint_fast8_t index)
{
#ifdef CONFIG_DEVICE_INIT_PARTIAL
  return dev->status == DEVICE_INIT_DONE ||
    (dev->init_mask >> index) & 1;
#else
  return 1;
#endif
}

/** @This can be used from the @ref dev_init_t function to test if we
    are doing additional passes of partial device initialization.
    @This returns false if we are in the first call to the
    initialization function which is always the case when @ref
    #CONFIG_DEVICE_INIT_PARTIAL is not defined. */
ALWAYS_INLINE bool_t device_init_is_partial(struct device_s *dev)
{
#ifdef CONFIG_DEVICE_INIT_PARTIAL
  return dev->status == DEVICE_INIT_ONGOING;
#else
  return 0;
#endif
}

/** @This function is similar to @ref device_get_accessor, the current
    scheduler context is suspended until the device is initialized. */
config_depend_and2(CONFIG_DEVICE_ENUM, CONFIG_MUTEK_CONTEXT_SCHED)
error_t device_wait_accessor(struct device_accessor_s *acc, struct device_s *dev,
                             enum driver_class_e cl, uint_fast8_t number);

/** @This initializes a device accessor object. If the return value is
    0, the accessor object can then be used to access device driver
    functions of requested api class.

    The @tt number parameter can be used when the device provides
    multiple sub-devices of the requested class type.

    @xcsee {Device accessor}
    @see device_put_accessor */
error_t device_get_accessor(struct device_accessor_s *acc, struct device_s *dev,
                            enum driver_class_e cl, uint_fast8_t number);

/** @This copies a device accessor.  This gets a new reference to the
    accessor that must be released with @tt device_put_accessor by
    itself.

    @xcsee {Device accessor}
    @see device_put_accessor */
error_t device_copy_accessor(struct device_accessor_s *dst,
                             const struct device_accessor_s *src);

ALWAYS_INLINE bool_t device_cmp_accessor(const struct device_accessor_s *a,
                                         const struct device_accessor_s *b)
{
  return a->dev == b->dev && a->number == b->number;
}

/** 
    @This initializes a device accessor object after lookup in the
    device tree. The @tt root parameter may be @tt NULL to lookup from
    the device tree root.

    The path is of the form "@tt {node0/node1}". An additionnal
    instance number may be added: "@tt {node0/node1[2]}", the default
    instance number is 0. Multiple space separated paths can be
    specified as fallbacks: "@tt {node0 node1[0] node1[1]}".

    See @ref device_get_by_path for more details in the path string
    format.
    @csee device_get_accessor
    @xcsee {Device accessor}
 */
error_t device_get_accessor_by_path(struct device_accessor_s *acc,
                                    struct device_node_s *root,
                                    const char *path, enum driver_class_e cl);

/** @This function is similar to @ref device_get_accessor_by_path, the current
    scheduler context is suspended until the device is initialized. */
config_depend_and2(CONFIG_DEVICE_ENUM, CONFIG_MUTEK_CONTEXT_SCHED)
error_t device_wait_accessor_by_path(struct device_accessor_s *acc,
                                     struct device_node_s *root,
                                     const char *path, enum driver_class_e cl);

/**
   @This must be called when device driver accessor is
   discarded. @This is used to decrement device usage references
   count.
   @see {device_get_accessor}
 */
void device_put_accessor(struct device_accessor_s *acc);

/**
   @This returns true is the device accessor has been successfully
   bound to a device by either the @ref device_get_accessor or @ref
   device_get_accessor_by_path function. @This returns false if one of
   these functions failed or if the accessor has not been touched
   since it was cleaned-up by @ref device_put_accessor or initialized
   with @ref #DEVICE_ACCESSOR_INIT.
*/
ALWAYS_INLINE bool_t device_check_accessor(const struct device_accessor_s *acc)
{
  return acc->dev != NULL;
}

/**
   @This initializes an accessor so that the @ref
   device_check_accessor function return false.
*/
ALWAYS_INLINE void device_init_accessor(struct device_accessor_s *acc)
{
  acc->dev = NULL;
}

/** @This specifies the value added to the device_s::start_count value
    when the @ref device_start function is invoked. */
#define DEVICE_START_COUNT_INC (1 << CONFIG_DEVICE_START_LOG2INC)

/** @This instructs the driver to keep the device in active
    state. @This internally invoke the @ref DEV_USE_START operation of
    the driver increases the value of @ref device_s::start_count and.

    This function always returns immediately. It is implementation
    defined if the actual device startup is delayed by the driver.

    Depending on the device class, the device operation may also get
    started when submitting requests to the device driver. In this
    case, this function can be used when the device must remain active
    between requests. Refer to the device class specific documentation
    for details.

    @see device_stop. @see #DEVICE_START_COUNT_INC
*/
error_t device_start(struct device_accessor_s *acc);

/** @This reverts the effect of the @ref device_start function. @This
    internally decreases the value of @ref device_s::start_count and
    invoke the @ref DEV_USE_STOP operation of the driver.

    @see device_start.
*/
error_t device_stop(struct device_accessor_s *acc);

/** @This retreives the value of the last possibly valid sub-devices
    number. It returns @tt -ENOTSUP if the class does not implement
    sub-devices. Other errors indicate that the driver class is not
    ready for use.

    Numbers are used to specify an instance of a hardware device
    function, a logical driver feature or both. Implemented
    sub-devices numbers may not be contiguous. This allows binding a
    number to a particular feature or a special way of using the
    hardware which never changes for a given driver. In this case the
    driver may report a last number which may not be available with
    the current configuration.
 */
error_t device_last_number(struct device_s *dev,
                           enum driver_class_e cl,
                           uint_fast8_t *num);

/** @This binds a device to a device driver. No check is performed to
    determine if the driver is appropriate. */
error_t device_bind_driver(struct device_s *dev, const struct driver_s *drv);

/** @This leave the device without any associated driver. */
config_depend(CONFIG_DEVICE_CLEANUP)
error_t device_unbind_driver(struct device_s *dev);

/** @This performs device initialization using previously bound driver. */
error_t device_init_driver(struct device_s *dev);

/** @This stops the device and cleanup driver allocated resources. */
config_depend(CONFIG_DEVICE_CLEANUP)
error_t device_release_driver(struct device_s *dev);

/** @This function does nothing but returning @tt -ENOTUP */
error_t dev_driver_notsup_fcn(void);

#endif

