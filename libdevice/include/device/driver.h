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
 * @module {Devices support library}
 * @short Driver structures and driver API classes
 */

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <hexo/types.h>
#include <hexo/error.h>
#include <device/device.h>

ENUM_DESCRIPTOR(driver_class_e, strip:DRIVER_CLASS_, upper);

/** @This specifies device driver personality class. */
enum driver_class_e
{
  DRIVER_CLASS_NONE = 0,
  DRIVER_CLASS_BLOCK,
  DRIVER_CLASS_CHAR,
  DRIVER_CLASS_ENUM,
  DRIVER_CLASS_FB,
  DRIVER_CLASS_ICU,
  DRIVER_CLASS_DMA,
  DRIVER_CLASS_INPUT,
  DRIVER_CLASS_NET,
  DRIVER_CLASS_SOUND,
  DRIVER_CLASS_TIMER,
  DRIVER_CLASS_PWM,
  DRIVER_CLASS_SPI_CTRL,
  DRIVER_CLASS_LCD,
  DRIVER_CLASS_CLOCK,
  DRIVER_CLASS_GPIO,
  DRIVER_CLASS_IOMUX,
  DRIVER_CLASS_UART,
  DRIVER_CLASS_I2C,
  DRIVER_CLASS_MEM,
  DRIVER_CLASS_RFPACKET,
  DRIVER_CLASS_CRYPTO,
  DRIVER_CLASS_CPU,
  DRIVER_CLASS_VALIO,
  DRIVER_CLASS_PERSIST,

  /** Custom driver class IDs should be registered in @ref
      #CONFIG_DEVICE_CUSTOM_CLASS_COUNT enum config token.
   */
};

ENUM_DESCRIPTOR(dev_enum_type_e, strip:DEV_ENUM_TYPE_, upper);

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
	uint_fast8_t type;   //< @see dev_enum_type_e

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
   Shortcut for creating a PCI entry in a static dev_enum_ident_s
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
   Shortcut for creating an ISA entry in a static dev_enum_ident_s
   array.

   @param _vendor the vendor id to match
 */
#define DEV_ENUM_ISA_ENTRY(_vendor)						\
	{ .type = DEV_ENUM_TYPE_PCI, { .isa = {				\
				.vendor = _vendor } } }

/**
   Shortcut for creating an ATA entry in a static dev_enum_ident_s
   array.

   @param _str the string to match from the device
 */
#define DEV_ENUM_ATA_ENTRY(_str)							\
	{ .type = DEV_ENUM_TYPE_ATA, { .ata = {				\
				.str = _str } } }

/**
   Shortcut for creating a flat-device-tree entry in a static
   dev_enum_ident_s array.

   @param _name The string to match from the device-tree
 */
#define DEV_ENUM_FDTNAME_ENTRY(_name)	\
	{ .type = DEV_ENUM_TYPE_FDTNAME, { .fdtname = {		\
				.name = _name } } }

/**
   Shortcut for creating a Gaisler GAISLER entry in a static dev_enum_ident_s
   array.

   @param _vendor the vendor id to match, -1 for wildcard
   @param _device the device id to match, -1 for wildcard
 */
#define DEV_ENUM_GAISLER_ENTRY(_vendor, _device)		\
	{ .type = DEV_ENUM_TYPE_GAISLER, { .grlib = {				\
				.vendor = _vendor, .device = _device } } }

/**
   Shortcut for creating a Generic with vendor/device ids and version
   number in a static dev_enum_ident_s array.

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


/** Common class init() function template. */
#define DEV_INIT(n)	error_t (n) (struct device_s *dev, uint32_t cl_missing)

/**
   @This is device init() function type. This function will allocate
   device private data and initialize the hardware. It must be called
   before using any other functions on the device.
   
   @This must update the @ref device_s::status value to indicate the
   new status of the device. The value can be set to any status with
   the @tt DEVICE_DRIVER_INIT_ prefix.

   The @ref DEVICE_DRIVER_INIT_PARTIAL status can be used to indicate
   that only some drivers classes are properly initialized and ready
   for use. The @ref device_s::init_mask bit mask must be updated
   along in this case.

   This init function must return 0 when some progress have been made
   or @tt -EAGAIN when some resources are missing to complete
   initialization of the device. In both cases, it may be called again
   later depending on the new device status. Any other error code can
   be used. Any permanent error must be indicated by setting the
   status to DEVICE_DRIVER_INIT_FAILED so that the function is not
   called again later.

   The @ref DRIVER_FLAGS_NO_DEPEND flag can be used so that this
   function is called even if some resource dependencies are not
   satisfied. The driver is then responsible for testing missing
   dependencies. In this case the @tt cl_missing mask indicates any
   driver classes related to missing resource dependencies.

   This function may be called early during startup depending on the
   driver. Some kernel service can not be used from this function if
   the driver has the @ref DRIVER_FLAGS_EARLY_INIT flag set.
*/
typedef DEV_INIT(dev_init_t);



/** Common device class cleanup() function template. */
#define DEV_CLEANUP(n)	__attribute__((unused)) error_t    (n) (struct device_s *dev)

/**
   @This is device cleanup() function type. @This tries to release
   all ressources allocated by the initialization function.
   This function will only be called when @ref device_s::ref_count
   and @ref device_s::start_count are both zero.

   This function is called with the device lock held.
   @This function must return @tt -EBUSY if it is not possible to
   release the device yet.
*/
typedef DEV_CLEANUP(dev_cleanup_t);

/** @This specifies device use and start/stop operations. @see dev_use_t */
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
};

struct device_accessor_s;

/** Common device class use() function template. */
#define DEV_USE(n) error_t (n) (void *param, enum dev_use_op_e op)

/** @This is called when the usage status of a device changes. The
    @ref dev_use_generic function provides a default implementation of
    this driver API function.

    This function is called with the device lock held.
    Valid operations are defined in @ref dev_use_op_e. */
typedef DEV_USE(dev_use_t);

extern DEV_USE(dev_use_generic);

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
};

/** device driver object structure */
struct driver_s
{
#if defined(CONFIG_DEVICE_DRIVER_DESC)
  /** driver description string */
  const char *desc;
#endif

  dev_init_t	*f_init;
#if defined(CONFIG_DEVICE_DRIVER_CLEANUP)
  dev_cleanup_t	*f_cleanup;
#endif
  dev_use_t     *f_use;

  enum driver_flags_e flags;

  /** NULL terminated array of pointers to driver classes structs */
  const struct driver_class_s *classes[];
};

#if defined(CONFIG_DEVICE_DRIVER_DESC)
# define DRIVER_DECLARE_DESC(x) .desc = (x),
#else
# define DRIVER_DECLARE_DESC(x)
#endif

#if defined(CONFIG_DEVICE_DRIVER_CLEANUP)
# define DRIVER_DECLARE_CLEANUP(x) .f_cleanup = (x),
#else
# define DRIVER_DECLARE_CLEANUP(x)
#endif

#define DRIVER_DECLARE(symbol_, flags_, pretty_, prefix_, ...)   \
  const struct driver_s symbol_ = {                    \
    DRIVER_DECLARE_DESC(pretty_)                       \
    .f_init = prefix_ ## _init,                        \
    DRIVER_DECLARE_CLEANUP(prefix_ ## _cleanup)        \
    .f_use = prefix_ ## _use,                          \
    .classes = { __VA_ARGS__, 0 },                     \
    .flags = flags_                                    \
  }

struct driver_registry_s
{
  const struct driver_s *driver;  
  const struct dev_enum_ident_s	*id_table;
  size_t id_count;
};

extern const struct driver_registry_s driver_registry_table[];
extern const struct driver_registry_s driver_registry_table_end[];

/**
   Registers a driver (struct driver_s) in the driver_registry_table
   table.
 */
#if defined(CONFIG_ARCH_EMU_DARWIN)
# define DRIVER_REGISTRY_SECTION(x) __attribute__((section ("__DATA, __drivers")))
#else
# define DRIVER_REGISTRY_SECTION(x) __attribute__((section (".drivers."#x)))
#endif

#ifdef CONFIG_DEVICE_DRIVER_REGISTRY
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

/**
   @internal @This declares a device accessor type.
*/
#define DRIVER_CLASS_TYPES(cl, ...)                                    \
/**                                                                     \
   @This is the device accessor object type for the cl device class.    \
   This accessor must be initialized                                    \
   using the @ref device_get_accessor function before being used to     \
   access the device.                                                   \
   @see {device_get_accessor, device_put_accessor}                      \
*/                                                                      \
struct device_##cl##_s                                                  \
{                                                                       \
  struct device_s *dev;                                                 \
  struct driver_##cl##_s *api;                                          \
  uint_fast8_t number;                                                  \
};                                                                      \
                                                                        \
/**                                                                     \
   @This is the driver API descriptor for the cl device class.          \
*/                                                                      \
struct driver_##cl##_s                                                  \
{                                                                       \
  uint16_t class_; /* enum driver_class_e */                            \
  __VA_ARGS__                                                           \
};                                                                      \
                                                                        \
ALWAYS_INLINE struct device_##cl##_s *                                  \
device_##cl##_s_cast(struct device_accessor_s *x)                       \
{                                                                       \
  return (void*)x;                                                      \
}                                                                       \
                                                                        \
ALWAYS_INLINE struct device_accessor_s *                                \
device_##cl##_s_base(struct device_##cl##_s *x)                         \
{                                                                       \
  return (void*)x;                                                      \
}

struct driver_class_s
{
  uint16_t class_;  /* enum driver_class_e */
  void *functions[];
};

struct device_accessor_s
{
  struct device_s *dev;
  const struct driver_class_s *api;
  uint_fast8_t number;
};

/**
   @This invokes the requested operation on a device accessor object.
   The function must be provided by the driver.
*/
#define DEVICE_OP(dev_accessor, op, ...)        \
({                                            \
  typeof(dev_accessor) __a__ = (dev_accessor);  \
  __a__->api->f_##op(__a__, ## __VA_ARGS__);       \
})

/**
   @This checks if the driver behind the given device accessor
   provides the specified operation.
*/
#define DEVICE_HAS_OP(dev_accessor, op)        \
({                                            \
  typeof(dev_accessor) __a__ = (dev_accessor);  \
  __a__->api->f_##op != NULL;       \
})

#define DEVICE_ACCESSOR_INIT { .dev = NULL, .api = NULL }

/** @internal */
error_t device_get_api(struct device_s *dev,
                       enum driver_class_e cl,
                       const struct driver_class_s **api);

/**
   @This initializes a device accessor object. If the return value is
   0, the accessor object can then be used to access device driver
   functions of requested api class.

   The @tt number parameter can be used when the device provides
   multiple sub-devices of the requested class type.

   @see {#DEVICE_ACCESSOR_INIT, #DEVICE_OP, device_put_accessor}
 */
error_t device_get_accessor(void *accessor, struct device_s *dev,
                            enum driver_class_e cl, uint_fast8_t number);

/**
   @This copies a device accessor.  This gets a new reference to the
   accessor that must be released with @tt device_put_accessor by
   itself.

   @see {#DEVICE_ACCESSOR_INIT, #DEVICE_OP, device_put_accessor}
 */
error_t device_copy_accessor(void *accessor, const void *source);

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
 */
error_t device_get_accessor_by_path(void *accessor, struct device_node_s *root,
                                    const char *path, enum driver_class_e cl);

/**
   @This must be called when device driver accessor is
   discarded. @This is used to decrement device usage references
   count.
   @see {device_get_accessor}
 */
void device_put_accessor(void *accessor);

/**
   @This returns true is the device accessor has been successfully
   bound to a device by either the @ref device_get_accessor or @ref
   device_get_accessor_by_path function. @This returns false if one of
   these functions failed or if the accessor has not been touched
   since it was cleaned-up by @ref device_put_accessor or initialized
   with @ref #DEVICE_ACCESSOR_INIT.
*/
ALWAYS_INLINE bool_t device_check_accessor(void *accessor)
{
  struct device_accessor_s *a = accessor;

  return a->dev != NULL;
}

/**
   @This initializes an accessor so that the @ref
   device_check_accessor function return false.
*/
ALWAYS_INLINE void device_init_accessor(void *accessor)
{
  struct device_accessor_s *a = accessor;
  a->dev = NULL;
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
error_t device_start(void *accessor);

/** @This reverts the effect of the @ref device_start function. @This
    internally decreases the value of @ref device_s::start_count and
    invoke the @ref DEV_USE_STOP operation of the driver.

    @see device_start.
*/
void device_stop(void *accessor);

/** @internal @This is similar to @ref device_stop, for test purpose only. */
void device_stop_safe(void *accessor);

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

/**
   @This walks down the device tree from specified node (from root if
   @tt dev is NULL) and try to find appropriate driver for each
   device and eventually initializes it provided that all resources
   are available.

   The device initialization can be skipped if dependencies have not
   been initialized yet. The tree is traversed multiple times until no
   more actions can be taken.
*/
config_depend(CONFIG_DEVICE_TREE)
void device_find_driver(struct device_node_s *node, uint_fast8_t pass);

/** @This binds a device to a device driver. No check is performed to
    determine if the driver is appropriate. */
error_t device_bind_driver(struct device_s *dev, const struct driver_s *drv);

/** @This leave the device without any associated driver. */
config_depend(CONFIG_DEVICE_DRIVER_CLEANUP)
error_t device_unbind_driver(struct device_s *dev);

/** @This performs device initialization using previously bound driver. */
error_t device_init_driver(struct device_s *dev);

/** @This stops the device and cleanup driver allocated resources. */
config_depend(CONFIG_DEVICE_DRIVER_CLEANUP)
error_t device_release_driver(struct device_s *dev);

/** @This function does nothing but returning @tt -ENOTUP */
error_t dev_driver_notsup_fcn(void);

#endif

