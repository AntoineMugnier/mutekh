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
#define DEV_INIT(n)	error_t (n) (struct device_s *dev)

/**
   @This is device init() function type. This function will allocate
   device private data and initialize the hardware. It must be called
   before using any other functions on the device.
   
   @This must update the @ref device_s::status value to indicate the
   new status of the device. The value can be left to @ref
   DEVICE_DRIVER_INIT_PENDING if the device was not initialized due to
   missing resource but can be successfully initialized later.

   @param dev pointer to device descriptor
   @param params driver dependent parameters, NULL if none
   @return negative error code, 0 on succes
*/
typedef DEV_INIT(dev_init_t);



/** Common device class cleanup() function template. */
#if defined(CONFIG_DEVICE_DRIVER_CLEANUP)
#define DEV_CLEANUP(n)	void    (n) (struct device_s *dev)
#else
#define DEV_CLEANUP(n)	__attribute__((unused)) void    (n) (struct device_s *dev)
#endif

/**
   @This is device cleanup() function type. Free all ressources
   allocated with the init() function.

   @param dev pointer to device descriptor
*/
typedef DEV_CLEANUP(dev_cleanup_t);

/** @This specifies device use and start/stop operations. @see dev_use_t */
enum dev_use_op_e
{
  DEV_USE_GET_ACCESSOR,
  DEV_USE_PUT_ACCESSOR,
  DEV_USE_START,
  DEV_USE_STOP,
};

struct device_accessor_s;

/** Common device class use() function template. */
#define DEV_USE(n) error_t (n) (struct device_accessor_s *accessor,  \
                                enum dev_use_op_e op)

/**
   @This is called when the usage status of a device changes.

   This function is optional and may not be provided by all device
   drivers.

   When the @ref device_get_accessor function is called, this function
   is called with the @tt op parameter set to @ref
   DEV_USE_GET_ACCESSOR and the @tt acc parameter pointing to the
   initialized accessor. The function may return an error code in
   order to make the process fail. The device use count is updated
   after this function call.

   When the @ref device_put_accessor function is called, this function
   is called with the @tt op parameter set to @ref DEV_USE_PUT_ACCESSOR.

   The @ref device_start and @ref device_stop function call this
   function with the @ref DEV_USE_START and @ref DEV_USE_STOP
   operation values.
*/
typedef DEV_USE(dev_use_t);

extern DEV_USE(dev_use_generic);

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

  /** NULL terminated array of pointers to driver classes structs */
  const void	*classes[];
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

#define DRIVER_DECLARE(symbol_, pretty_, prefix_, ...) \
  const struct driver_s symbol_ = {                    \
    DRIVER_DECLARE_DESC(pretty_)                       \
    .f_init = prefix_ ## _init,                        \
    DRIVER_DECLARE_CLEANUP(prefix_ ## _cleanup)        \
    .f_use = prefix_ ## _use,                          \
    .classes = { __VA_ARGS__, 0 },                     \
  }

struct driver_registry_s
{
  const struct driver_s *driver;  
  const struct dev_enum_ident_s	*id_table;
  size_t id_count;
};

/**
   Registers a driver (struct driver_s) in the driver_registry_table
   table.
 */
#if defined(CONFIG_ARCH_EMU_DARWIN)
# define DRIVER_REGISTRY_SECTION(x) __attribute__((section ("__DATA, __drivers")))
#else
# define DRIVER_REGISTRY_SECTION(x) __attribute__((section (".drivers."#x)))
#endif

#if defined(CONFIG_DEVICE_ENUM)
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

/**
   @This initializes a device accessor object. If the return value is
   0, the accessor object can then be used to access device driver
   functions of requested api class. The @tt number parameter can be
   used when the device provides more than one api instance of the
   requested class type.

   @see {#DEVICE_ACCESSOR_INIT, #DEVICE_OP, device_put_accessor}
 */
error_t device_get_accessor(void *accessor, struct device_s *dev,
                            enum driver_class_e cl, uint_fast8_t number);

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

/** @This starts the device operation.

    Depending on the device class, the device operation may be started
    and stopped by submitting requests. This function can be used when
    the device active state needs to be changed explicitly. A counter
    must be used internally to match the number of calls with the @ref
    device_stop function.

    @see device_stop.
*/
ALWAYS_INLINE error_t device_start(void *accessor)
{
  struct device_accessor_s *acc = accessor;
  dev_use_t *use = acc->dev->drv->f_use;
  return use != NULL ? use(accessor, DEV_USE_START) : -ENOTSUP;
}

/** @This stops the device operation. This function return 0 if the
    device has actually been stopped. If the internal use count has
    not reached zero, @tt -EBUSY is returned.

    @see device_start.
*/
ALWAYS_INLINE error_t device_stop(void *accessor)
{
  struct device_accessor_s *acc = accessor;
  dev_use_t *use = acc->dev->drv->f_use;
  return use != NULL ? use(accessor, DEV_USE_STOP) : -ENOTSUP;
}

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
void device_find_driver(struct device_node_s *node);

/** @This binds a device to a device driver. No check is performed to
    determine if the driver is appropriate. */
error_t device_bind_driver(struct device_s *dev, const struct driver_s *drv);

/** @This performs device initialization using previously bound driver. */
error_t device_init_driver(struct device_s *dev);

/** @This function does nothing but returning @tt -ENOTUP */
error_t dev_driver_notsup_fcn();

#endif

