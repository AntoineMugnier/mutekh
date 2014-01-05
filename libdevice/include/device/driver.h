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
 * @module{Device drivers}
 * @short Driver classes definitions
 */                                                                 

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <hexo/types.h>
#include <hexo/error.h>
#include <device/device.h>

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
  DRIVER_CLASS_SPI_CTRL,
  DRIVER_CLASS_LCD,
  DRIVER_CLASS_GPIO,
  DRIVER_CLASS_I2C,
  DRIVER_CLASS_MEM,
  DRIVER_CLASS_CPU,
  DRIVER_CLASS_Sys_Last = DRIVER_CLASS_CPU, //< last MutekH reserved value in use
  DRIVER_CLASS_User_First = 128,            //< First user defined device class id
};

#define DRIVER_CLASS_NAMES                                         \
  "None", "Block", "Char", "Enumerator", "FrameBuffer",            \
  "ICU", "DMA", "Input", "Network", "Sound",                       \
  "Timer", "SPI Controller" , "LCD", "GPIO", "I2C", "Memory", "CPU"

enum dev_enum_type_e
{
  DEVENUM_TYPE_INVALID,
  DEVENUM_TYPE_GENERIC,
  DEVENUM_TYPE_PCI,
  DEVENUM_TYPE_ISA,
  DEVENUM_TYPE_ATA,
  DEVENUM_TYPE_FDTNAME,
  DEVENUM_TYPE_GAISLER,
};

/** device structure identification informations. wildcard values are
    enum driver dependent */
struct devenum_ident_s
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
   Shortcut for creating a PCI entry in a static devenum_ident_s
   array.

   @param _vendor the vendor id to match, -1 for wildcard
   @param _device the device id to match, -1 for wildcard
   @param _class the class to match, -1 for wildcard
 */
#define DEVENUM_PCI_ENTRY(_vendor, _device, _class)		\
	{ .type = DEVENUM_TYPE_PCI, { .pci = {				\
				.vendor = _vendor, .device = _device,	\
				.class = _class } } }

/**
   Shortcut for creating an ISA entry in a static devenum_ident_s
   array.

   @param _vendor the vendor id to match
 */
#define DEVENUM_ISA_ENTRY(_vendor)						\
	{ .type = DEVENUM_TYPE_PCI, { .isa = {				\
				.vendor = _vendor } } }

/**
   Shortcut for creating an ATA entry in a static devenum_ident_s
   array.

   @param _str the string to match from the device
 */
#define DEVENUM_ATA_ENTRY(_str)							\
	{ .type = DEVENUM_TYPE_ATA, { .ata = {				\
				.str = _str } } }

/**
   Shortcut for creating a flat-device-tree entry in a static
   devenum_ident_s array.

   @param _name The string to match from the device-tree
 */
#define DEVENUM_FDTNAME_ENTRY(_name)	\
	{ .type = DEVENUM_TYPE_FDTNAME, { .fdtname = {		\
				.name = _name } } }

/**
   Shortcut for creating a Gaisler GAISLER entry in a static devenum_ident_s
   array.

   @param _vendor the vendor id to match, -1 for wildcard
   @param _device the device id to match, -1 for wildcard
 */
#define DEVENUM_GAISLER_ENTRY(_vendor, _device)		\
	{ .type = DEVENUM_TYPE_GAISLER, { .grlib = {				\
				.vendor = _vendor, .device = _device } } }

/**
   Shortcut for creating a Generic with vendor/device ids and version
   number in a static devenum_ident_s array.

   @param _vendor the vendor id to match, -1 for wildcard
   @param _device the device id to match, -1 for wildcard
   @param _rev_major the device major revision, -1 for wildcard
   @param _rev_minor the minimum supported minor revision, -1 for wildcard
 */
#define DEVENUM_GENERIC_ENTRY(_vendor, _device, _rev_major, _rev_minor)	\
  { .type = DEVENUM_TYPE_GENERIC, { .generic = {                        \
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
#define DEV_CLEANUP(n)	void    (n) (struct device_s *dev)

/**
   @This is device cleanup() function type. Free all ressources
   allocated with the init() function.

   @param dev pointer to device descriptor
*/
typedef DEV_CLEANUP(dev_cleanup_t);



/** Common device class ioctl() function template. */
#define DEV_IOCTL(n) void (n) (struct device_s *dev, uint_fast8_t id, void *param)


/**
   @This is device ioctl() function type. This function may be used to
   tweak device specific features which are not available using driver
   class API. This function should be used to access optional device
   features only, relying on this function for operations which are
   mandatory to make the device work indicates design error or wrong
   driver class usage.

   @param dev pointer to device descriptor
   @param id device specific operation id
   @param param device specific operation parameters
*/
typedef DEV_IOCTL(dev_ioctl_t);



/** device driver object structure */

struct driver_s
{
  /** device identifier table for detection (optional) */
  const struct devenum_ident_s	*id_table;

  /** driver description string */
  const char *desc;

  /** driver initialization function */
  dev_init_t	*f_init;
  /** driver cleanup function */
  dev_cleanup_t	*f_cleanup;

  /** NULL terminated array of pointers to driver classes structs */
  const void	*classes[];
};

/**
   Registers a driver (struct driver_s) in the dev_drivers_table
   table.
 */
#if defined(CONFIG_ARCH_EMU_DARWIN)
#define REGISTER_DRIVER(name) \
	const __attribute__((section ("__DATA, __drivers"))) \
	const struct driver_s *name##_drv_ptr = &name
#else
#define REGISTER_DRIVER(name) \
	const __attribute__((section (".drivers"))) \
	const struct driver_s *name##_drv_ptr = &name
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
  enum driver_class_e class_;                                           \
  __VA_ARGS__                                                           \
};

struct device_accessor_s
{
  struct device_s *dev;
  const void *api;
  uint_fast8_t number;
};

struct driver_class_s
{
  enum driver_class_e class_;
  void *functions[];
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
   @This invokes the requested operation on a device accessor object.
   If the function is not provided by the driver, this evaluates to @tt -ENOTSUP.
*/
#define DEVICE_SAFE_OP(dev_accessor, op, ...)        \
({                                            \
  typeof(dev_accessor) __a__ = (dev_accessor);  \
  __a__->api->f_##op ? __a__->api->f_##op(__a__, ## __VA_ARGS__) : -ENOTSUP;     \
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

   @see {#DEVICE_ACCESSOR, #DEVICE_OP, device_put_accessor}
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
static inline bool_t device_check_accessor(void *accessor)
{
  struct device_accessor_s *a = accessor;

  return a->dev != NULL;
}

/**
   @This initializes an accessor so that the @ref
   device_check_accessor function return false.
*/
static inline void device_init_accessor(void *accessor)
{
  struct device_accessor_s *a = accessor;
  a->dev = NULL;
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

#endif

