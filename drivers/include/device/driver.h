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
enum device_class_e
  {
    DEVICE_CLASS_NONE = 0,

    DEVICE_CLASS_BLOCK,
    DEVICE_CLASS_CHAR,
    DEVICE_CLASS_ENUM,
    DEVICE_CLASS_FB,
    DEVICE_CLASS_ICU,
    DEVICE_CLASS_INPUT,
    DEVICE_CLASS_NET,
    DEVICE_CLASS_SOUND,
    DEVICE_CLASS_TIMER,
    DEVICE_CLASS_SPI,
    DEVICE_CLASS_LCD,
    DEVICE_CLASS_GPIO,
    DEVICE_CLASS_I2C,
    DEVICE_CLASS_MEM,
  };

#define PARAM_DATATYPE_INT 1
#define PARAM_DATATYPE_DEVICE_PTR 2
#define PARAM_DATATYPE_ADDR 3
#define PARAM_DATATYPE_BOOL 4

/**
   A link from a device property and a field in the parameter
   structure of a driver init()
 */
struct driver_param_binder_s
{
	const char *param_name;
	uint16_t struct_offset;
	uint8_t datatype;
	uint8_t datalen;
};

/**
   Helper macro to create an entry in a struct driver_param_binder_s

   @param _struct_type full type name of the parameter structure type
   @param _struct_entry field name in the parameter structure
   @param _datatype type of the data chosen in the PARAM_DATATYPE_*
 */
#define PARAM_BIND(_struct_type, _struct_entry, _datatype)			    \
	{																	\
		.param_name = #_struct_entry,									\
		.struct_offset = __builtin_offsetof(_struct_type, _struct_entry), \
		.datatype = _datatype,										    \
		.datalen = sizeof(((_struct_type *)0)->_struct_entry),		    \
	}

enum dev_enum_type_e {
  DEVENUM_TYPE_INVALID,
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
			size_t param_size;
			const struct driver_param_binder_s *binder;
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
   @param _binder The data binder table pointer for the fdt to param conversion
 */
#define DEVENUM_FDTNAME_ENTRY(_name, _psize, _binder)	\
	{ .type = DEVENUM_TYPE_FDTNAME, { .fdtname = {		\
				.name = _name, .param_size = _psize,	\
				.binder = _binder } } }

/**
   Shortcut for creating a Gaisler GAISLER entry in a static devenum_ident_s
   array.

   @param _vendor the vendor id to match, -1 for wildcard
   @param _device the device id to match, -1 for wildcard
 */
#define DEVENUM_GAISLER_ENTRY(_vendor, _device)		\
	{ .type = DEVENUM_TYPE_GAISLER, { .grlib = {				\
				.vendor = _vendor, .device = _device } } }

#define DEV_IRQ(n) struct dev_irq_ep_s* (n) (struct dev_irq_ep_s *src, uint_fast8_t *id)

/** Common device class irq() function type. Must be called on
    interrupt request.

    * @param dev pointer to device descriptor
    * @return 1 if interrupt have been handled by the device
    */

/**
   @This is irq handling function of device node.

   @param src end point which relayed the irq.
   @param id local identifier of irq line for relaying device.

   Icu devices return pointer to next irq sink end-point or
   NULL. Non-icu devices always return NULL.

   The id must be changed to -1 when no irq were pending.
   Non-icu devices only set to -1 or 0.

   Icu devices have to determine the next sink endpoint from its
   internal registers or passed id value. On some systems the icu
   passes the decoded vector id to the processor in hardware and we
   need a way to pass this value back from one icu handler to the next
   one. Icu devices may change the id value so that it is relevant for
   the next handler.
*/
typedef DEV_IRQ(dev_irq_t);




/** Common class init() function template. */
#define DEV_INIT(n)	error_t (n) (struct device_s *dev, void *params)

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

#ifdef CONFIG_HEXO_IRQ
  /** driver irq handling function */
  dev_irq_t	*f_irq;
#endif

  /** NULL terminated array of pointers to driver classes structs */
  const void	*classes[];
};

/**
   Registers a driver (struct driver_s) in the global_driver_registry
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
#define DEVICE_CLASS_TYPES(cl, ...)                                    \
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
  enum device_class_e class_;                                           \
  __VA_ARGS__                                                           \
};



/**
   @This invokes requested operation on device using device accessor object.
 */
#define DEVICE_OP(dev_accessor, op, ...)        \
({                                            \
  typeof(dev_accessor) __a__ = (dev_accessor);  \
  __a__->api->f_##op(__a__, __VA_ARGS__);       \
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
                            enum device_class_e cl, uint_fast8_t number);

/**
   @This must be called when device driver accessor is
   discarded. @This is used to decrement device usage references
   count.
   @see {device_get_accessor}
 */
void device_put_accessor(void *accessor);

/**
   @This walks down the device tree from specified node (from root if
   @tt dev is NULL) and try to find appropriate driver for each
   device and eventually initializes it provided that all resources
   are available. (The device initialization can be skipped if the
   associated interrupt controller has not been initialized yet for
   instance.)

   The tree is traversed multiple times until no more actions can be
   taken.
*/
void device_bind_driver(struct device_s *dev);

#endif
