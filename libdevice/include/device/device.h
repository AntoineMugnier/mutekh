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
 * @short Devices definitions
 */

#ifndef __DEVICE_H__
#define __DEVICE_H__

struct device_s;
struct driver_s;

#include <hexo/types.h>
#include <hexo/error.h>

typedef uint8_t address_space_id_t;

/** Number of resource slots for statically allocated @ref device_s objects */
#define DEVICE_STATIC_RESOURCE_COUNT	2

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>

#ifdef CONFIG_DEVICE_TREE
# include <gpct/cont_clist.h>
# include <gpct/object_refcount.h>
#endif

enum dev_resource_type_e
{
    DEV_RES_UNUSED = 0,         //< Unused resource slot
    DEV_RES_MEM,                //< Physical memory address mapping resource
    DEV_RES_IO,                 //< Io space address mapping resource
    DEV_RES_IRQ,                //< Interrupt line resource
    DEV_RES_ID,                 //< Unique numeric id, meaning depends on parent device type
    DEV_RES_VENDORID,           //< Vendor id, meaning depends on parent device type
    DEV_RES_PRODUCTID,          //< Model id specific to current vendor id
    DEV_RES_REVISION,
    DEV_RES_FREQ,               //< frequency in Hertz
    DEV_RES_STR_PARAM,
    DEV_RES_UINT_PARAM,
    DEV_RES_UINT_ARRAY_PARAM,

    DEV_RES_TYPES_COUNT,              //< Number of resource types
};

struct dev_resource_s
{
  uint16_t type;                // resource descriptor type @see dev_resource_type_e
  union {
    uintptr_t uint[2];
    uint64_t uint64;
    struct {
      uintptr_t start;
      uintptr_t end;
    }   mem;

    struct {
      uintptr_t start;
      uintptr_t end;
    }   io;

    struct {
      uint8_t dev_out_id;        //< id of physical irq link going out of the device
      uint8_t icu_in_id;         //< id of physical irq link going into the interrupt controller
      uint16_t irq_id;            //< logical irq id
      struct device_s *icu;       //< associated interrupt controller
    }   irq;

    struct {
      uintptr_t major;          //< dynamic numeric id
      uintptr_t minor;          //< dynamic numeric id
    }   id;

    struct {
      uintptr_t id;          //< optional vendor numeric id, may be -1
      const char *name;     //< optional vendor string id, may be NULL
    }   vendor;

    struct {
      uintptr_t id;          //< optional device numeric id, may be -1
      const char *name;     //< optional device string, may be NULL
    }   product;

    struct {
      uintptr_t major;
      uintptr_t minor;
    }   revision;

    struct {
      uint64_t f40_24;     //< frequency in 40.24 fixed point format
    }   freq;

    struct {
      const char *name;
      const char *value;
    }   str_param;

    struct {
      const char *name;
      uintptr_t value;
    }   uint_param;

    struct {
      const char *name;
      uintptr_t *value;
    }   uint_array_param;
  };
};


/** @This returns a poiner to next unused resource slot. @internal */
struct dev_resource_s * device_res_unused(struct device_s *dev);

/** @This adds an IO space address range to the device resources list. */
error_t device_res_add_io(struct device_s *dev, uintptr_t start, uintptr_t end);

/** @This adds an memory space address range to the device resources list. */
error_t device_res_add_mem(struct device_s *dev, uintptr_t start, uintptr_t end);

/** @This adds an IRQ binding to the device resources list.

    The entry specifies how the physical output irq link of the device
    must be connected to the specified input link of the given
    interrupt controller and gives the logical irq number associated
    to the device. The logical irq number is used when the link is a
    bus carrying an interrupt identifier; zero is used if the link is a
    single wire.

    Actual binding will take place when the device driver is
    initialized. Use count of the interrupt controller device is
    increased. */
error_t device_res_add_irq(struct device_s *dev, uint_fast8_t dev_out_id, uint_fast8_t icu_in_id,
                           uint_fast16_t irq_id, struct device_s *icu);

/** @This adds a numerical identifier which uniquely identify an
    instance of the device. This is generally used by device which are
    commonly referred to by using a number. Processor devices must
    use this resource for the cpu id. */
error_t device_res_add_id(struct device_s *dev, uintptr_t major, uintptr_t minor);

/** @This adds a revision information for the device. */
error_t device_res_add_revision(struct device_s *dev, uintptr_t major, uintptr_t minor);

/** @This attaches a vendor identifier resource to the device. Both a
    numerical and a string values can be specified. The exact meaning
    of the value depends on the parent enumerator device. When the
    device node has been dynamically allocated using @ref device_alloc
    and the @tt name string is not NULL, the string pointer will be
    freed on cleanup. */
error_t device_res_add_vendorid(struct device_s *dev, uintptr_t id, const char *name);

/** @This attaches a product identifier resource to the device. Both a
    numerical and a string values can be specified. The exact meaning
    of the value depends on the parent enumerator device. When the
    device node has been dynamically allocated using @ref device_alloc
    and the @tt name string is not NULL, the string pointer will be
    freed on cleanup. */
error_t device_res_add_productid(struct device_s *dev, uintptr_t id, const char *name);

/** @This attaches a frequency resource to the device. The frenquency
    must be in 40.24 fixed point format. This value can be read by
    calling the @ref device_res_get_uint64 function. */
error_t device_res_add_frequency(struct device_s *dev, uint64_t f_40_24);

/** @This attaches a string parameter resource to the device. The
    exact meaning of the value is driver dependent. When the device
    node has been dynamically allocated using @ref device_alloc, the
    name and value string pointers will be freed on cleanup. */
error_t device_res_add_str_param(struct device_s *dev, const char *name, const char *value);

/** @This attaches an integer parameter resource to the device. The
    exact meaning of the value is driver dependent. When the device
    node has been dynamically allocated using @ref device_alloc, the
    name string pointer will be freed on cleanup. */
error_t device_res_add_uint_param(struct device_s *dev, const char *name, uintptr_t value);

/** @This attaches an integer array parameter resource to the
    device. The first value of the array must indicate the number of
    subsequent entries in the array. The exact meaning of the value is
    driver dependent. When the device node has been dynamically
    allocated using @ref device_alloc, the name and value pointers
    will be freed on cleanup. */
error_t device_res_add_uint_array_param(struct device_s *dev, const char *name, uintptr_t *value);

/** @This returns a pointer to resource of requested type with given position. */
struct dev_resource_s *device_res_get(struct device_s *dev,
                                      enum dev_resource_type_e type,
                                      uint_fast8_t number);

/** @This reads integer resource values. @This can be used to read
    ids, memory and io ranges resources. @tt a and @tt b pointers may
    be @tt NULL. */
error_t device_res_get_uint(const struct device_s *dev,
                            enum dev_resource_type_e type,
                            uint_fast8_t id, uintptr_t *a, uintptr_t *b);

/** @This reads a 64 bits integer resource values. */
error_t device_res_get_uint64(const struct device_s *dev,
                              enum dev_resource_type_e type,
                              uint_fast8_t id, uint64_t *a);

error_t device_get_param_uint(const struct device_s *dev, const char *name, uintptr_t *a);

error_t device_get_param_str(const struct device_s *dev, const char *name, char * const *a);

void device_get_param_uint_default(const struct device_s *dev, const char *name, uintptr_t *a, uintptr_t def);

void device_get_param_str_default(const struct device_s *dev, const char *name, char * const *a, const char *def);

enum device_status_e
{
  /** No driver is currently attached to the device */
  DEVICE_NO_DRIVER,
  /** A driver has been attached to the device but initialization has not been performed yet */
  DEVICE_DRIVER_INIT_PENDING,
  /** A driver has been attached to the device and initialization took place */
  DEVICE_DRIVER_INIT_DONE,
  /** A driver has been attached to the device but initialization failed */
  DEVICE_DRIVER_INIT_FAILED,
};

#define DEVICE_STATUS_NAMES "no driver", "init pending", "init ok", "init failed"

enum device_flags_e
{
  /** The device object has been dynamically allocated and must be freed on cleanup. */
  DEVICE_FLAG_ALLOCATED = 1,
  /** This device is a processor. Operations on this device may only
      be executed on the corresponding processor. */
  DEVICE_FLAG_CPU = 2,
  DEVICE_FLAG_DEVICE = 8,
  DEVICE_FLAG_ALIAS = 16,
};




#ifdef CONFIG_DEVICE_TREE
CONTAINER_TYPE(device_list, CLIST,
#endif
/** device tree base node structure */
struct device_node_s
{
  /** indicated if the device node has been dynamically allocated */
  uint_fast8_t                  flags;

#ifdef CONFIG_DEVICE_TREE
  /** device name, freed on device object destruction if not NULL and
      @tt allocated is set. */
  const char *                  name;

  struct device_node_s		*parent;
  device_list_entry_t		list_entry;
  device_list_root_t		children;
#endif
}
#ifdef CONFIG_DEVICE_TREE
, list_entry);

CONTAINER_PROTOTYPE(device_list, inline, device_list);
#endif
;


/** device node structure */
struct device_s
{
  // must be first field
  struct device_node_s          node;

  /** general purpose device lock */
  lock_t			lock;

  /** Set to true if driver initialization done */
  enum device_status_e          status;

  /** pointer to device driver if any */
  const struct driver_s		*drv;
  /** pointer to device driver private data */
  void				*drv_pv;

  /** device resources table */
  uint_fast8_t                  res_count;

  /** device uses counter */
  uint_fast8_t                  ref_count;

#ifdef CONFIG_DEVICE_TREE
  /** pointer to device enumerator private data if any */
  struct device_s               *enum_dev;
  void				*enum_pv;
#endif /* !CONFIG_DEVICE_TREE */

#if defined(CONFIG_ARCH_SMP) && defined(CONFIG_DEVICE_IRQ)
  //  struct cpu_set_s              irq_cpus;
#endif

  /** device resources table */
  struct dev_resource_s         res[DEVICE_STATIC_RESOURCE_COUNT];
};

#ifdef CONFIG_DEVICE_TREE
/** device alias node structure */
struct device_alias_s
{
  // must be first field
  struct device_node_s          node;

  /** alias path in device tree */
  const char *path;
};
#endif

/** @This initializes a statically allocated device object. Number of
    resource slot is @ref #DEVICE_STATIC_RESOURCE_COUNT
    @see {device_init, device_cleanup} */
void device_init(struct device_s *dev);

/** @This allocates and initializes a device object. Requested number
    of resource slots is allocated.
    @see {device_alloc, device_cleanup} */
struct device_s *device_alloc(size_t resources);

/** @This cleanups a device object. Memory is freed if device has been
    allocated using @ref device_alloc. Device must not be registered
    or have registered children and references count must be zero. */
void device_cleanup(struct device_s *dev);

/** @This reduces resource slots count to number of used slots
    count. The device node is reallocated to save memory. */
void device_shrink(struct device_s *dev);

/** @This creates an alias in the device tree. The parent parameter
    may be @tt NULL to attach the alias on the tree root. */
config_depend(CONFIG_DEVICE_TREE)
struct device_alias_s * device_new_alias_to_path(struct device_node_s *parent, const char *name, const char *path);

/** @This creates an alias in the device tree. The target string is
    obtained using the @ref device_get_path function. The parent
    parameter may be @tt NULL to attach the alias on the tree root. */
config_depend(CONFIG_DEVICE_TREE)
struct device_alias_s * device_new_alias_to_node(struct device_node_s *parent, const char *name, struct device_node_s *node);

config_depend(CONFIG_DEVICE_TREE)
void device_alias_remove(struct device_alias_s *alias);

/** @This returns child device by path. The @tt root parameter may be
    @tt NULL to lookup from the device tree root. Multiple space
    separated paths can be specified as fallbacks. The question mark
    character can be used as wildcard.*/
config_depend(CONFIG_DEVICE_TREE)
struct device_s *device_get_by_path(struct device_node_s *root, const char *path);

/** @This writes a null terminated device tree path in buffer. If the
    @tt number parameter is not 0, the value is appended at the end
    within a pair of square brackets. @return size of string excluding
    null byte or a negative error code. */
error_t device_get_path(struct device_node_s *root, char *buf,
                        size_t buflen, struct device_node_s *dev, uint_fast8_t number);

static inline struct device_s * device_from_node(struct device_node_s *node)
{
  return node && node->flags & DEVICE_FLAG_DEVICE ? (struct device_s*)node : NULL;
}

static inline struct device_alias_s * device_alias_from_node(struct device_node_s *node)
{
  return node && node->flags & DEVICE_FLAG_ALIAS ? (struct device_alias_s*)node : NULL;
}

static inline struct device_node_s * device_to_node(struct device_s *dev)
{
  return &dev->node;
}

static inline struct device_node_s * device_node_from_alias(struct device_alias_s *alias)
{
  return &alias->node;
}

/** @internal @This lookup a node in the device tree. */
struct device_node_s *device_node_from_path(struct device_node_s *root, const char *path,
                                            uint_fast8_t depth, const char **brackets);

/** @This attaches a device to a parent enumerator device. If the
    parent device pointer is NULL, the device is attached on root enumerator. */
config_depend(CONFIG_DEVICE_TREE)
void device_attach(struct device_s *dev,
                   struct device_s *parent);

/** @This detaches a device from its parent enumerator device */
config_depend(CONFIG_DEVICE_TREE)
void device_detach(struct device_s *dev);

/** @This prints the current devices tree. */
void device_dump(struct device_s *root);

/** @This prints the current devices tree. */
config_depend(CONFIG_DEVICE_TREE)
void device_dump_tree(struct device_node_s *root);

/** @see device_tree_walker_t */
#define DEVICE_TREE_WALKER(x) bool_t (x)(struct device_s *dev, void *priv)

/** @see device_tree_walk */
typedef DEVICE_TREE_WALKER(device_tree_walker_t);

/** @This traverse device tree calling @ref device_tree_walker_t
    function type foreach each node. Traversal stops if the provided
    function returns non-zero. */
config_depend(CONFIG_DEVICE_TREE)
bool_t device_tree_walk(struct device_node_s *root, device_tree_walker_t *walker, void *priv);

/** @This returns first device with the @tt CPU flag set and matching
    specified numerical ids. The -1 value can be used as wildcard for
    both ids but the device still has to have an id resource attached. */
config_depend(CONFIG_DEVICE_TREE)
struct device_s *device_get_cpu(uint_fast8_t major_id, uint_fast8_t minor_id);

/** @This returns the number of processor device present in the
    devices tree. */
uint_fast8_t device_get_cpu_count();

#ifdef CONFIG_VMEM
uintptr_t vpage_io_map(paddr_t paddr, size_t size);
#endif

static inline
error_t device_mem_map(struct device_s *dev, uint_fast8_t mask)
{
#if defined( CONFIG_VMEM )
    uint_fast8_t i = 0;
    while ( mask )
    {
        if ( mask & 1 )
            dev->addr[i] = vpage_io_map( dev->addr[i], 1 );
        ++i;
        mask >>= 1;
    }
#endif
    return 0;
}

/** @This specifies type of access on address space of a device. */
enum device_access_type_e
{
  DEVICE_ACCESS_READ  = 1,
  DEVICE_ACCESS_WRITE = 2,
};

/** @This finds the master interface number (@tt id) of the device @tt dev
    suitable to perform a memory access of types @tt ops on the device
    @tt target. */
config_depend(CONFIG_DEVICE_ADDRESS_SPACES)
error_t device_get_address_route(struct device_s *dev, struct device_s *target,
                                 enum device_access_type_e ops, uint_fast8_t *id);

#endif

