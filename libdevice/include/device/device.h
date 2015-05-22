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

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006-2013

*/

/**
 * @file
 * @module{Devices support library}
 * @short Device structures and device tree
 */

#ifndef __DEVICE_H__
#define __DEVICE_H__

struct device_s;
struct device_node_s;
struct device_alias_s;
struct driver_s;
struct dev_resource_s;
struct dev_resource_table_s;

#include <hexo/types.h>
#include <hexo/error.h>
#include <hexo/lock.h>

#include <device/types.h>

typedef uint8_t address_space_id_t;

#include <gct_platform.h>

#ifdef CONFIG_DEVICE_TREE
# include <gct/container_clist.h>
#endif

ENUM_DESCRIPTOR(device_status_e, strip:DEVICE_, upper);

/** @This specifies the initialization status of a device */
enum device_status_e
{
  /** Device enumeration error, some resource entries may be wrong or missing. */
  DEVICE_ENUM_ERROR,
  /** No driver is currently attached to the device */
  DEVICE_NO_DRIVER,
  /** A driver has been attached to the device but initialization has not been performed yet */
  DEVICE_DRIVER_INIT_PENDING,
  /** A driver has been attached to the device and initialization took place */
  DEVICE_DRIVER_INIT_DONE,
  /** A driver has been attached to the device but initialization failed */
  DEVICE_DRIVER_INIT_FAILED,
};

/** @This specifies device node type and flags */
enum device_flags_e
{
  /** The device object has been dynamically allocated and must be freed on cleanup. */
  DEVICE_FLAG_ALLOCATED = 1,
  /** The node name has been dynamically allocated and must be freed on cleanup. */
  DEVICE_FLAG_NAME_ALLOCATED = 2,

  /** This device is a processor. Operations on this device may only
      be executed on the corresponding processor. */
  DEVICE_FLAG_CPU = 4,
  /** The tree node is a device node */
  DEVICE_FLAG_DEVICE = 8,
  /** The tree node is an alias node */
  DEVICE_FLAG_ALIAS = 16,
  /** Mark the device as not available. The device will not be
      initialized on startup and lookup functions will ignore the node. */
  DEVICE_FLAG_IGNORE = 32,
};

#define GCT_CONTAINER_ALGO_device_list CLIST

#ifdef CONFIG_DEVICE_TREE
GCT_CONTAINER_TYPES(device_list,
#endif
/** device tree base node structure */
struct device_node_s
{
  enum device_flags_e           flags;

  /** device name, freed on device object destruction if not NULL and
      @tt allocated is set. */
  const char *                  name;

#ifdef CONFIG_DEVICE_TREE
  struct device_node_s		*parent;
  device_list_entry_t		list_entry;
  device_list_root_t		children;
#endif
}
#ifdef CONFIG_DEVICE_TREE
 *, list_entry);

GCT_CONTAINER_FCNS(device_list, inline, device_list,
                   init, destroy, pushback, remove, isempty);
#endif
;


/** device node structure */
struct device_s
{
  /* must be first field */
  struct device_node_s          node;

  /** general purpose device lock */
  lock_t			lock;

  /** Set to true if driver initialization done */
  enum device_status_e          status;

  /** pointer to device driver if any */
  const struct driver_s		*drv;
  /** pointer to device driver private data */
  void				*drv_pv;

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
  struct dev_resource_table_s   *res_tbl;
};

#ifdef CONFIG_DEVICE_TREE
/** @This statically declares and intializes a global @ref device_s
    object. The device object is initialized as if the @ref
    device_init, @ref device_set_name and @ref device_bind_driver
    functions were called.

    When @ref #CONFIG_DEVICE_TREE is defined, statically declared
    devices are attached to the device tree on startup, unless the
    @ref DEVICE_FLAG_IGNORE flag is used.

    When @ref #CONFIG_DEVICE_TREE is not defined, statically declared
    devices are part of the static device table. Devices in this table
    are initialized on startup and can be searched using @ref
    device_get_by_path and related functions, unless the @ref
    DEVICE_FLAG_IGNORE flag is set.

    @see #DEV_DECLARE_STATIC_RESOURCES
    @see device_init @see device_alloc
*/
# define DEV_DECLARE_STATIC(declname_, name_, flags_, driver_, resources_...) \
    extern const struct driver_s driver_;                               \
    __attribute__ ((aligned (sizeof(void*))))                           \
    __attribute__((section (".devices")))                               \
    struct device_s declname_ = {                                       \
      .node = {                                                         \
        .flags = flags_ | DEVICE_FLAG_DEVICE,                           \
        .name = name_,                                                  \
        .parent = NULL,                                                 \
        .children = { .ht = { .next = &declname_.node.children.ht,      \
                              .prev = &declname_.node.children.ht } },  \
      },                                                                \
      .lock = LOCK_INITIALIZER,                                         \
      .status = DEVICE_DRIVER_INIT_PENDING,                             \
      .drv = &driver_,                                                  \
      .ref_count = 0,                                                   \
      .enum_dev = NULL,                                                 \
      .res_tbl = VA_COUNT(resources_)                                   \
      ? (struct dev_resource_table_s *)DEV_STATIC_RESOURCES(resources_) \
      : NULL,                                                           \
    }
#else
# define DEV_DECLARE_STATIC(declname_, name_, flags_, driver_, resources_...) \
    extern const struct driver_s driver_;                               \
    __attribute__ ((aligned (sizeof(void*))))                           \
    __attribute__((section (".devices")))                               \
    struct device_s declname_ = {                                       \
      .node = {                                                         \
        .flags = flags_ | DEVICE_FLAG_DEVICE,                           \
        .name = name_,                                                  \
      },                                                                \
      .lock = LOCK_INITIALIZER,                                         \
      .status = DEVICE_DRIVER_INIT_PENDING,                             \
      .drv = &driver_,                                                  \
      .ref_count = 0,                                                   \
      .res_tbl = ARRAY_SIZE(DEV_STATIC_RESOURCES_ARRAY(resources_))     \
      ? (struct dev_resource_table_s *)DEV_STATIC_RESOURCES(resources_) \
      : NULL,                                                           \
    };
#endif


#ifdef CONFIG_DEVICE_TREE
/** device alias node structure */
struct device_alias_s
{
  /* must be first field */
  struct device_node_s          node;

  /** alias path in device tree */
  const char *path;
};
#endif

#ifdef CONFIG_DEVICE_TREE
/** @This iterates over child nodes of a device tree node. */
# define DEVICE_NODE_FOREACH(root_, rvar_, ... /* loop body */)         \
  GCT_FOREACH_NOLOCK(device_list, &(root_)->children, item, { \
      struct device_node_s *rvar_ = item;                               \
      __VA_ARGS__;                                                      \
    })
#else
/** @This iterates over statically defined devices. */
# define DEVICE_NODE_FOREACH(root_, rvar_, ... /* loop body */)         \
  do {                                                                  \
    extern struct device_s dev_devices_table;                           \
    extern struct device_s dev_devices_table_end;                       \
    struct device_s *_d;                                                \
    for (_d = &dev_devices_table; _d < &dev_devices_table_end; _d++)    \
      {                                                                 \
        struct device_node_s *rvar_ = &_d->node;                        \
        __VA_ARGS__;                                                    \
      }                                                                 \
  } while(0)
#endif

/**
   @This initializes a statically allocated device object. Devices
   declared using the @ref #DEV_DECLARE_STATIC macro do not require
   use of this function.

   @see {device_init, device_cleanup} */
void device_init(struct device_s *dev, const struct dev_resource_table_s *tbl);

/** @This dynamically allocates and initializes a device object. The
    specified number of resource slots are pre-allocated.

    @see {device_alloc, device_cleanup}
    @see #DEV_DECLARE_STATIC
*/
struct device_s *device_alloc(size_t resources);

/** @This cleanups a device object. Memory is freed if the device has
    been allocated using the @ref device_alloc function. The device
    must not be attached or have attached children and its reference
    count must be zero when this function is called. */
void device_cleanup(struct device_s *dev);

/** @This reduces resource slots count to number of used slots
    count. The device node is reallocated to save memory. */
void device_shrink(struct device_s *dev);

/** @This returns the device tree root node. */
config_depend(CONFIG_DEVICE_TREE)
struct device_node_s *device_tree_root();

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

/** device lookup filter prototype */
#define DEVICE_FILTER(n) bool_t n(struct device_node_s *)

/** device lookup filter function type */
typedef DEVICE_FILTER(device_filter_t);

/** @internal @This lookup a node in the device tree. */
error_t device_node_from_path(struct device_node_s **node, const char *path,
                              uint_fast8_t depth, const char **brackets,
                              device_filter_t *filter);

/** @internal */
error_t device_resolve_alias(struct device_node_s **node, uint_fast8_t depth,
                             const char **brackets);

/** @internal */
DEVICE_FILTER(device_filter_init_done);

/** @This returns child device by path. The @tt node parameter must
    provide an initialized device pointer which will be updated is a
    matching device is found. The initial value is the search root
    node and may be @tt NULL to lookup from the device tree
    root. Multiple paths separated by spaces can be specified, the
    first match is retained.

    Paths with a leading @tt / are searched from the device tree root
    regardless of the @tt root parameter. The special name @tt .. and
    @tt . can be used to write relative paths. The @tt ? character can
    be used as a single character wildcard and the @tt * character can
    be used to match any node name suffix.

    The filter can be used to provide an additional filter
    function. It may be NULL.
 */
error_t device_get_by_path(struct device_s **dev, const char *path, device_filter_t *filter);

/** @This writes a null terminated device tree path in buffer. If the
    @tt number parameter is not 0, the value is appended at the end
    within a pair of square brackets. @return size of string excluding
    null byte or a negative error code. */
config_depend(CONFIG_DEVICE_TREE)
error_t device_get_path(struct device_node_s *root, char *buf,
                        size_t buflen, struct device_node_s *dev, uint_fast8_t number);

ALWAYS_INLINE struct device_s * device_from_node(struct device_node_s *node)
{
  return node && node->flags & DEVICE_FLAG_DEVICE ? (struct device_s*)node : NULL;
}

config_depend(CONFIG_DEVICE_TREE)
ALWAYS_INLINE struct device_alias_s * device_alias_from_node(struct device_node_s *node)
{
#ifdef CONFIG_DEVICE_TREE
  return node && node->flags & DEVICE_FLAG_ALIAS ? (struct device_alias_s*)node : NULL;
#else
  return NULL;
#endif
}

ALWAYS_INLINE struct device_node_s * device_to_node(struct device_s *dev)
{
  return &dev->node;
}

config_depend(CONFIG_DEVICE_TREE)
ALWAYS_INLINE struct device_node_s * device_node_from_alias(struct device_alias_s *alias)
{
#ifdef CONFIG_DEVICE_TREE
  return &alias->node;
#else
  return NULL;
#endif
}

/** @This sets the name of the device. The device name can only be
    changed if the device has not been attached to a parent node.

    The name string will be duplicated and later freed on cleanup only if
    the @ref DEVICE_FLAG_NAME_ALLOCATED flag of the device node is set. */
error_t device_set_name(struct device_s *dev, const char *name);

/** @This attaches a device to a parent enumerator device. If the
    parent device pointer is NULL, the device is attached on root enumerator. */
config_depend(CONFIG_DEVICE_TREE)
void device_attach(struct device_s *dev,
                   struct device_s *parent);

/** @This detaches a device from its parent enumerator device */
config_depend(CONFIG_DEVICE_TREE)
void device_detach(struct device_s *dev);

/** @see device_tree_walker_t */
#define DEVICE_TREE_WALKER(x) bool_t (x)(struct device_s *dev, void *priv)

/** @see device_tree_walk */
typedef DEVICE_TREE_WALKER(device_tree_walker_t);

/** @This traverse device tree calling @ref device_tree_walker_t
    function type foreach each node. Traversal stops if the provided
    function returns non-zero. */
bool_t device_tree_walk(struct device_node_s *root, device_tree_walker_t *walker, void *priv);

/** @This returns the number of processor devices present in the
    devices tree with a properly initialized driver attached. */
uint_fast8_t device_get_cpu_count();

#ifdef CONFIG_VMEM
uintptr_t vpage_io_map(paddr_t paddr, size_t size);
#endif

ALWAYS_INLINE
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

