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
 * @module {Core::Devices support library}
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
typedef uint16_t device_enum_rev_t;

#include <gct_platform.h>
#include <gct_lock_hexo_lock.h>

#ifdef CONFIG_DEVICE_TREE
# include <gct/container_clist.h>
#endif

#ifdef CONFIG_DEVICE_SLEEP
# include <gct/container_slist.h>
/** @see device_sleep_schedule */
enum device_sleep_policy_e
{
  /** When this policy is in use, the @ref DEV_USE_SLEEP driver
      operation is delayed until all processors in the platform become
      idle. */
  DEVICE_SLEEP_CPUIDLE,
};
#endif

ENUM_DESCRIPTOR(device_status_e, strip:DEVICE_, upper);

/** @This specifies the initialization status of a @ref device_s
    @xsee {Device status} */
enum device_status_e
{
  /** Device enumeration incomplete, some resource entries may be
      wrong or missing. */
  DEVICE_INIT_ENUM_ERR,
  /** No driver is currently attached to the device but the driver
      registry will be searched for an appropriate driver. */
  DEVICE_INIT_ENUM_DRV,
  /** No driver is currently attached to the device. */
  DEVICE_INIT_NODRV,
  /** A driver has been attached to the device but initialization has
      not been performed yet. Initialization will start as soon as all
      dependencies are satisfied. It is not possible to get a device
      accessor at this point. */
  DEVICE_INIT_PENDING,
  /** The driver initialization is ongoing. Some classes of the driver
      may already be usable. This can only result from the @ref
      dev_init_t function returning @tt -EAGAIN. It is possible to get
      a device accessor for the initialized classes at this point if
      the driver supports partial initialization. */
  DEVICE_INIT_ONGOING,
  /** Some classes of the driver have not been properly initialized
      but other classes are usable. This occurs when the @ref
      device_init_enable_api function is called during an
      initialization which eventually terminate with an error. */
  DEVICE_INIT_PARTIAL,
  /** The device is fully initialized and functional. This results
      from the @ref dev_init_t function returning @tt 0 or by invoking
      the @ref device_async_init_done function. */
  DEVICE_INIT_DONE,
  /** The driver cleanup is ongoing. This can only result from the
      @ref dev_cleanup_t function returning @tt -EAGAIN. It is
      not possible to get a device accessor anymore. */
  DEVICE_INIT_DECLINE,
  /** A driver is attached to the device but the initialization has
      failed or a dependency is not available. It is not possible to
      get a device accessor but the device initialization may be
      retried. */
  DEVICE_INIT_FAILED,
  /** The device was previously initialized but has been explicitely
      released. This is similar to @ref DEVICE_INIT_PENDING but
      the initialization will not be performed automatically. */
  DEVICE_INIT_RELEASED,
};

/** @This specifies @ref device_node_s type and flags */
enum device_flags_e
{
  /** @internal The device object has been dynamically allocated and
      must be freed on cleanup. */
  DEVICE_FLAG_ALLOCATED = 1,
  /** @internal The node name has been dynamically allocated and must
      be freed on cleanup. */
  DEVICE_FLAG_NAME_ALLOCATED = 2,

  /** This device is a processor. Operations on this device
      may only be executed on the corresponding processor. */
  DEVICE_FLAG_CPU = 4,
  /** @internal The tree node is a device node */
  DEVICE_FLAG_DEVICE = 8,
  /** @internal The tree node is an alias node */
  DEVICE_FLAG_ALIAS = 16,
  /** Automatic binding of a driver to the device will not be performed,
      the @ref device_bind_driver function must be called explicitly. */
  DEVICE_FLAG_NO_AUTOBIND = 32,
  /** Automatic initialization of the device will not be performed,
      the @ref device_init_driver function must be called explicitly. */
  DEVICE_FLAG_NO_AUTOINIT = 64,
  /** Do not wait for initialization of this device before starting
      execution of the @tt INIT_DEVREADY_INIT group during startup. */
  DEVICE_FLAG_NO_STARTUP_WAIT = 128,
};

#define GCT_CONTAINER_ALGO_device_list CLIST

#define GCT_CONTAINER_ALGO_device_sleep SLIST
#define GCT_CONTAINER_LOCK_device_sleep HEXO_LOCK
#define GCT_CONTAINER_ORPHAN_device_sleep

#ifdef CONFIG_DEVICE_TREE
GCT_CONTAINER_TYPES(device_list,
#endif
/** @internal Device tree base node structure
    @xsee {Device tree} */
struct device_node_s
{
  /** device name, freed on device object destruction if not NULL and
      @tt allocated is set. */
  const char *                  name;

#ifdef CONFIG_DEVICE_TREE
  struct device_node_s		*parent;
  GCT_CONTAINER_ENTRY           (device_list, list_entry);
  device_list_root_t		children;
#endif

  enum device_flags_e           BITFIELD(flags,8);
} __attribute__((packed,aligned(4)))
#ifdef CONFIG_DEVICE_TREE
 *, list_entry);

GCT_CONTAINER_FCNS(device_list, inline, device_list,
                   init, destroy, pushback, remove, isempty);
#endif
;

/** device node structure
    @xcsee {Device instance} */
struct device_s
{
  /* @internal must be first field */
  struct device_node_s          node;

  /** Device/driver initialization status @xsee {Device status} */
  enum device_status_e          BITFIELD(status,4);

  /** @internal When the @ref status is @ref DEVICE_INIT_ONGOING or
      @ref DEVICE_INIT_PARTIAL, this is a mask of initialized classes
      in driver API order. Extra bits can be used by the driver in
      order to flag internal initialization states.  @see
      device_init_enable_apies */
#ifdef CONFIG_DEVICE_INIT_PARTIAL
  uint8_t                       BITFIELD(init_mask,8);
#endif

  /** device uses counter, @see {device_get_accessor} */
  uint16_t                      BITFIELD(ref_count,CONFIG_DEVICE_USE_BITS);

  /** device start counter, @see {device_start} */
  uint16_t                      BITFIELD(start_count,CONFIG_DEVICE_USE_BITS
                                         + CONFIG_DEVICE_START_LOG2INC);

#ifdef CONFIG_DEVICE_SLEEP
  /** @internal */
  uint_fast8_t                  sleep_order;
  /** @internal */
  enum device_sleep_policy_e    sleep_policy;
  /** @internal */
  GCT_CONTAINER_ENTRY           (device_sleep, sleep_queue_entry);
#endif

  /** general purpose device lock */
  lock_t			lock;

  /** pointer to device driver if any */
  const struct driver_s		*drv;
  /** pointer to device driver private data */
  void				*drv_pv;

#ifdef CONFIG_DEVICE_ENUM
  /** enumerator list revision */
  device_enum_rev_t             enum_rev;
  /** pointer to device enumerator private data if any */
  void				*enum_pv;
#endif

  /** device resources table */
  struct dev_resource_table_s   *res_tbl;
};

#ifdef CONFIG_DEVICE_SLEEP
GCT_CONTAINER_TYPES(device_sleep, struct device_s *, sleep_queue_entry);
GCT_CONTAINER_KEY_TYPES(device_sleep, PTR, SCALAR, sleep_order);
GCT_CONTAINER_FCNS(device_sleep, ALWAYS_INLINE, device_sleep_queue,
                   orphan, isorphan);
#endif

/** @This schedules a call to the @ref dev_use_t function of the
    device driver so that the device power consumption can be reduced.

    This function should be called when the driver notice that the
    device is not in active use anymore and is candidate for being put
    in low power mode.

    When the @ref dev_use_t function is called, the driver then have
    to take the required actions to reduce device power usage. See
    @cref DEV_USE_SLEEP for details. This call is delayed depending on
    the current device sleep policy.

    This function must be called with the device lock held.
    It is harmless to call this function multiple times.
    @see device_sleep_policy_e
    @csee #CONFIG_DEVICE_SLEEP
*/
config_depend(CONFIG_DEVICE_SLEEP)
void device_sleep_schedule(struct device_s *dev);

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

    @xcsee{Device instance}
    @see dev_resource_type_e
*/
# define DEV_DECLARE_STATIC(declname_, name_, flags_, driver_, resources_...) \
    extern const struct driver_s driver_;                               \
    __attribute__ ((aligned (sizeof(void*))))                           \
    __attribute__((section (".devices."#declname_)))                    \
    struct device_s declname_ = {                                       \
      .node = {                                                         \
        .flags = flags_ | DEVICE_FLAG_DEVICE,                           \
        .name = name_,                                                  \
        .parent = NULL,                                                 \
        .children = { .ht = { .next = &declname_.node.children.ht,      \
                              .prev = &declname_.node.children.ht } },  \
      },                                                                \
      .lock = LOCK_INITIALIZER,                                         \
      .status = DEVICE_INIT_PENDING,                             \
      .drv = &driver_,                                                  \
      .ref_count = 0,                                                   \
      .res_tbl = ARRAY_SIZE(DEV_STATIC_RESOURCES_ARRAY(resources_))     \
      ? (struct dev_resource_table_s *)DEV_STATIC_RESOURCES(resources_) \
      : NULL,                                                           \
    }
#else
# define DEV_DECLARE_STATIC(declname_, name_, flags_, driver_, resources_...) \
    extern const struct driver_s driver_;                               \
    __attribute__ ((aligned (sizeof(void*))))                           \
    __attribute__((section (".devices."#declname_)))                    \
    struct device_s declname_ = {                                       \
      .node = {                                                         \
        .flags = flags_ | DEVICE_FLAG_DEVICE,                           \
        .name = name_,                                                  \
      },                                                                \
      .lock = LOCK_INITIALIZER,                                         \
      .status = DEVICE_INIT_PENDING,                             \
      .drv = &driver_,                                                  \
      .ref_count = 0,                                                   \
      .res_tbl = ARRAY_SIZE(DEV_STATIC_RESOURCES_ARRAY(resources_))     \
      ? (struct dev_resource_table_s *)DEV_STATIC_RESOURCES(resources_) \
      : NULL,                                                           \
    }
#endif


#ifdef CONFIG_DEVICE_TREE
/** device alias node structure
    @xsee{Device tree} */
struct device_alias_s
{
  /* must be first field */
  struct device_node_s          node;

  /** alias path in device tree */
  const char *path;
};
#endif

#ifdef CONFIG_DEVICE_TREE
/** @This iterates over child nodes of a device tree node.
    @xsee{Device tree} */
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

/** @This initializes a statically allocated device object. Devices
    declared using the @ref #DEV_DECLARE_STATIC macro do not require
    use of this function.

    @xsee {Device instance} */
void device_init(struct device_s *dev, const struct dev_resource_table_s *tbl);

/** @This dynamically allocates and initializes a device object. The
    specified number of resource slots are pre-allocated.

    @see {device_cleanup, device_shrink}
    @xcsee {Device instance} */
struct device_s *device_alloc(size_t resources);

/** @This release memory used by a device. This includes device object
    allocated by the @ref device_alloc function along with resources
    entries. The device must not be attached or have attached children
    and its reference count must be zero when this function is
    called.
    @xcsee {Device instance} */
config_depend(CONFIG_DEVICE_CLEANUP)
void device_cleanup(struct device_s *dev);

/** @This reduces resource slots count to number of used slots
    count. The device node is reallocated to save memory.
    @xsee {Device instance}
    @xsee {Device resources} */
config_depend(CONFIG_DEVICE_RESOURCE_ALLOC)
void device_shrink(struct device_s *dev);

/** @This returns the device tree root node.
    @xsee {Device tree} */
config_depend(CONFIG_DEVICE_TREE)
struct device_node_s *device_tree_root(void);

/** @This creates an alias in the device tree. The parent parameter
    may be @tt NULL to attach the alias on the tree root.
    @see device_new_alias_to_node
    @xsee {Device tree} */
config_depend(CONFIG_DEVICE_TREE)
struct device_alias_s * device_new_alias_to_path(struct device_node_s *parent, const char *name, const char *path);

/** @This creates an alias in the device tree. The target string is
    obtained using the @ref device_get_path function. The parent
    parameter may be @tt NULL to attach the alias on the tree root.
    @see device_new_alias_to_path
    @xsee {Device tree} */
config_depend(CONFIG_DEVICE_TREE)
struct device_alias_s * device_new_alias_to_node(struct device_node_s *parent, const char *name, struct device_node_s *node);

/** @xsee {Device tree} */
config_depend(CONFIG_DEVICE_TREE)
void device_alias_remove(struct device_alias_s *alias);

/** @internal device lookup filter prototype
    @see device_filter_t */
#define DEVICE_FILTER(n) bool_t n(struct device_node_s *)

/** @internal device lookup filter function type */
typedef DEVICE_FILTER(device_filter_t);

/** @internal @This lookup a node in the device tree. */
error_t device_node_from_path(struct device_node_s **node, const char *path,
                              uint_fast8_t depth, const char **brackets,
                              device_filter_t *filter);

/** @internal */
error_t device_resolve_alias(struct device_node_s **node, uint_fast8_t depth,
                             const char **brackets);

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
    @xcsee {Device tree} */
error_t device_get_by_path(struct device_s **dev, uint_fast8_t *number,
                           struct device_node_s *root, const char *path,
                           device_filter_t *filter);

/** @This writes a null terminated device tree path in buffer. If the
    @tt number parameter is not 0, the value is appended at the end
    within a pair of square brackets. @return size of string excluding
    null byte or a negative error code.
    @xsee {Device tree} */
config_depend(CONFIG_DEVICE_TREE)
error_t device_get_path(struct device_node_s *root, char *buf,
                        size_t buflen, struct device_node_s *dev, uint_fast8_t number);

/** @internal */
ALWAYS_INLINE struct device_s * device_from_node(struct device_node_s *node)
{
  return node && node->flags & DEVICE_FLAG_DEVICE ? (struct device_s*)node : NULL;
}

/** @internal */
config_depend(CONFIG_DEVICE_TREE)
ALWAYS_INLINE struct device_alias_s * device_alias_from_node(struct device_node_s *node)
{
#ifdef CONFIG_DEVICE_TREE
  return node && node->flags & DEVICE_FLAG_ALIAS ? (struct device_alias_s*)node : NULL;
#else
  return NULL;
#endif
}

/** @internal */
ALWAYS_INLINE struct device_node_s * device_to_node(struct device_s *dev)
{
  return &dev->node;
}

/** @internal */
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
    the @ref DEVICE_FLAG_NAME_ALLOCATED flag of the device node is set.
    @xcsee {Device instance} */
error_t device_set_name(struct device_s *dev, const char *name);

/** @This attaches a device to a parent enumerator device. If the
    parent device pointer is NULL, the device is attached on root
    enumerator.
    @xcsee {Device tree} */
config_depend(CONFIG_DEVICE_TREE)
void device_attach(struct device_s *dev,
                   struct device_s *parent,
                   const struct driver_s *drv);

/** @This detaches a device from its parent enumerator device
    @xcsee {Device tree} */
config_depend(CONFIG_DEVICE_TREE)
error_t device_detach(struct device_s *dev);

/** @see device_tree_walker_t */
#define DEVICE_TREE_WALKER(x) bool_t (x)(struct device_s *dev, void *priv)

/** @see device_tree_walk */
typedef DEVICE_TREE_WALKER(device_tree_walker_t);

/** @This traverse device tree calling @ref device_tree_walker_t
    function type foreach each node. Traversal stops if the provided
    function returns non-zero.
    @xsee {Device tree} */
bool_t device_tree_walk(struct device_node_s *root, device_tree_walker_t *walker, void *priv);

/** @This returns the number of processor devices present in the
    devices tree with a properly initialized driver attached. */
uint_fast8_t device_get_cpu_count(void);

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

