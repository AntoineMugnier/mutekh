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

struct dev_irq_ep_s;

#define DEV_IRQ_EP_PROCESS(n) bool_t (n) (struct dev_irq_ep_s *ep, int_fast8_t id)
typedef DEV_IRQ_EP_PROCESS(dev_irq_ep_process_t);

/** Device irq end-point object. Irq source and sink endpoints are
    linked together to make irqs topology graph */
struct dev_irq_ep_s
{
  /** Irq event handling function for endpoint */
  dev_irq_ep_process_t *process;

  /** Number of links */
  uint_fast8_t links_count;

  union {
    /** Single link case */
    struct dev_irq_ep_s *single;

    /** Multiple links case */
    struct dev_irq_ep_s **array;
  }  /** For source ep: list of sink ep which can recieve the irq signal,
         for sink ep: list of source ep which can relay this irq */
    links;

  /** For source ep: link to device which may raise irq,
      for sink ep: link to device which can handle irq. */
  struct device_s   *dev;
};


/** Number of resource slots for statically allocated @ref device_s objects */
#define DEVICE_STATIC_RESOURCE_COUNT	2

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>

#ifdef CONFIG_DEVICE_TREE

#include <gpct/cont_clist.h>
#include <gpct/object_refcount.h>

#endif

enum dev_resource_type_e
{
    DEV_RES_UNUSED = 0,         //< Unused resource slot
    DEV_RES_MEM,                //< Physical memory address mapping resource
    DEV_RES_IO,                 //< Io space address mapping resource
    DEV_RES_IRQ,                //< Interrupt line resource
    DEV_RES_ID,                 //< Unique numeric id, meaning depends on parent device type
    DEV_RES_VENDORID,           //< Vendor id, meaning depends on parent device type
    DEV_RES_DEVICEID,           //< Device id specific to current vendor id
};

struct dev_resource_s
{
  uint16_t type;                // resource descriptor type @see dev_resource_type_e
  union {
    uintptr_t uint;
    struct {
      uintptr_t start;
      uintptr_t end;
    }   mem;

    struct {
      uintptr_t start;
      uintptr_t end;
    }   io;

    struct {
      uintptr_t id;         //< irq number given by device enumerator
      struct dev_irq_ep_s *ep; //< associated irq end point, may be NULL
    }   irq;

    struct {
      uintptr_t id;          //< dynamic numeric id
    }   id;

    struct {
      uintptr_t id;          //< optional vendor numeric id, may be -1
      const char *name;     //< optional vendor string id, may be NULL
    }   vendorid;

    struct {
      uintptr_t id;          //< optional device numeric id, may be -1
      const char *name;     //< optional device string, may be NULL
    }   deviceid;
  };
};

error_t device_res_id(const struct device_s *dev,
                      enum dev_resource_type_e type,
                       uint_fast8_t id, uint_fast8_t *res);

error_t device_res_get_uint(const struct device_s *dev,
                            enum dev_resource_type_e type,
                            uint_fast8_t id, uintptr_t *res);

struct dev_resource_s * device_res_add(struct device_s *dev);

error_t device_res_add_io(struct device_s *dev, uintptr_t start, uintptr_t end);
error_t device_res_add_mem(struct device_s *dev, uintptr_t start, uintptr_t end);
error_t device_res_add_irq(struct device_s *dev, uintptr_t irq);

enum device_status_e
{
  DEVICE_NO_DRIVER,
  DEVICE_DRIVER_INIT_PENDING,
  DEVICE_DRIVER_INIT_DONE,
  DEVICE_DRIVER_INIT_FAILED,
};

#ifdef CONFIG_DEVICE_TREE

#define CONTAINER_LOCK_device_list HEXO_SPIN

CONTAINER_TYPE(device_list, CLIST,
#endif
/** device object structure */
struct device_s
{
  /** general purpose device lock */
  lock_t			lock;

  /** pointer to device driver private data if any */
  void				*drv_pv;

  /** device resources table */
  uint_fast8_t                  res_count;

#ifdef CONFIG_DEVICE_TREE
  /** pointer to device enumerator private data if any */
  void				*enum_pv;
  uint_fast8_t                  enum_type; //< type of enumerator @see dev_enum_type_e

  struct device_s		*parent;
  device_list_entry_t		list_entry;
  device_list_root_t		children;
  uint_fast8_t                  ref_count;
  bool_t                        allocated;
#endif /* !CONFIG_DEVICE_TREE */

  /** Set to true if driver initialization done */
  enum device_status_e          status;
  const struct driver_s		*drv;

  /** device resources table */
  struct dev_resource_s         res[DEVICE_STATIC_RESOURCE_COUNT];
}
#ifdef CONFIG_DEVICE_TREE
, list_entry)
#endif
;

#ifdef CONFIG_DEVICE_TREE

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

CONTAINER_PROTOTYPE(device_list, inline, device_list);

/** @This attaches a device to a parent enumerator device. If the
    parent device pointer is NULL, the device is attached on root enumerator. */
void device_attach(struct device_s *dev,
                      struct device_s *parent);

/** @This detaches a device from its parent enumerator device */
void device_detach(struct device_s *dev);

/** @This prints the current devices tree. */
void device_dump_list(struct device_s *root);

#define DEVICE_TREE_WALKER(x) void (x)(struct device_s *dev, void *priv)

typedef DEVICE_TREE_WALKER(device_tree_walker_t);

void device_tree_walk(device_tree_walker_t *walker, void *priv);

#endif /* !CONFIG_DEVICE_TREE */

void device_init(struct device_s *dev);
struct device_s *device_get_child(struct device_s *dev, uint_fast8_t i);

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


#endif

