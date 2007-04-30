/*
    This file is part of MutekH.

    MutekH is free software; you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    MutekH is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with MutekH; if not, write to the Free Software Foundation,
    Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA

    Copyright Alexandre Becoulet <alexandre.becoulet@lip6.fr> (c) 2006

*/

/*

    %config CONFIG_HEXO_DEVICE_TREE
    desc Have all device nodes sorted in a tree form
    default defined
    %config end

*/


#ifndef __DEVICE_H__
#define __DEVICE_H__

struct device_s;
struct driver_s;

#include <hexo/types.h>
#include <hexo/error.h>


/** Common class irq() function tempate. */
#define DEV_IRQ(n)	bool_t (n) (struct device_s *dev)

/** Common device class irq() function type. Must be called on
    interrupt request.

    * @param dev pointer to device descriptor
    * @return 1 if interrupt have been handled by the device
    */
typedef DEV_IRQ(dev_irq_t);




/** Common class init() function tempate. */
#define DEV_INIT(n)	error_t (n) (struct device_s *dev, struct device_s *icudev)

/** Common device class init() methode shortcut */
#define dev_init(dev) (dev)->drv->f_init(dev)

/** Common device class init() function type. Must be called before
    using any other functions on the device. This function will
    allocate device private data.

    * @param dev pointer to device descriptor
    * @return negative error code, 0 on succes
    */
typedef DEV_INIT(dev_init_t);




/** Common device class cleanup() function tempate. */
#define DEV_CLEANUP(n)	void    (n) (struct device_s *dev)

/** Common device class cleanup() methode shortcut */
#define dev_cleanup(dev) (dev)->drv->f_cleanup(dev)

/** Common device class cleanup() function type. Free all ressources
    allocated with the init() function.

   * @param dev pointer to device descriptor
   */
typedef DEV_CLEANUP(dev_cleanup_t);




/** Device descriptor structure */

#define DEVICE_MAX_ADDRSLOT	4


#ifdef CONFIG_HEXO_DEVICE_TREE

#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
#include <gpct/cont_clist.h>
#include <gpct/object_refcount.h>

OBJECT_TYPE(device_obj, REFCOUNT, struct device_s);

OBJECT_PROTOTYPE(device_obj, static inline, device_obj);

#endif

/** device object structure */

#ifdef CONFIG_HEXO_DEVICE_TREE

#define CONTAINER_LOCK_device_list HEXO_SPIN
#define CONTAINER_OBJ_device_list device_obj

CONTAINER_TYPE(device_list, CLIST,
#endif
struct device_s
{
#ifndef CONFIG_STATIC_DRIVERS

  const struct driver_s		*drv;
  struct device_s		*icudev;

#endif

  /** pointer to device driver private data if any */
  void				*drv_pv;

  /** hardware interrupt line number */
  int_fast8_t			irq;

  /** device IO addresses table */
  uintptr_t			addr[DEVICE_MAX_ADDRSLOT];

#ifdef CONFIG_HEXO_DEVICE_TREE
  /** pointer to device enumrator private data if any */
  void				*enum_pv;

  struct device_s		*parent;
  device_list_entry_t		list_entry;
  device_obj_entry_t		obj_entry;
  device_list_root_t		children;
#endif /* !CONFIG_HEXO_DEVICE_TREE */

}
#ifdef CONFIG_HEXO_DEVICE_TREE
, list_entry)
#endif
;

/* used when no irq line is present/available */
#define DEVICE_IRQ_INVALID	-1

#ifdef CONFIG_HEXO_DEVICE_TREE

OBJECT_CONSTRUCTOR(device_obj);
OBJECT_DESTRUCTOR(device_obj);

OBJECT_FUNC(device_obj, REFCOUNT, static inline, device_obj, obj_entry);

CONTAINER_PROTOTYPE(device_list, inline, device_list);

error_t device_register(struct device_s *dev,
			struct device_s *parent,
			void *enum_pv);

void device_dump_list(struct device_s *root);
void device_init(struct device_s *dev);

#endif /* !CONFIG_HEXO_DEVICE_TREE */

#ifdef CONFIG_HEXO_DEVICE_TREE
#define DEVICE_INITIALIZER							\
{										\
  .children = CONTAINER_ROOT_INITIALIZER(device_list, CLIST),			\
  .obj_entry = OBJECT_INITIALIZER(device_obj, REFCOUNT)				\
}
#else
#define DEVICE_INITIALIZER	{ }
#endif

#endif

