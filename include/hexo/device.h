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


#ifndef DEVICE_H
#define DEVICE_H

struct device_s;

#include "types.h"




/** Common class irq() function tempate. */
#define DEV_IRQ(n)	__bool_t (n) (struct device_s *dev)

/** Common device class irq() function type. Must be called on
    interrupt request.

    * @param dev pointer to device descriptor
    * @return 1 if interrupt have been handled by the device
    */
typedef DEV_IRQ(dev_irq_t);




#include "device/char.h"
#include "device/icu.h"
#include "device/fb.h"
#include "device/timer.h"
#include "device/enum.h"





/** Common class init() function tempate. */
#define DEV_INIT(n)	error_t (n) (struct device_s *dev)

/** Common device class init() methode shortcut */
//#define dev_init(dev) (dev)->f_init(dev)

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
#define dev_cleanup(dev, ...) (dev)->f_cleanup(dev, __VA_ARGS__ )

/** Common device class cleanup() function type. Free all ressources
    allocated with the init() function.

   * @param dev pointer to device descriptor
   */
typedef DEV_CLEANUP(dev_cleanup_t);




/** Device descriptor structure */

#define DEVICE_MAX_ADDRSLOT	4


#ifdef CONFIG_DEVICE_HIERARCHY
#include <hexo/template/lock_spin.h>
#include <hexo/template/cont_dlist.h>
CONTAINER_TYPE_DECL(device, DLIST, struct device_s, SPIN);
#endif

struct device_s
{
#ifndef CONFIG_STATIC_DRIVERS

  dev_cleanup_t			*f_cleanup;
  dev_irq_t			*f_irq;

  union {
    /** char devices */
    struct dev_class_char_s	chr;
    /** icu devices */
    struct dev_class_icu_s	icu;
    /** frame buffer devices */
    struct dev_class_fb_s	fb;
    /** timer devices */
    struct dev_class_timer_s	timer;
    /** device enumerator class */
    struct dev_class_enum_s	denum;
  };

#endif

  /** pointer to device driver private data if any */
  void				*drv_pv;

  /** hardware interrupt line number */
  uint_fast8_t			irq;

  /** device IO addresses table */
  uintptr_t			addr[DEVICE_MAX_ADDRSLOT];

#ifdef CONFIG_DEVICE_HIERARCHY
  /** pointer to device enumrator private data if any */
  void				*enum_pv;

  struct device_s		*parent;
  device_entry_t		siblings;
  device_cont_t			children;
#endif /* !CONFIG_DEVICE_HIERARCHY */

};


#ifdef CONFIG_DEVICE_HIERARCHY

error_t device_init(struct device_s *dev);

error_t device_register(struct device_s *dev,
			struct device_s *parent,
			void *enum_pv);

void device_dump_list(struct device_s *root);

#endif /* !CONFIG_DEVICE_HIERARCHY */

#endif

