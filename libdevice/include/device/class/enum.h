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
 * @module{Devices support library}
 * @short Enumerator driver API
 */                                                                 

#ifndef __DEVICE_ENUM_H__
#define __DEVICE_ENUM_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>

struct device_enum_s;

/** @see dev_enum_match_driver_t */
#define DEV_ENUM_MATCH_DRIVER(n) bool_t (n)(struct device_enum_s *accessor, \
                                            const struct dev_enum_ident_s *ident, \
                                            size_t count, struct device_s *dev)

/** @This determines if the @tt drv driver is suitable to drive the
    @tt dev device. The device must have been enumerated by the @tt
    accessor device. @This generally relies on the enumeration ids table
    provided by the driver. */
typedef DEV_ENUM_MATCH_DRIVER(dev_enum_match_driver_t);

DRIVER_CLASS_TYPES(DRIVER_CLASS_ENUM, enum,
                   dev_enum_match_driver_t *f_match_driver;
                   );

/** @see driver_enum_s */
#define DRIVER_ENUM_METHODS(prefix)                               \
  ((const struct driver_class_s*)&(const struct driver_enum_s){   \
    .class_ = DRIVER_CLASS_ENUM,                                  \
    .f_match_driver = prefix ## _match_driver,                    \
  })

#endif

