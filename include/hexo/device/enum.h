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


#if !defined(DEVICE_H) || defined(DEVICE_ENUM_H_)
#error This file can not be included directly
#else

#define DEVICE_ENUM_H_

#include "../types.h"
#include "../error.h"


/** Enum device class register() function tempate. */
#define DEVENUM_REGISTER(n)	struct device_s * (n) (struct device_s *dev, struct driver_s *drv, \
						       uint_fast16_t id[], size_t idlen)

/** Enum device class register() methode shortcut */

#define dev_enum_register(dev, ...) (dev)->drv->f.denum.f_register(dev, __VA_ARGS__ )
/**
   Enum device class register() function type.
   Try to bind a driver

   @param dev pointer to enumerator device descriptor
   @param id pointer to identifier structure
   @param idlen size of the identifier structure
   @return pointer to first matching device
*/
typedef DEVENUM_REGISTER(devenum_register_t);



/** Enum device class methodes */
struct dev_class_enum_s
{
  devenum_register_t		*f_register;
};


#endif

