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

#ifndef __DEVICE_ENUM_H__
#define __DEVICE_ENUM_H__

#include <hexo/types.h>
#include <hexo/error.h>

struct device_s;
struct driver_s;

/** Enum device class register() function tempate. */
#define DEVENUM_REGISTER(n)	size_t (n) (struct device_s *dev,	\
					    const struct driver_s *drv)


/** Enum device class register() methode shortcut */

#define dev_enum_register(dev, ...) (dev)->drv->f.denum.f_register(dev, __VA_ARGS__ )
/**
   Enum device class register() function type.
   Try to bind a driver

   @param dev pointer to enumerator device descriptor
   @param drv pointer pointer to driver to be used for matching devices
   @return registered devices count
*/
typedef DEVENUM_REGISTER(devenum_register_t);



/** Enum device class methodes */
struct dev_class_enum_s
{
  devenum_register_t		*f_register;
};


#endif

