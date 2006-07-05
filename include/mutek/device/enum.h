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




/** Enum device class find() function tempate. */
#define DEVENUM_FIND(n)	struct device_s * (n) (struct device_s *dev, void *id, size_t idlen)

/** Enum device class find() methode shortcut */

#define dev_enum_find(dev, ...) (dev)->chr.f_find(dev, __VA_ARGS__ )
/**
   Enum device class find() function type.
   Find an enumerated device matching the bus specific identifier

   @param dev pointer to enumerator device descriptor
   @param id pointer to identifier structure
   @param idlen size of the identifier structure
   @return pointer to first matching device
*/
typedef DEVENUM_FIND(devenum_find_t);



/** Enum device class methodes */
struct dev_class_enum_s
{
  devenum_find_t		*f_find;
};


#endif

