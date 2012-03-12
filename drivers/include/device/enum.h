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
 * @short Enumerator driver API
 */                                                                 

#ifndef __DEVICE_ENUM_H__
#define __DEVICE_ENUM_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>

/**
   Lookup function prototype macro
 */
#define DEVENUM_LOOKUP(x) struct device_s *(x)(struct device_s *dev, const char *path)

/**
   Lookup function prototype. Lookup a device inside an enumerated
   device. The path parameter depends on the type of enumerator.

   @param dev The device to lookup from
   @param path The device to lookup for in dev
   @return a pointer to the found device, or NULL
 */
typedef DEVENUM_LOOKUP(devenum_lookup_t);


#define DEV_ENUM_MAX_PATH_LEN 32

struct dev_enum_info_s
{
    char path[DEV_ENUM_MAX_PATH_LEN];
};


/**
   Info function prototype macro
 */
#define DEVENUM_INFO(x) error_t (x)(struct device_s *dev,              \
                                    struct device_s *child,            \
                                    struct dev_enum_info_s *info)

/**
   Info function prototype. Queries information about a child of a
   given device.

   @param dev The device to info from
   @param child The child of @tt dev to query information about
   @param info The caller-allocated information structure to return
   information in
   @return 0 if done, or an error.
 */
typedef DEVENUM_INFO(devenum_info_t);


struct driver_enum_s
{
  enum device_class_e class_;
  devenum_lookup_t *f_lookup;
  devenum_info_t *f_info;
};

#endif
