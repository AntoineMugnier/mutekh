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

    Copyright Nicolas Pouillon <nipo@ssji.net> (c) 2009
*/

/**
 * @file
 * @module{Devices support library}
 * @short Memory device driver API
 */

#ifndef __DEVICE_MEM_H__
#define __DEVICE_MEM_H__

#include <hexo/types.h>
#include <hexo/error.h>

#include <device/driver.h>

struct device_s;
struct driver_s;
struct device_mem_s;
struct driver_mem_s;

#define DEV_MEM_CACHED   1
#define DEV_MEM_COHERENT 2

struct dev_mem_info_s
{
	paddr_t base;
	paddr_t size;
	uint16_t flags;
};


/** Mem device class get_info() function tempate. */
#define DEVMEM_GET_INFO(n)	void  (n) (struct device_mem_s *mdev, struct dev_mem_info_s *info)

/**
   Mem device class get_info() function type. Get information about a ram device.

   @param dev pointer to device descriptor
   @param info pointer to a yet-to-fill info structure
*/
typedef DEVMEM_GET_INFO(devmem_get_info_t);


DRIVER_CLASS_TYPES(mem,
                   devmem_get_info_t *f_get_info;
                   );


#endif

