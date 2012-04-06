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

    Copyright (c) Nicolas Pouillon <nipo@ssji.net> 2009
*/

#include <hexo/types.h>

#include <device/class/mem.h>
#include <device/device.h>
#include <device/driver.h>

#include "memory.h"

DEVMEM_GET_INFO(memory_get_info)
{
	struct device_s *dev = mdev->dev;
	uint32_t flags = (uint32_t)dev->drv_pv;

	uint_fast8_t i;

	if (!device_res_id(dev, DEV_RES_MEM, 0, &i))
		{
			info->base = dev->res[i].mem.start;
			info->size = dev->res[i].mem.end - dev->res[i].mem.start;
			info->flags = flags;
		}
}

static const struct devenum_ident_s	memory_ids[] =
{
	DEVENUM_FDTNAME_ENTRY("memory"),
	{ 0 }
};

static const struct driver_mem_s   memory_mem_drv =
{
	.class_     = DEVICE_CLASS_MEM,
	.f_get_info = memory_get_info,
};

const struct driver_s   memory_drv =
{
	.id_table	= memory_ids,
	.f_init		= memory_init,
	.f_cleanup	= memory_cleanup,
	.classes	= { &memory_mem_drv, 0 }
};

REGISTER_DRIVER(memory_drv);

DEV_INIT(memory_init)
{
	dev->drv = &memory_drv;

	uint32_t flags = 0;
#warning FIXME
	flags |= DEV_MEM_CACHED|DEV_MEM_COHERENT;
	flags |= DEV_MEM_CACHED;

	dev->drv_pv = (void*)flags;

	return 0;
}

DEV_CLEANUP(memory_cleanup)
{
}

