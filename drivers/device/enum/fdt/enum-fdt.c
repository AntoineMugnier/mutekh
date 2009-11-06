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

    Copyright (c) 2009, Nicolas Pouillon <nipo@ssji.net>
*/


#include <hexo/types.h>

#include <device/enum.h>
#include <device/device.h>
#include <device/driver.h>

#include <mutek/mem_alloc.h>
#include <hexo/lock.h>
#include <hexo/interrupt.h>

#include <string.h>

#include <fdt/reader.h>

#include "enum-fdt.h"
#include "enum-fdt-private.h"



struct device_s *enum_fdt_get_at_offset(struct device_s *dev, uint32_t offset)
{
	CONTAINER_FOREACH_NOLOCK(device_list, CLIST, &dev->children, {
			struct enum_pv_fdt_s *enum_pv = item->enum_pv;
			if ( enum_pv->offset == offset )
				return item;
		});
	return NULL;
}

struct device_s *enum_fdt_lookup(struct device_s *dev, const char *path)
{
	struct enum_fdt_context_s *pv = dev->drv_pv;

	CONTAINER_FOREACH(fdt_node, CLIST, &pv->devices, {
			dprintk("[node %s] ", item->device_path);
			if ( !strcmp(item->device_path, path) )
				return item->dev;
		});
	return NULL;
}


error_t enum_fdt_register_one(struct device_s *dev, struct device_s *item)
{
	struct enum_pv_fdt_s *enum_pv = item->enum_pv;

	/* ignore already configured devices */
	if (item->drv != NULL)
		return 0;

	struct driver_s *drv = driver_get_matching_fdtname(enum_pv->device_type);

	if ( drv == NULL )
		return ENOTSUP;

	return enum_fdt_use_drv(dev, item, drv);
}

/*
 * device open operation
 */

const struct driver_s	enum_fdt_drv =
{
	.class		= device_class_enum,
	.f_init		= enum_fdt_init,
	.f_cleanup		= enum_fdt_cleanup,
	.f.denum = {
		.f_lookup = enum_fdt_lookup,
//		.f_register		= enum_fdt_register,
	}
};

static void *clone_blob( void *blob )
{
	size_t size = fdt_get_size(blob);
	if ( !size )
		return 0;
	void *b2 = mem_alloc(size, mem_region_get_local(mem_scope_sys));
	if ( b2 )
		memcpy(b2, blob, size);
	return b2;
}

extern struct device_s *console_dev;	

DEV_INIT(enum_fdt_init)
{
	struct enum_fdt_context_s *pv;

	dev->drv = &enum_fdt_drv;

	/* allocate private driver data */
	pv = mem_alloc(sizeof(*pv), mem_region_get_local(mem_scope_sys));

	if (!pv)
		return -1;

	pv->blob = clone_blob(params);
	dprintk("blob cloned from %p to %p\n", params, pv->blob);
	if ( !pv->blob ) {
		mem_free(pv);
		return -1;
	}

	fdt_node_init(&pv->devices);
	pv->console_path = NULL;

	dev->drv_pv = pv;

	dprintk("creating children\n");
	enum_fdt_create_children(dev);

	dprintk("registering drivers\n");
	CONTAINER_FOREACH(device_list, CLIST, &dev->children, {
			struct enum_pv_fdt_s *enum_pv = item->enum_pv;
			dprintk(" registering driver for %s\n", enum_pv->device_path);
			enum_fdt_register_one(dev, item);
		});

	if ( pv->console_path ) {
		struct device_s *cd = enum_fdt_lookup(dev, pv->console_path);
		if ( cd && cd->drv ) {
			printk("Setting console device to node %s\n", pv->console_path);
			console_dev = cd;
		}
	}

	return 0;
}


/*
 * device close operation
 */

DEV_CLEANUP(enum_fdt_cleanup)
{
	struct enum_fdt_context_s	*pv = dev->drv_pv;

	mem_free(pv->blob);
	mem_free(pv);
}

