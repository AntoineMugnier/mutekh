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

struct initdev_state_s
{
	struct device_s *enum_dev;
	struct device_s *dev;
	const struct devenum_ident_s *ident;
	uint8_t *param;
	error_t err;
};


static void parse_icudev( struct device_s *enum_dev,
					   struct device_s *dev,
					   const uint32_t *data,
					   size_t datalen )
{
	if ( datalen < 4 )
		return;

	dprintk("Getting icudev with path %s... ", data);

	dev->icudev = enum_fdt_lookup(enum_dev, (const char*)data);

	dprintk("got %p\n", dev->icudev);
}

static void parse_reg( struct device_s *dev, const void *data, size_t datalen )
{
	struct enum_pv_fdt_s *pv = dev->enum_pv;
	uint_fast8_t i;
	const void *ptr = data;

	for ( i=0; i<DEVICE_MAX_ADDRSLOT; ++i ) {
		if ( ptr > (void*)((uintptr_t)data+datalen) )
			break;
		ptr = parse_sized( pv->addr_cells, ptr,
						   sizeof(dev->addr[i]), &dev->addr[i] );
		ptr = parse_sized( pv->size_cells, ptr,
						   0, NULL );
	}
}

static void parse_reg_size( struct device_s *dev, const void *data, size_t datalen )
{
	struct enum_pv_fdt_s *pv = dev->enum_pv;
	uint_fast8_t i;
	const void *ptr = data;

	for ( i=0; i<DEVICE_MAX_ADDRSLOT; i+=2 ) {
		if ( ptr > (void*)((uintptr_t)data+datalen) )
			break;
		ptr = parse_sized( pv->addr_cells, ptr,
						   sizeof(*dev->addr), &dev->addr[i] );
		ptr = parse_sized( pv->size_cells, ptr,
						   sizeof(*dev->addr), &dev->addr[i+1] );
		dev->addr[i+1] += dev->addr[i];
	}
}

static FDT_ON_NODE_ENTRY_FUNC(initdev_node_entry)
{
	struct initdev_state_s *priv = private;
	struct enum_pv_fdt_s *enum_pv = priv->dev->enum_pv;
	const struct driver_param_binder_s *binder = priv->ident->fdtname.binder;
	
	{
		const void *value = NULL;
		size_t len;

		if ( fdt_reader_has_prop(state, "irq", &value, &len ) )
			priv->dev->irq = endian_be32(*(uint32_t*)value);
		else
			priv->dev->irq = -1;

		if ( fdt_reader_has_prop(state, "icudev", &value, &len ) )
			parse_icudev( priv->enum_dev, priv->dev, value, len );
		else
			priv->dev->icudev = NULL;

		if ( fdt_reader_has_prop(state, "reg", &value, &len ) ) {
			if ( !strcmp(enum_pv->device_type, "memory") )
				parse_reg_size( priv->dev, value, len );
			else
				parse_reg( priv->dev, value, len );
		}
	}

	/*
	  If the icu controller is not initialized yet, try to do it now
	 */
	if ( priv->dev->icudev && !priv->dev->icudev->drv ) {
		error_t err = 
			enum_fdt_register_one(priv->enum_dev, priv->dev->icudev);
		if (err) {
			priv->err = err;
			return 0;
		}
	}

	if ( binder ) {
		dprintk("  has a binder\n");
		for ( ; binder->param_name; binder++ ) {
			const void *value = NULL;
			size_t len;
			dprintk("   considering parameter %s... ", binder->param_name);
			if ( fdt_reader_has_prop(state, binder->param_name, &value, &len ) ) {
				dprintk("%P\n", value, len);
				switch (binder->datatype) {
				case PARAM_DATATYPE_BOOL:
					*(bool_t*)(priv->param + binder->struct_offset) = 1;
					break;
				case PARAM_DATATYPE_INT:
					parse_sized( binder->datalen, value,
								 binder->datalen, priv->param + binder->struct_offset );
					break;
				case PARAM_DATATYPE_DEVICE_PTR:
					*(struct device_s **)(priv->param + binder->struct_offset) =
						enum_fdt_get_at_offset(
							priv->enum_dev,
							endian_be32(*(uint32_t*)value));
					break;
				case PARAM_DATATYPE_ADDR:
					parse_sized( binder->datalen, value,
								 binder->datalen, priv->param + binder->struct_offset );
					break;
				}
			} else {
				dprintk("no value\n");
				switch (binder->datatype) {
				case PARAM_DATATYPE_BOOL:
					*(bool_t*)(priv->param + binder->struct_offset) = 0;
					break;
				default:
#if 1
					printk("  Warning: parameter %s not found for device %s, trying without\n", binder->param_name, enum_pv->device_path);
#else
					priv->err = ENOENT;
					return 0;
#endif
				}
			}
		}
	}

	return 0;
}

error_t enum_fdt_use_drv(
	struct device_s *enum_dev,
	struct device_s *dev,
	struct driver_s *drv)
{
	struct enum_pv_fdt_s *enum_pv = dev->enum_pv;
	struct enum_fdt_context_s *pv = enum_dev->drv_pv;
	const struct devenum_ident_s *ident = drv->id_table;

	assert(ident);

	dprintk(" Considering usage of driver %p for %s\n",
		   drv,
		   enum_pv->device_path);


	for ( ; ident->type != 0; ident++ ) {
		if ( (ident->type == DEVENUM_TYPE_FDTNAME)
			 && !strcmp(ident->fdtname.name, enum_pv->device_type) )
			break;
	}

	assert( !strcmp(ident->fdtname.name, enum_pv->device_type) );

	uint8_t param[ident->fdtname.param_size];

	dprintk("  param size: %d\n", ident->fdtname.param_size);

	struct initdev_state_s priv = {
		.enum_dev = enum_dev,
		.dev = dev,
		.ident = ident,
		.param = param,
		.err = 0,
	};

	struct fdt_walker_s walker = {
		.private = &priv,
		.on_node_entry = initdev_node_entry,
		.on_node_leave = NULL,
		.on_node_prop = NULL,
		.on_mem_reserve = NULL,
	};

	const char *reason = "walking";
	error_t err = fdt_walk_blob_from(pv->blob, &walker, enum_pv->offset);

	if ( !err ) {
		reason = "inside";
		err = priv.err;
	}

	if ( !err ) {
		reason = "driver";
		err = drv->f_init(dev, param);
	}
	printk("Initialization of device %s with driver %p %s %s: %d\n",
		   enum_pv->device_path,
		   drv,
		   reason,
		   err ? "failed" : "succeded",
		   err);
	return err;
}

