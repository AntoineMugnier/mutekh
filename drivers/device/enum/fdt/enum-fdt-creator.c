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
#include <hexo/endian.h>

#include <device/enum.h>
#include <device/device.h>
#include <device/driver.h>

#include <mutek/mem_alloc.h>

#include <string.h>

#include <fdt/reader.h>
#include <mutek/printk.h>

#include "enum-fdt.h"
#include "enum-fdt-private.h"

enum where_e {
	IN_NONE,
	IN_CPU,
	IN_CHOSEN,
};

struct walker_node_info_s
{
	struct walker_node_info_s *parent;
	struct device_s *new;
	struct enum_pv_fdt_s *new_pv;
	uint8_t addr_cells;
	uint8_t size_cells;
	enum where_e where;
};

struct creator_state_s
{
	struct device_s *dev;
	struct walker_node_info_s *node_info;
};

static FDT_ON_NODE_ENTRY_FUNC(creator_node_entry)
{
	struct creator_state_s *priv = private;
	struct walker_node_info_s *parent = priv->node_info;

	struct walker_node_info_s *node_info =
		mem_alloc(sizeof(struct walker_node_info_s), mem_region_get_local(mem_scope_sys));

	priv->node_info = node_info;
	node_info->parent = parent;
	node_info->new = NULL;
	node_info->where = IN_NONE;

	dprintk("FDT enum considering node '%s'\n", path);

	if ( ! strcmp( path, "/chosen" ) )
		node_info->where = IN_CHOSEN;

	const char *devtype = NULL;
	size_t devtypelen;
	if ( fdt_reader_has_prop(state, "device_type", (const void**)&devtype, &devtypelen ) ) {
		dprintk("  found a new %s device\n", devtype);


		node_info->new = device_obj_new(NULL);
		node_info->new_pv = mem_alloc(sizeof(struct enum_pv_fdt_s), mem_region_get_local(mem_scope_sys));
		node_info->new_pv->offset = fdt_reader_get_struct_offset(state);
		node_info->new_pv->device_type = devtype;
		strncpy(node_info->new_pv->device_path, path, ENUM_FDT_PATH_MAXLEN);

		if ( !strcmp( devtype, "cpu" ) ) {
			const char *icudevtype = NULL;
			size_t icudevlen;

			node_info->where = IN_CPU;
			if ( fdt_reader_has_prop(state, "icudev_type",
									 (const void**)&icudevtype, &icudevlen ) ) {
				node_info->new_pv->device_type = icudevtype;
			}
		}
	}

	if ( parent ) {
		node_info->addr_cells = parent->addr_cells;
		node_info->size_cells = parent->size_cells;
	}

	dprintk("   going on\n");

	return 1;
}

static FDT_ON_NODE_LEAVE_FUNC(creator_node_leave)
{
	struct creator_state_s *priv = private;
	struct enum_fdt_context_s *pv = priv->dev->drv_pv;
	dprintk("   creator_node_leave(%p) ni: %p, npv: %p\n",
		   private, priv->node_info,
		   priv->node_info->new_pv);
	struct walker_node_info_s *node_info = priv->node_info;

	if ( node_info->new_pv ) {
		node_info->new_pv->addr_cells = node_info->addr_cells;
		node_info->new_pv->size_cells = node_info->size_cells;

		if ( node_info->new )
			node_info->new_pv->dev = node_info->new;

		fdt_node_pushback(&pv->devices, node_info->new_pv);
	}

	if ( node_info->new ) {
		dprintk("   registered a new %p device '%s', offset: %p, ac: %d, sc: %d...",
			   node_info->new_pv->device_type,
			   node_info->new_pv->device_path,
			   node_info->new_pv->offset,
			   node_info->addr_cells,
			   node_info->size_cells);

		device_obj_refnew(node_info->new);
		device_register(node_info->new, priv->dev, node_info->new_pv);

		dprintk(" ok\n");
	}

	node_info->new = NULL;
	node_info->new_pv = NULL;

	priv->node_info = node_info->parent;

	mem_free(node_info);
}

static FDT_ON_NODE_PROP_FUNC(creator_node_prop)
{
	struct creator_state_s *priv = private;
	struct enum_fdt_context_s *pv = priv->dev->drv_pv;

	if ( !strcmp( name, "#address-cells" ) )
		priv->node_info->addr_cells = endian_be32(*(uint32_t*)data);
	else if ( !strcmp( name, "#size-cells" ) )
		priv->node_info->size_cells = endian_be32(*(uint32_t*)data);
	else if ( priv->node_info->where == IN_CHOSEN && !strcmp( name, "console" ) )
		pv->console_path = data;
	else if ( priv->node_info->where == IN_CPU && !strcmp( name, "reg" ) ) {
		uint32_t val;
		fdt_parse_sized( priv->node_info->addr_cells, data,
					 sizeof(val), &val );
		priv->node_info->new_pv->cpuid = val;
	} else if ( priv->node_info->where == IN_CPU && !strcmp( name, "ipi_dev" ) )
		priv->node_info->new_pv->ipi_icudev = data;
	else if ( priv->node_info->where == IN_CPU && !strcmp( name, "ipi_no" ) ) {
		uint32_t val;
		fdt_parse_sized( priv->node_info->addr_cells, data,
					 sizeof(val), &val );
		priv->node_info->new_pv->ipi_no = val;
	}
}

static FDT_ON_MEM_RESERVE_FUNC(creator_mem_reserve)
{
}

void enum_fdt_create_children(struct device_s *dev)
{
	struct enum_fdt_context_s *pv = dev->drv_pv;

	struct creator_state_s priv = {
		.dev = dev,
		.node_info = NULL,
	};

	struct fdt_walker_s walker = {
		.private = &priv,
		.on_node_entry = creator_node_entry,
		.on_node_leave = creator_node_leave,
		.on_node_prop = creator_node_prop,
		.on_mem_reserve = creator_mem_reserve,
	};

	dprintk("walking blob\n");
	fdt_walk_blob(pv->blob, &walker);
}
