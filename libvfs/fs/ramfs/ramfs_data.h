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
  02110-1301 USA

  Copyright Nicolas Pouillon, <nipo@ssji.net>, 2009
*/

#ifndef _RAMFS_DATA_H_
#define _RAMFS_DATA_H_

#include <hexo/types.h>

#include <gct_platform.h>
#include <gct/refcount.h>

struct ramfs_data_s
{
    uint32_t magic;
    GCT_REFCOUNT_ENTRY(obj_entry);
	void *data;
	size_t allocated_size;
	size_t actual_size;
};

static inline struct ramfs_data_s * ramfs_data_create()
{
    struct ramfs_data_s *obj = mem_alloc(sizeof(*obj), mem_scope_sys);
	obj->data = NULL;
	obj->magic = 0x1ada1ada;
	obj->allocated_size = 0;
	obj->actual_size = 0;

	return 0;
}

static inline void ramfs_data_destroy(struct ramfs_data_s *obj)
{
	if ( obj->data )
		mem_free(obj->data);
    mem_free(obj);
}

GCT_REFCOUNT(ramfs_data, struct ramfs_data_s *, obj_entry);

error_t ramfs_data_realloc(struct ramfs_data_s *db, size_t new_size);

#endif

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

