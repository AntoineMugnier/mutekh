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

#include <vfs/vfs.h>
#include "vfs-private.h"
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>


struct vfs_fs_s * vfs_fs_create()
{
    struct vfs_fs_s *obj = mem_alloc(sizeof(*obj), mem_scope_sys);

    if (!obj)
        return NULL;

    atomic_set(&obj->ref, 0);

#if defined(CONFIG_VFS_STATS)
    atomic_set(&obj->node_open_count, 0);
    atomic_set(&obj->lookup_count, 0);
    atomic_set(&obj->create_count, 0);
    atomic_set(&obj->link_count, 0);
    atomic_set(&obj->move_count, 0);
    atomic_set(&obj->unlink_count, 0);
    atomic_set(&obj->stat_count, 0);
    atomic_set(&obj->node_create_count, 0);
    atomic_set(&obj->node_destroy_count, 0);
    atomic_set(&obj->file_open_count, 0);
    atomic_set(&obj->file_close_count, 0);
#endif

	return obj;
}

void vfs_fs_destroy(struct vfs_fs_s *obj)
{
    mem_free(obj);
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

