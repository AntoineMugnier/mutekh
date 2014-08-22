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

  Copyright Nicolas Pouillon, <nipo@ssji.net>, 2009,2014
*/

#include <vfs/fs.h>

#include "vfs-private.h"

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

error_t vfs_fs_init(struct vfs_fs_s *fs, const struct vfs_fs_ops_s *ops, bool_t ro)
{
    vfs_printk("<fs %p init>", fs);

    fs->ops = ops;
	fs->old_node = NULL;
    fs->root = NULL;
    fs->flag_ro = ro;

    vfs_fs_refinit(fs);

#if defined(CONFIG_VFS_STATS)
    atomic_set(&fs->node_open_count, 0);
    atomic_set(&fs->lookup_count, 0);
    atomic_set(&fs->create_count, 0);
    atomic_set(&fs->link_count, 0);
    atomic_set(&fs->move_count, 0);
    atomic_set(&fs->unlink_count, 0);
    atomic_set(&fs->stat_count, 0);
    atomic_set(&fs->node_create_count, 0);
    atomic_set(&fs->node_destroy_count, 0);
    atomic_set(&fs->file_open_count, 0);
    atomic_set(&fs->file_close_count, 0);
#endif

    return 0;
}

void vfs_fs_root_set(struct vfs_fs_s *fs, struct vfs_node_s *root)
{
    vfs_printk("<fs %p root %p>", fs, root);

    fs->root = root;
    root->parent = vfs_node_refinc(root);
}

void vfs_fs_cleanup(struct vfs_fs_s *fs)
{
    vfs_fs_refcleanup(fs);

    assert(fs->root == NULL);

    vfs_printk("<fs %p cleanup>", fs);
}

void vfs_fs_destroy(struct vfs_fs_s *fs)
{
    if (fs->ops->cleanup)
        fs->ops->cleanup(fs);

    vfs_fs_cleanup(fs);

    mem_free(fs);
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

