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

#define GPCT_CONFIG_NODEPRECATED

#include <vfs/vfs.h>
#include "vfs-private.h"
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <string.h>

ssize_t vfs_node_get_name(struct vfs_node_s *node,
                          char *name,
                          size_t namelen)
{
    strncpy(name, node->name, CONFIG_VFS_NAMELEN);
    return (strlen(name) < CONFIG_VFS_NAMELEN) ? 0 : -ENOMEM;
}

struct vfs_fs_s *vfs_node_get_fs(struct vfs_node_s *node)
{
    return node->fs;
}

struct vfs_node_s *vfs_node_create(
    struct vfs_fs_s *fs,
    const char *mangled_name,
    struct fs_node_s *fs_node)
{
    struct vfs_node_s *obj = mem_alloc(sizeof(*obj), mem_scope_sys);

    if (!obj)
        return NULL;

    vfs_node_refinit(obj);

    if ( mangled_name )
        memcpy(obj->name, mangled_name, CONFIG_VFS_NAMELEN);
    else
        memset(obj->name, 0, CONFIG_VFS_NAMELEN);
	obj->fs = fs;

    obj->fs_node = obj->fs->ops->node_refnew(fs_node);

	obj->parent = NULL;
    lock_init(&obj->parent_lock);

	atomic_inc(&fs->ref);
	VFS_STATS_INC(fs, node_create_count);

    semaphore_init(&obj->dir_semaphore, 1);

#if defined(CONFIG_VFS_STATS)
    atomic_set(&obj->lookup_count, 0);
    atomic_set(&obj->open_count, 0);
    atomic_set(&obj->close_count, 0);
    atomic_set(&obj->stat_count, 0);
#endif

    vfs_dir_init(&obj->children);

	vfs_printk("<node create %p %p '%s' free: %p>",
               fs, obj, obj->name,
               obj->obj_entry.storage_free);

	return obj;
}

void vfs_node_destroy(struct vfs_node_s *obj)
{
	vfs_printk("<node delete %p '%s' free: %p", obj, obj->name,
               obj->obj_entry.storage_free);

    struct vfs_node_s *parent = vfs_node_get_parent(obj);
    if ( parent ) {
        vfs_node_dirlock(parent);
        vfs_node_parent_nolock_unset(obj);
        vfs_node_dirunlock(parent);
        vfs_node_refdec(parent);
    }
    
    vfs_dir_destroy(&obj->children);

    semaphore_destroy(&obj->dir_semaphore);

    atomic_dec(&obj->fs->ref);

    VFS_STATS_INC(obj->fs, node_destroy_count);

    obj->fs->ops->node_refdrop(obj->fs_node);
    lock_destroy(&obj->parent_lock);

    vfs_node_refcleanup(obj);
    mem_free(obj);

    vfs_printk(" done>");
}

struct vfs_node_s *vfs_node_get_parent(struct vfs_node_s *node)
{
    struct vfs_node_s *parent = NULL;

    CPU_INTERRUPT_SAVESTATE_DISABLE;
    lock_spin(&node->parent_lock);
	if ( !vfs_node_is_dandling(node) )
        parent = vfs_node_refinc(node->parent);
    lock_release(&node->parent_lock);
    CPU_INTERRUPT_RESTORESTATE;

    return parent;
}

void
vfs_node_parent_nolock_unset(struct vfs_node_s *node)
{
    CPU_INTERRUPT_SAVESTATE_DISABLE;
    lock_spin(&node->parent_lock);
	if ( !vfs_node_is_dandling(node) ) {
        struct vfs_node_s *parent = node->parent;
        vfs_dir_remove(&parent->children, node);
        node->parent = NULL;
        vfs_node_refdec(parent);
    }
    lock_release(&node->parent_lock);
    CPU_INTERRUPT_RESTORESTATE;
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

