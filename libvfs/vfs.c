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
#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <string.h>

#include "vfs-private.h"

error_t vfs_mount(struct vfs_node_s *mountpoint,
				  struct vfs_fs_s *fs)
{
	struct vfs_node_s *fs_root = fs->root;

	/* Cant mount on a file */
	if ( mountpoint->type != VFS_NODE_DIR )
		return -EINVAL;

	/* Cant mount a non-root */
	if ( fs->old_node != NULL )
		return -EBUSY;

	/* Cant mount at the root of the filesystem */
	if ( mountpoint->parent == mountpoint )
		return -EINVAL;

	/* Cant mount at the root of another mount */
	if ( mountpoint->fs->root == mountpoint )
		return -EINVAL;

	lock_spin(&mountpoint->dir.children.lock);

	/* Keep a reference to the mountpoint node, it may be open */
	fs->old_node = vfs_node_refnew(mountpoint);

	/* Replace entry in parent's hash, with correct name */
	memcpy(fs_root->name, mountpoint->name, CONFIG_VFS_NAMELEN);
	vfs_dir_nolock_remove(&mountpoint->parent->dir.children, mountpoint);
	vfs_dir_nolock_push(&mountpoint->parent->dir.children, fs_root);
	vfs_node_reparent(fs_root, mountpoint->parent);

	lock_release(&mountpoint->dir.children.lock);

	return 0;
}

error_t vfs_umount(struct vfs_fs_s *fs)
{
	struct vfs_node_s *fs_root = fs->root;
	struct vfs_node_s *parent = fs_root->parent;

	/* Cant umount the global root */
	if ( fs->old_node == NULL )
		return -EINVAL;

	/* This should not happen, but I want to be sure... */
	if ( parent == NULL )
		return -EINVAL;

	/* >>> Critical section */
	lock_spin(&parent->dir.children.lock);

	if ( ! fs->can_unmount(fs) )
		goto is_busy;

	/* Reput the old node where it belongs */
	vfs_dir_nolock_remove(&parent->dir.children, fs_root);
	vfs_dir_nolock_push(&parent->dir.children, fs->old_node);
	vfs_node_refdrop(fs->old_node);

	lock_release(&parent->dir.children.lock);
	/* <<< Critical section */

	fs_root->parent = NULL;
	fs->old_node = NULL;

	/* Reset the root name */
	memset(fs_root->name, 0, CONFIG_VFS_NAMELEN);

	return 0;

  is_busy:
	lock_release(&parent->dir.children.lock);
	return -EBUSY;
}

/* Node operations */

error_t vfs_node_lookup(struct vfs_node_s *parent,
						const char *name,
						size_t namelen,
						struct vfs_node_s **node)
{
	error_t err = 0;

	vfs_printk("<lookup \"%s\"/%d parent: %p [%s]... ", name, namelen, parent, parent->name);

	/* Dandling nodes are valid, but no lookup is authorized on them... */
	if ( parent->parent == NULL && parent->fs->root != parent )
		return -EINVAL;

	/* First easy ones, . and .. */
	if ( namelen == 1 && *name == '.' ) {
		*node = vfs_node_refnew(parent);
		vfs_printk("ok .>");
		return 0;
	}

	if ( namelen == 2 && name[0] == '.' && name[1] == '.' ) {
        if (parent->parent == NULL)
            return -EINVAL;
		*node = vfs_node_refnew(parent->parent);
		vfs_printk("ok ..>");
		return 0;
	}

	char tmpname[CONFIG_VFS_NAMELEN];
	memset(tmpname, 0, CONFIG_VFS_NAMELEN);
	memcpy(tmpname, name, namelen);

	semaphore_wait(&parent->dir.semaphore);

	/* Now lookup inside the hash */
	*node = vfs_dir_lookup(&parent->dir.children, tmpname);
	if ( *node ) {
		vfs_node_refnew(*node);
		vfs_printk("ok %p [%s]>", (*node), (*node)->name);
		err = 0;
		goto fini;
	}

	/* Last call: ask the FS */
	err = parent->fs->lookup(parent, name, namelen, node);
	if ( err ) {
		vfs_printk("err %d>", err);
		goto fini;
	}

	/* As FS got a node for this, we can register it in the hash */
	vfs_dir_push(&parent->dir.children, *node);
	vfs_node_reparent(*node, parent);

	vfs_printk("fs %p [%s]>", (*node), (*node)->name);

  fini:
	semaphore_post(&parent->dir.semaphore);
	return err;
}

error_t vfs_node_create(struct vfs_fs_s *fs,
						enum vfs_node_type_e type,
						struct vfs_node_s **node)
{
    if ( fs->create == NULL )
        return -ENOTSUP;

    if ( fs->flag_ro )
        return -EPERM;

	return fs->create(fs, type, node);
}

error_t vfs_node_open(struct vfs_fs_s *fs,
                      struct vfs_node_s *node,
                      enum vfs_open_flags_e flags,
                      struct vfs_file_s **file)
{
	vfs_printk(" node_open[%p:%p](%p): ", fs, node->fs->node_open, node);

    assert( fs->node_open != NULL );

    if ( flags & (VFS_OPEN_WRITE | VFS_OPEN_CREATE | VFS_OPEN_APPEND) && fs->flag_ro )
        return -EPERM;

    return fs->node_open(node, flags, file);
}

error_t vfs_node_link(struct vfs_node_s *parent,
					  struct vfs_node_s *node,
					  const char *name,
					  size_t namelen,
					  struct vfs_node_s **rnode)
{
	error_t err = 0;

	vfs_printk("<%s '%s' %p [%s] in %p [%s]... ", __FUNCTION__, name, node, node->name, parent, parent->name);

	if ( parent->type != VFS_NODE_DIR )
		return -EINVAL;

    if ( parent->fs->link == NULL )
        return -ENOTSUP;

    if ( parent->fs->flag_ro )
        return -EPERM;

	char tmpname[CONFIG_VFS_NAMELEN];
	memset(tmpname, 0, CONFIG_VFS_NAMELEN);
	memcpy(tmpname, name, namelen);

	semaphore_wait(&parent->dir.semaphore);

	struct vfs_node_s *prev_node = vfs_dir_lookup(&parent->dir.children, tmpname);

	if ( prev_node ) {
		if ( prev_node->type == VFS_NODE_DIR ) {
			err = -EBUSY;
			goto fini;
		} else {
			vfs_node_reparent(prev_node, NULL);
			vfs_dir_remove(&parent->dir.children, prev_node);
		}
	}

	err = parent->fs->link(parent, node, name, namelen, rnode);
	if ( err ) {
		vfs_printk("fail %d>\n", err);
		goto fini;
	}

	vfs_node_reparent(*rnode, parent);
	vfs_dir_push(&parent->dir.children, *rnode);

	vfs_printk("ok>\n");

	err = 0;
  fini:
	semaphore_post(&parent->dir.semaphore);
	return err;
}

/**
   Unlinks a node from its parent.
 */
error_t vfs_node_unlink(struct vfs_node_s *parent,
						const char *name,
						size_t namelen)
{
	if ( parent->type != VFS_NODE_DIR )
		return -EINVAL;

    if ( parent->fs->unlink == NULL )
        return -ENOTSUP;

    if ( parent->fs->flag_ro )
        return -EPERM;

	char tmpname[CONFIG_VFS_NAMELEN];
	memset(tmpname, 0, CONFIG_VFS_NAMELEN);
	memcpy(tmpname, name, namelen);

	semaphore_wait(&parent->dir.semaphore);

	struct vfs_node_s *node = vfs_dir_lookup(&parent->dir.children, tmpname);

	error_t err = parent->fs->unlink(parent, name, namelen);
	if ( err )
		goto fini;

	if ( node ) {
		vfs_dir_remove(&parent->dir.children, node);
		vfs_node_reparent(node, NULL);
	}

	err = 0;
  fini:
	semaphore_post(&parent->dir.semaphore);
	return err;
}

error_t vfs_node_stat(struct vfs_node_s *node,
					  struct vfs_stat_s *stat)
{
	return node->fs->stat(node, stat);
}

OBJECT_CONSTRUCTOR(vfs_node)
{
	struct vfs_fs_s *fs = va_arg(ap, struct vfs_fs_s *);
	obj->type = va_arg(ap, enum vfs_node_type_e);
	const char *name = va_arg(ap, const char *);
	size_t size = va_arg(ap, size_t);
	obj->priv = va_arg(ap, void*);
	obj->priv_deleter = va_arg(ap, vfs_node_fs_priv_deleter_t*);

    /* FIXME should gracefully handle shortened names colision */
    if (size > CONFIG_VFS_NAMELEN)
        size = CONFIG_VFS_NAMELEN;
    memcpy(obj->name, name, size);
	memset(obj->name + size, 0, CONFIG_VFS_NAMELEN - size);

	obj->fs = fs;

	atomic_inc(&fs->ref);

	obj->parent = NULL;

	vfs_printk("<node create %p %p '%s'", fs, obj, obj->name);

	if ( obj->type == VFS_NODE_DIR ) {
		vfs_dir_init(&obj->dir.children);
		semaphore_init(&obj->dir.semaphore, 1);
	}

	vfs_printk(">");

	return 0;
}

OBJECT_DESTRUCTOR(vfs_node)
{
	vfs_printk("<node] delete %p '%s'", obj, obj->name);

	atomic_dec(&obj->fs->ref);

	if ( obj->parent )
		vfs_node_refdrop(obj->parent);
	if (obj->type == VFS_NODE_DIR) {
		vfs_dir_destroy(&obj->dir.children);
		semaphore_destroy(&obj->dir.semaphore);
	}

	if ( obj->priv && obj->priv_deleter )
		obj->priv_deleter(obj);

	vfs_printk(">");
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

