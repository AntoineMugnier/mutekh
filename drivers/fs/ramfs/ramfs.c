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

#include <hexo/types.h>
#include <hexo/error.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <vfs/types.h>
#include <vfs/fs.h>

#include "ramfs.h"
#include "ramfs-private.h"

error_t ramfs_close(struct vfs_fs_s *fs)
{
	return -EBUSY;
}

VFS_FS_CAN_UNMOUNT(ramfs_can_unmount)
{
	return 0;
}

error_t ramfs_open(struct vfs_fs_s **fs)
{
	struct vfs_fs_s *mnt = mem_alloc(sizeof(struct vfs_fs_s), mem_scope_sys);
	if ( mnt == NULL )
		goto nomem_fs;

    vfs_printk("ramfs: opening new ramfs volume\n");

    memset(mnt, 0, sizeof(*mnt));
	atomic_set(&mnt->ref, 0);
	mnt->node_open = ramfs_node_open;
	mnt->lookup = ramfs_lookup;
	mnt->create = ramfs_create;
	mnt->link = ramfs_link;
	mnt->unlink = ramfs_unlink;
	mnt->stat = ramfs_stat;
	mnt->can_unmount = ramfs_can_unmount;

	mnt->old_node = NULL;

	struct vfs_node_s *node = vfs_node_new(NULL, mnt, VFS_NODE_DIR, "", 0, NULL, NULL);
	if ( node == NULL )
		goto nomem_dir;

	mnt->root = node;

	// TODO register destructor

	*fs = mnt;

	return 0;
  nomem_dir:
	mem_free(mnt);
  nomem_fs:
	return -ENOMEM;
}

VFS_FS_LOOKUP(ramfs_lookup)
{
	return -ENOENT;
}

static VFS_NODE_FS_PRIV_DELETER(ramfs_file_node_deleted)
{
	ramfs_data_refdrop(node->priv);
}

VFS_FS_CREATE(ramfs_create)
{
	vfs_printk("<%s ", __FUNCTION__);

	struct ramfs_data_s *priv = NULL;
	vfs_node_fs_priv_deleter_t *priv_deleter = NULL;
	if ( type == VFS_NODE_FILE ) {
		priv = ramfs_data_new(NULL);
		if ( !priv )
			goto err_priv;
		priv_deleter = ramfs_file_node_deleted;
	}
	struct vfs_node_s *rnode = vfs_node_new(NULL, fs, type, "", 0,
											priv, priv_deleter);
	if ( !rnode )
		goto err_rnode;

	*node = rnode;

	vfs_printk("ok>\n");

	return 0;
  err_rnode:
	if ( priv )
		ramfs_data_refdrop(priv);
	vfs_printk("err>\n");
  err_priv:
	return -ENOMEM;
}

VFS_FS_LINK(ramfs_link)
{
	if ( namelen >= CONFIG_VFS_NAMELEN )
		return -EINVAL;

	/* We cant support hardlinks of directories */
	if ( node->parent != NULL && node->type == VFS_NODE_DIR )
		return -EINVAL;

	vfs_printk("<%s %s ", __FUNCTION__, name);

	struct vfs_node_s *nnode;
	if ( node->parent != NULL ) {
		vfs_printk("clone (parent=%p) ", node->parent);
		nnode = vfs_node_new(NULL, parent->fs, VFS_NODE_FILE, name, namelen,
							 ramfs_data_refnew(node->priv),
							 ramfs_file_node_deleted);
		if (nnode == NULL) {
			vfs_printk("failed>");
			return -ENOMEM;
		}
	} else {
		vfs_printk("use ");
		nnode = vfs_node_refnew(node);
		memcpy(node->name, name, namelen);
	}

	*rnode = nnode;

	/*
	  We dont really do anything useful as the generic part of the VFS
	  will register this node in the parent's hash.
	 */

	vfs_printk("OK>");

	return 0;
}

VFS_FS_UNLINK(ramfs_unlink)
{
	char tmpname[CONFIG_VFS_NAMELEN];
	memset(tmpname, 0, CONFIG_VFS_NAMELEN);
	memcpy(tmpname, name, namelen);

	struct vfs_node_s *node = vfs_dir_lookup(&parent->dir.children, name);

	if ( node == NULL )
		return -ENOENT;

	// There is 1 for the filesystem itself, and 1 for the caller of
	// this very function. Other references are usage.
	// Files are always unlinkable, data is refcounted anyway.
	if ( node->type == VFS_NODE_DIR && vfs_node_refcount(node) > 2 )
		return -EBUSY;

	if ( node->parent == NULL )
		return -ENOENT;

	vfs_node_refdrop(node);
	return 0;
}

VFS_FS_STAT(ramfs_stat)
{
	stat->type = node->type;
	stat->nlink = 1;

	switch ( node->type ) {
	case VFS_NODE_DIR:
		stat->size = vfs_dir_count(&node->dir.children);
		break;
	case VFS_NODE_FILE: {
		struct ramfs_data_s *data = LEAF(node);
		stat->size = data->actual_size;
		stat->nlink = ramfs_data_refcount(data);
		break;
	}
	}

	return 0;
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

