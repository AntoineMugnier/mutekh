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

#define GPCT_CONFIG_NODEPRECATED

#include <hexo/types.h>
#include <hexo/error.h>

#include <mutek/mem_alloc.h>
#include <mutek/printk.h>

#include <vfs/node.h>
#include <vfs/fs.h>
#include <vfs/file.h>
#include <vfs/name.h>

#include "ramfs.h"
#include "ramfs-private.h"
#include "ramfs_data.h"
#include "ramfs_file.h"

GCT_CONTAINER_KEY_FCNS       (ramfs_dir_hash, ASC, static inline, ramfs_dir, name,
                              init, destroy, clear, count, wrlock, unlock, lookup, rdlock);

GCT_CONTAINER_KEY_NOLOCK_FCNS(ramfs_dir_hash, ASC, static inline, ramfs_dir_nolock, name,
                              push, head, remove, next, lookup, count);

/*
  Here are two helpers when we need to take two locks at the same time
  in two directories. We MUST always take them in the same order to
  avoid deadlocks. So we take them in pointer order, and release them
  in the opposite order.
 */
void ramfs_2dir_wrlock(ramfs_dir_hash_root_t *d1,
                       ramfs_dir_hash_root_t *d2)
{
    if ( d1 < d2 ) {
        ramfs_dir_wrlock(d1);
        ramfs_dir_wrlock(d2);
    } else {
        ramfs_dir_wrlock(d2);
        ramfs_dir_wrlock(d1);
    }
}

void ramfs_2dir_unlock(ramfs_dir_hash_root_t *d1,
                       ramfs_dir_hash_root_t *d2)
{
    if ( d1 < d2 ) {
        ramfs_dir_unlock(d2);
        ramfs_dir_unlock(d1);
    } else {
        ramfs_dir_unlock(d1);
        ramfs_dir_unlock(d2);
    }
}

struct ramfs_node_s *ramfs_node_create(
    enum vfs_node_type_e type, 
    struct vfs_fs_s *fs,
    struct ramfs_data_s *data,
    const char *name, size_t namelen)
{
    struct ramfs_node_s *node = mem_alloc(sizeof(*node), mem_scope_sys);

    if (!node)
        return NULL;

    vfs_printk("<ramfs_node_ctor");

    vfs_node_init(&node->node, fs, type, name, namelen);

    switch (type) {
    case VFS_NODE_FILE:
        if ( data == NULL ) {
            node->data = ramfs_data_create();
            if ( node->data == NULL ) {
                return NULL;
            }
        } else {
            node->data = ramfs_data_refinc(data);
        }
        break;
    case VFS_NODE_DIR:
        ramfs_dir_init(&node->children);
    }

    vfs_printk(">");

    return node;
}

VFS_FS_NODE_CLEANUP(ramfs_node_cleanup)
{
    struct ramfs_node_s *rnode = (void*)node;

    vfs_printk("<ramfs_node_dtor %p", node);

    switch (node->type) {
    case VFS_NODE_DIR:
        ramfs_dir_clear(&rnode->children);
        ramfs_dir_destroy(&rnode->children);
        break;
    case VFS_NODE_FILE:
        ramfs_data_refdec(rnode->data);
        break;
    }

    vfs_printk(">");
}

void ramfs_dump_item(struct ramfs_node_s *node, size_t pf)
{
    size_t i;
    for ( i=0; i<pf; ++i )
        printk(" ");
    printk(" %s %d '%s': %d\n",
           node->node.type == VFS_NODE_DIR ? ">" : "-",
           vfs_node_refcount(&node->node),
           node->name,
           node->node.type == VFS_NODE_FILE
           ? ramfs_data_refcount(node->data)
           : 0);
    if ( node->node.type == VFS_NODE_DIR )
		GCT_FOREACH(ramfs_dir_hash, &node->children, item, {
                ramfs_dump_item(item, pf+2);
            });
}

void ramfs_dump(struct vfs_fs_s *fs)
{
    printk("Ramfs dump. ref: %d\n", vfs_fs_refcount(fs));
    vfs_fs_dump_stats(fs);
    struct ramfs_node_s *root = (struct ramfs_node_s *)fs->root;
    ramfs_dump_item(root, 0);
}



VFS_FS_CAN_UNMOUNT(ramfs_can_unmount)
{
	return vfs_fs_refcount(fs) == 1;
}

VFS_FS_LOOKUP(ramfs_lookup)
{
    struct ramfs_node_s *rref = (struct ramfs_node_s *)ref;
    vfs_printk("<%s %s/%d ", __FUNCTION__, name, namelen);

    if ( rref->node.type != VFS_NODE_DIR )
        return -EINVAL;

    vfs_name_mangle(name, namelen, mangled_name);

    struct ramfs_node_s *child = ramfs_dir_lookup(&rref->children, mangled_name);
    if ( child ) {
        *node = &child->node;
        vfs_printk(" ok>");
        return 0;
    }
    vfs_printk(" noent>");
    return -ENOENT;
}

VFS_FS_CREATE(ramfs_create)
{
	vfs_printk("<%s ", __FUNCTION__);

    struct ramfs_node_s *rfs_node;
    rfs_node = ramfs_node_create(type, fs, NULL, NULL, 0);
	if ( !rfs_node )
		goto err_priv;

	*node = &rfs_node->node;

	vfs_printk("ok>\n");

	return 0;

  err_priv:
	vfs_printk("err>\n");
	return -ENOMEM;
}

VFS_FS_LINK(ramfs_link)
{
    struct ramfs_node_s *rparent = (struct ramfs_node_s *)parent;
    struct ramfs_node_s *rfs_node = (struct ramfs_node_s *)node;

	if ( namelen >= CONFIG_VFS_NAMELEN )
		return -EINVAL;
    
    if ( rparent->node.type != VFS_NODE_DIR )
        return -EINVAL;
    
	vfs_printk("<%s %s ", __FUNCTION__, name);

    if ( rparent->parent == NULL ) {
        vfs_printk("dandling>");
        return -ENOTSUP;
    }

	/* We cant support hardlinks of directories */
	if ( (rfs_node->parent != NULL)
         && (rfs_node->node.type == VFS_NODE_DIR) ) {
        vfs_printk("isdir>");
        return -EISDIR;
    }

    struct ramfs_node_s *ret_node;
	if (rfs_node->parent != NULL) {
		vfs_printk("clone ");
        ret_node = ramfs_node_create(
            VFS_NODE_FILE, rfs_node->node.fs, rfs_node->data,
            name, namelen);
        if ( ret_node == NULL ) {
			vfs_printk("node_new fail>");
			return -ENOMEM;
        }
	} else {
		vfs_printk("use ");
		ret_node = (struct ramfs_node_s *)vfs_node_refinc(node);
	}

    vfs_printk("file linked as %s ", rfs_node->node.name);

    ramfs_dir_wrlock(&rparent->children);
    struct ramfs_node_s *old_file = ramfs_dir_nolock_lookup(&rparent->children, ret_node->name);
    if ( old_file ) {
        old_file->parent = NULL;
        ramfs_dir_nolock_remove(&rparent->children, old_file);
        vfs_printk("replaces another ");
    }
    ret_node->parent = rparent;
    ramfs_dir_nolock_push(&rparent->children, ret_node);
    ramfs_dir_unlock(&rparent->children);

    if ( old_file )
        vfs_node_refdec(&old_file->node);

	vfs_printk("ok>");

    *rnode = &ret_node->node;

	return 0;
}

VFS_FS_MOVE(ramfs_move)
{
    struct ramfs_node_s *rparent = (struct ramfs_node_s *)parent;
    struct ramfs_node_s *rnode = (struct ramfs_node_s *)node;

	if ( namelen >= CONFIG_VFS_NAMELEN )
		return -EINVAL;
    
    if ( rparent->node.type != VFS_NODE_DIR )
        return -EINVAL;

    if ( rnode->parent == NULL
         || rnode->parent == rnode )
        return -EINVAL;
    
	vfs_printk("<%s %s ", __FUNCTION__, name);

    if ( rparent->parent == NULL ) {
        vfs_printk("dandling>");
        return -ENOTSUP;
    }

    struct ramfs_node_s *parent_src = rnode->parent;
    ramfs_2dir_wrlock(&parent_src->children, &rparent->children);
    ramfs_dir_nolock_remove(&parent_src->children, rnode);
	memset(rnode->name, 0, CONFIG_VFS_NAMELEN);
	memcpy(rnode->name, name, namelen);

    struct ramfs_node_s *old_file = ramfs_dir_nolock_lookup(&rparent->children, rnode->name);
    if ( old_file ) {
        old_file->parent = NULL;
        ramfs_dir_nolock_remove(&rparent->children, old_file);
    }

    rnode->parent = rparent;
    ramfs_dir_nolock_push(&rparent->children, rnode);
    ramfs_2dir_unlock(&rparent->children, &parent_src->children);

    if ( old_file )
        vfs_node_refdec(&old_file->node);

	vfs_printk("ok>");

	return 0;
}

VFS_FS_UNLINK(ramfs_unlink)
{
    struct ramfs_node_s *rparent = (struct ramfs_node_s *)parent;

	char tmpname[CONFIG_VFS_NAMELEN];
	memset(tmpname, 0, CONFIG_VFS_NAMELEN);
	memcpy(tmpname, name, namelen);

	vfs_printk("<%s %s ", __FUNCTION__, tmpname);

    ramfs_dir_wrlock(&rparent->children);

    struct ramfs_node_s *node = ramfs_dir_nolock_lookup(&rparent->children, tmpname);
	if ( node == NULL ) {
        ramfs_dir_unlock(&rparent->children);
        vfs_printk("not found>");
		return -ENOENT;
    }

    if ( (node->node.type == VFS_NODE_DIR)
         && (ramfs_dir_nolock_count(&node->children) != 0) ) {
        vfs_node_refdec(&node->node);
        ramfs_dir_unlock(&rparent->children);
        vfs_printk("nonempty>");
		return -EBUSY;
    }

    node->parent = NULL;
    ramfs_dir_nolock_remove(&rparent->children, node);
    ramfs_dir_unlock(&rparent->children);

    vfs_node_refdec(&node->node);

	vfs_printk("ok>");

	return 0;
}

VFS_FS_STAT(ramfs_stat)
{
    struct ramfs_node_s *rnode = (struct ramfs_node_s *)node;

	stat->type = rnode->node.type;

	switch ( rnode->node.type ) {
	case VFS_NODE_DIR:
		stat->size = ramfs_dir_count(&rnode->children);
        stat->nlink = 1;
		break;
	case VFS_NODE_FILE:
		stat->size = rnode->data->actual_size;
		stat->nlink = ramfs_data_refcount(rnode->data);
		break;
	}

	return 0;
}

VFS_FS_CLEANUP(ramfs_cleanup)
{
    struct ramfs_fs_s *rfs = (struct ramfs_fs_s *)fs;
    vfs_fs_cleanup(&rfs->fs);
	mem_free(rfs);
}

static const struct vfs_fs_ops_s ramfs_ops =
{
    .node_open = ramfs_node_open,
    .lookup = ramfs_lookup,
    .create = ramfs_create,
    .link = ramfs_link,
    .move = ramfs_move,
    .unlink = ramfs_unlink,
    .stat = ramfs_stat,
    .can_unmount = ramfs_can_unmount,
    .cleanup = ramfs_cleanup,
    .node_cleanup = ramfs_node_cleanup,
};

error_t ramfs_open(struct vfs_fs_s **_fs)
{
    error_t err = -ENOMEM;
	struct ramfs_fs_s *fs = mem_alloc(sizeof(*fs), mem_scope_sys);
	if (!fs)
		goto nomem_fs;

    vfs_printk("ramfs: opening new ramfs volume\n");

    err = vfs_fs_init(&fs->fs, &ramfs_ops, 0);
    if (err)
        goto release_fs;

    struct ramfs_node_s *root = ramfs_node_create(VFS_NODE_DIR, &fs->fs, NULL, NULL, 0);
	if ( root == NULL )
		goto nomem_dir;

    root->parent = root;

    vfs_fs_root_set(&fs->fs, &root->node);

	*_fs = &fs->fs;

	return 0;

  nomem_dir:
    vfs_fs_cleanup(&fs->fs);
 release_fs:
	mem_free(fs);
  nomem_fs:
	return err;
}


bool_t ramfs_dir_get_nth(struct ramfs_node_s *node, struct vfs_dirent_s *dirent, size_t n)
{
	size_t i;

	ramfs_dir_rdlock(&node->children);

    struct ramfs_node_s *ent;

	i = 0;
    ent = ramfs_dir_nolock_head(&node->children);
    while ( (i < n) && (ent != NULL) ) {
        i++;
        struct ramfs_node_s *nent;
        nent = ramfs_dir_nolock_next(&node->children, ent);
        ramfs_node_refdec(ent);
        ent = nent;
    }

    if ( ent == NULL ) {
        ramfs_dir_unlock(&node->children);
        return 0;
    }

	memcpy( dirent->name, ent->name, CONFIG_VFS_NAMELEN );
	dirent->type = ent->node.type;
	dirent->size = ent->node.type == VFS_NODE_FILE
		? ent->data->actual_size
		: ramfs_dir_count(&ent->children);

	ramfs_dir_unlock(&node->children);

    ramfs_node_refdec(ent);

    return 1;
}

