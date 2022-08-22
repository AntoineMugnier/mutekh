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
    if (d1 < d2) {
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
    if (d1 < d2) {
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

    memset(node->name, 0, sizeof(node->name));
    memcpy(node->name, name, namelen);

    switch (type) {
    case VFS_NODE_FILE:
        if (data) {
            node->data = ramfs_data_refinc(data);
        } else {
            node->data = ramfs_data_create();

            if (!node->data) {
                free(node);
                return NULL;
            }
        }
        break;

    case VFS_NODE_DIR:
        ramfs_dir_init(&node->children);
        break;
    }

    vfs_node_init(&node->node, fs, type, name, namelen);

    vfs_printk(">");

    return node;
}

VFS_FS_NODE_CLEANUP(ramfs_node_cleanup)
{
    struct ramfs_node_s *to_cleanup = ramfs_node_from_vfs(node);

    vfs_printk("<ramfs_node_dtor %p", node);

    switch (node->type) {
    case VFS_NODE_DIR:
        ramfs_dir_clear(&to_cleanup->children);
        ramfs_dir_destroy(&to_cleanup->children);
        break;

    case VFS_NODE_FILE:
        ramfs_data_refdec(to_cleanup->data);
        break;
    }

    vfs_printk(">");
}

void ramfs_dump_item(struct ramfs_node_s *node, size_t pf)
{
    size_t i;

    for (i = 0; i < pf; ++i)
        printk(" ");

    printk(" %s %d '%s': %d\n",
           node->node.type == VFS_NODE_DIR ? ">" : "-",
           vfs_node_refcount(&node->node),
           node->name,
           node->node.type == VFS_NODE_FILE
           ? ramfs_data_refcount(node->data)
           : 0);

    if (node->node.type == VFS_NODE_DIR)
		GCT_FOREACH(ramfs_dir_hash, &node->children, item, {
                ramfs_dump_item(item, pf+2);
            });
}

void ramfs_dump(struct vfs_fs_s *fs)
{
    printk("Ramfs dump. ref: %d\n", vfs_fs_refcount(fs));

    vfs_fs_dump_stats(fs);

    struct ramfs_node_s *root = ramfs_node_from_vfs(fs->root);

    ramfs_dump_item(root, 0);
}

VFS_FS_CAN_UNMOUNT(ramfs_can_unmount)
{
	return vfs_fs_refcount(fs) == 1;
}

VFS_FS_LOOKUP(ramfs_lookup)
{
    struct ramfs_node_s *directory = ramfs_node_from_vfs(ref);
    struct ramfs_node_s *found;

    vfs_printk("<%s %s/%d in %p ", __FUNCTION__, name, namelen, directory);

    if (directory->node.type != VFS_NODE_DIR)
        return -EINVAL;

    vfs_name_mangle(name, namelen, mangled_name);

    found = ramfs_dir_lookup(&directory->children, mangled_name);
    if (!found) {
        vfs_printk(" noent>");
        return -ENOENT;
    }

    *node = &found->node;
    vfs_printk(" ok %p %d>", found, vfs_node_refcount(*node));
    return 0;
}

VFS_FS_CREATE(ramfs_create)
{
    struct ramfs_node_s *created;

	vfs_printk("<%s ", __FUNCTION__);

    created = ramfs_node_create(type, fs, NULL, NULL, 0);
	if (!created) {
        vfs_printk("err>\n");
        return -ENOMEM;
    }

	*node = &created->node;

	vfs_printk("ok>\n");

	return 0;
}

VFS_FS_LINK(ramfs_link)
{
    struct ramfs_node_s *target_dir = ramfs_node_from_vfs(parent);
    struct ramfs_node_s *to_link = ramfs_node_from_vfs(node);
    struct ramfs_node_s *linked;
    struct ramfs_node_s *prev_entry;

	if (namelen >= CONFIG_VFS_NAMELEN)
		return -EINVAL;
    
    if (target_dir->node.type != VFS_NODE_DIR)
        return -EINVAL;

	vfs_printk("<%s %s ", __FUNCTION__, name);

    if (target_dir->parent == NULL) {
        vfs_printk("dangling>");
        return -ENOTSUP;
    }

	/* We cant support hardlinks of directories */
	if (to_link->parent && to_link->node.type == VFS_NODE_DIR) {
        vfs_printk("isdir>");
        return -EISDIR;
    }

	if (to_link->parent) {
		vfs_printk("clone ");
        linked = ramfs_node_create(
            VFS_NODE_FILE, to_link->node.fs, to_link->data,
            name, namelen);
        if (!linked) {
			vfs_printk("node_new fail>");
			return -ENOMEM;
        }
	} else {
		vfs_printk("use ");
		linked = to_link;
	}

	memset(linked->name, 0, CONFIG_VFS_NAMELEN);
	memcpy(linked->name, name, namelen);

    ramfs_dir_wrlock(&target_dir->children);

    prev_entry = ramfs_dir_nolock_lookup(&target_dir->children, linked->name);

    if (prev_entry) {
        if (prev_entry->node.type == VFS_NODE_DIR) {
            vfs_printk("target is dir>");
            ramfs_node_refdec(prev_entry);
            ramfs_dir_unlock(&target_dir->children);
            return -EISDIR;
        }

        prev_entry->parent = NULL;
        ramfs_dir_nolock_remove(&target_dir->children, prev_entry);
        vfs_printk("replaces another ");
    }

    linked->parent = target_dir;
    ramfs_dir_nolock_push(&target_dir->children, linked);
    ramfs_dir_unlock(&target_dir->children);

    vfs_printk("file linked as %s ", linked->name);

    /* Removing from children dropped ref, but we still have local one */
    if (prev_entry)
        ramfs_node_refdec(prev_entry);

	vfs_printk("ok>");

    *rnode = &linked->node;

	return 0;
}

VFS_FS_MOVE(ramfs_move)
{
    struct ramfs_node_s *target_dir = ramfs_node_from_vfs(parent);
    struct ramfs_node_s *to_move = ramfs_node_from_vfs(node);
    struct ramfs_node_s *source_parent;
    struct ramfs_node_s *prev_entry;
	char tmpname[CONFIG_VFS_NAMELEN];

	if (namelen >= CONFIG_VFS_NAMELEN)
		return -EINVAL;

	memset(tmpname, 0, CONFIG_VFS_NAMELEN);
	memcpy(tmpname, name, namelen);
    
    if (target_dir->node.type != VFS_NODE_DIR)
        return -EINVAL;

    if (!to_move->parent || to_move->parent == to_move)
        return -EINVAL;

	vfs_printk("<%s %s ", __FUNCTION__, name);

    if (!target_dir->parent) {
        vfs_printk("dangling>");
        return -ENOTSUP;
    }

    source_parent = to_move->parent;
    ramfs_2dir_wrlock(&source_parent->children, &target_dir->children);

    /* Did someone manage to move the file just before locking ? */
    if (source_parent != to_move->parent) {
        ramfs_2dir_unlock(&target_dir->children, &source_parent->children);
        vfs_printk("stolen>");
        return -EIO;
    }

    prev_entry = ramfs_dir_nolock_lookup(&target_dir->children, tmpname);
    if (prev_entry) {
        if (prev_entry->node.type == VFS_NODE_DIR) {
            vfs_printk("target is dir>");
            ramfs_node_refdec(prev_entry);
            ramfs_2dir_unlock(&target_dir->children, &source_parent->children);
            return -EISDIR;
        }

        prev_entry->parent = NULL;
        ramfs_dir_nolock_remove(&target_dir->children, prev_entry);
    }

    ramfs_dir_nolock_remove(&source_parent->children, to_move);
	memcpy(to_move->name, tmpname, CONFIG_VFS_NAMELEN);

    to_move->parent = target_dir;
    ramfs_dir_nolock_push(&target_dir->children, to_move);

    ramfs_2dir_unlock(&target_dir->children, &source_parent->children);

    if (prev_entry)
        ramfs_node_refdec(prev_entry);

	vfs_printk("ok>");

	return 0;
}

VFS_FS_UNLINK(ramfs_unlink)
{
    struct ramfs_node_s *directory = ramfs_node_from_vfs(parent);
    struct ramfs_node_s *to_remove;
	char tmpname[CONFIG_VFS_NAMELEN];

	memset(tmpname, 0, CONFIG_VFS_NAMELEN);
	memcpy(tmpname, name, namelen);

	vfs_printk("<%s %s ", __FUNCTION__, tmpname);

    ramfs_dir_wrlock(&directory->children);

    to_remove = ramfs_dir_nolock_lookup(&directory->children, tmpname);
	if (!to_remove) {
        ramfs_dir_unlock(&directory->children);
        vfs_printk("not found>");
		return -ENOENT;
    }

    if (to_remove->node.type == VFS_NODE_DIR
        && ramfs_dir_nolock_count(&to_remove->children)) {
        ramfs_node_refdec(to_remove);
        ramfs_dir_unlock(&directory->children);
        vfs_printk("nonempty>");
		return -EBUSY;
    }

    to_remove->parent = NULL;
    ramfs_dir_nolock_remove(&directory->children, to_remove);
    ramfs_dir_unlock(&directory->children);

    ramfs_node_refdec(to_remove);

	vfs_printk("ok>");

	return 0;
}

VFS_FS_STAT(ramfs_stat)
{
    struct ramfs_node_s *to_stat = ramfs_node_from_vfs(node);

	stat->type = to_stat->node.type;

	switch (to_stat->node.type) {
	case VFS_NODE_DIR:
		stat->size = ramfs_dir_count(&to_stat->children);
        stat->nlink = 1;
		break;

	case VFS_NODE_FILE:
		stat->size = to_stat->data->actual_size;
		stat->nlink = ramfs_data_refcount(to_stat->data);
		break;
	}

	return 0;
}

VFS_FS_CLEANUP(ramfs_cleanup)
{
    struct ramfs_fs_s *ramfs = ramfs_fs_from_vfs(fs);
    struct ramfs_node_s *root = ramfs_node_from_vfs(ramfs->fs.root);

    ramfs_node_refdec(root);
    vfs_fs_cleanup(&ramfs->fs);
	mem_free(ramfs);
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

error_t ramfs_open(struct vfs_fs_s **fs)
{
    error_t err = -ENOMEM;
	struct ramfs_fs_s *ramfs;
    struct ramfs_node_s *root;

    ramfs = mem_alloc(sizeof(*ramfs), mem_scope_sys);
	if (!ramfs)
		goto nomem_fs;

    vfs_printk("ramfs: opening new ramfs volume\n");

    err = vfs_fs_init(&ramfs->fs, &ramfs_ops, 0);
    if (err)
        goto release_fs;

    root = ramfs_node_create(VFS_NODE_DIR, &ramfs->fs, NULL, NULL, 0);
	if (!root)
		goto nomem_dir;

    root->parent = root;

    vfs_fs_root_set(&ramfs->fs, &root->node);

	*fs = &ramfs->fs;

	return 0;

  nomem_dir:
    vfs_fs_cleanup(&ramfs->fs);
 release_fs:
	mem_free(ramfs);
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
    while (i < n && ent) {
        struct ramfs_node_s *nent;

        i++;
        nent = ramfs_dir_nolock_next(&node->children, ent);
        ramfs_node_refdec(ent);
        ent = nent;
    }

    if (!ent) {
        ramfs_dir_unlock(&node->children);
        return 0;
    }

	memcpy(dirent->name, ent->name, CONFIG_VFS_NAMELEN);
	dirent->type = ent->node.type;
	dirent->size = ent->node.type == VFS_NODE_FILE
		? ent->data->actual_size
		: ramfs_dir_count(&ent->children);

	ramfs_dir_unlock(&node->children);

    ramfs_node_refdec(ent);

    return 1;
}

