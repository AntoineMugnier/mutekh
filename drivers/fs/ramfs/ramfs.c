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
#include "ramfs_data.h"
#include "ramfs_file.h"


OBJECT_PROTOTYPE         (ramfs_node, static inline, ramfs_node);
CONTAINER_FUNC           (ramfs_dir_hash, HASHLIST, static inline, ramfs_dir, name);
CONTAINER_FUNC_NOLOCK    (ramfs_dir_hash, HASHLIST, static inline, ramfs_dir_nolock, name);
CONTAINER_KEY_FUNC       (ramfs_dir_hash, HASHLIST, static inline, ramfs_dir, name);
CONTAINER_KEY_FUNC_NOLOCK(ramfs_dir_hash, HASHLIST, static inline, ramfs_dir_nolock, name);
OBJECT_FUNC              (ramfs_node, REFCOUNT, static inline, ramfs_node, obj_entry);

static VFS_NODE_FS_PRIV_DELETER(ramfs_node_deletepriv);

OBJECT_CONSTRUCTOR(ramfs_node)
{
    vfs_printk("<ramfs_node_ctor");
    enum vfs_node_type_e type = va_arg(ap, enum vfs_node_type_e);
    obj->type = type;
    if ( type == VFS_NODE_FILE ) {
        struct ramfs_data_s *data = va_arg(ap, struct ramfs_data_s*);
        if ( data == NULL ) {
            obj->data = ramfs_data_new(NULL);
            if ( obj->data == NULL ) {
                return -ENOMEM;
            }
        } else
            obj->data = ramfs_data_refnew(data);
    } else {
        ramfs_dir_init(&obj->children);
    }
    vfs_printk(">");
    return 0;
}

OBJECT_DESTRUCTOR(ramfs_node)
{
    vfs_printk("<ramfs_node_dtor");
    if ( obj->type == VFS_NODE_DIR ) {
        ramfs_dir_clear(&obj->children);
    } else {
        ramfs_data_refdrop(obj->data);
    }
    vfs_printk(">");
}

static VFS_NODE_FS_PRIV_DELETER(ramfs_node_deletepriv)
{
    vfs_printk("<ramfs_delpriv");
    struct ramfs_node_s *rfs_node = node->priv;
    ramfs_node_refdrop(rfs_node);
    vfs_printk(">");
}

void ramfs_dump_item(struct ramfs_node_s *node, size_t pf)
{
    size_t i;
    for ( i=0; i<pf; ++i )
        printk(" ");
    printk(" %s %d '%s': %d\n",
           node->type == VFS_NODE_DIR ? ">" : "-",
           node->obj_entry.refcnt.value,
           node->name,
           node->type == VFS_NODE_FILE
           ? node->data->obj_entry.refcnt.value
           : 0);
    if ( node->type == VFS_NODE_DIR )
		CONTAINER_FOREACH(ramfs_dir_hash, HASHLIST, &node->children, {
                ramfs_dump_item(item, pf+2);
            });
}

void ramfs_dump(struct vfs_fs_s *fs)
{
    printk("Ramfs dump. ref: %d\n", atomic_get(&fs->ref));
    vfs_fs_dump_stats(fs);
    struct ramfs_node_s *root = fs->root->priv;
    ramfs_dump_item(root, 0);
}



VFS_FS_CAN_UNMOUNT(ramfs_can_unmount)
{
	return atomic_get(&fs->ref) == 1;
}

VFS_FS_LOOKUP(ramfs_lookup)
{
    struct ramfs_node_s *rfs_node = ref->priv;

    vfs_printk("<%s %s/%d ", __FUNCTION__, name, namelen);

    if ( rfs_node->type != VFS_NODE_DIR )
        return -EINVAL;

    char vfsname[CONFIG_VFS_NAMELEN];
	memset(vfsname, 0, CONFIG_VFS_NAMELEN);
    memcpy(vfsname, name, namelen);

    struct ramfs_node_s *child = ramfs_dir_lookup(&rfs_node->children, vfsname);
    if ( child ) {
        struct vfs_node_s *found_node = vfs_node_new(NULL, ref->fs,
                                                     child->type, name, namelen,
                                                     child, ramfs_node_deletepriv);
        // TODO ?
//        ramfs_node_refdrop(child);
        if ( found_node == NULL ) {
            vfs_printk(" nomem>");
            ramfs_node_refdrop(child);
            return -ENOMEM;
        }
        *node = found_node;
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
    rfs_node = ramfs_node_new(NULL, type, NULL);
	if ( !rfs_node )
		goto err_priv;
	struct vfs_node_s *rnode = vfs_node_new(NULL, fs, type, "", 0,
											rfs_node, ramfs_node_deletepriv);
	if ( !rnode )
		goto err_rnode;

	*node = rnode;

	vfs_printk("ok>\n");

	return 0;
  err_rnode:
    ramfs_node_refdrop(rfs_node);
  err_priv:
	vfs_printk("err>\n");
	return -ENOMEM;
}

VFS_FS_LINK(ramfs_link)
{
	if ( namelen >= CONFIG_VFS_NAMELEN )
		return -EINVAL;

	char tmpname[CONFIG_VFS_NAMELEN];
	memset(tmpname, 0, CONFIG_VFS_NAMELEN);
	memcpy(tmpname, name, namelen);

	/* We cant support hardlinks of directories */
    struct vfs_node_s *node_parent = vfs_node_get_parent(node);

	if ( (node_parent != NULL) && (node->type == VFS_NODE_DIR) ) {
        vfs_node_refdrop(node_parent);
		return -EINVAL;
    }

	vfs_printk("<%s %s ", __FUNCTION__, tmpname);

    struct ramfs_node_s *rfs_parent = parent->priv;

    ramfs_dir_wrlock(&rfs_parent->children);

    struct ramfs_node_s *rfs_curnode = node->priv;

    struct ramfs_node_s *old_file = ramfs_dir_nolock_lookup(&rfs_parent->children,
                                                     tmpname);
    if ( old_file ) {
        ramfs_dir_nolock_remove(&rfs_parent->children, old_file);
        ramfs_node_refdrop(old_file);
    }

    struct ramfs_node_s *rfs_node;
	struct vfs_node_s *nnode;
	if ( node_parent != NULL ) {
        rfs_node = ramfs_node_new(NULL, VFS_NODE_FILE, ramfs_data_refnew(rfs_curnode->data));
        vfs_printk("clone (parent=%p) ", node_parent);
		nnode = vfs_node_new(NULL, parent->fs, VFS_NODE_FILE, name, namelen,
							 rfs_node, ramfs_node_deletepriv);
		if (nnode == NULL) {
			vfs_printk("failed>");
            ramfs_dir_unlock(&rfs_parent->children);
			return -ENOMEM;
		}
	} else {
		vfs_printk("use ");
		nnode = vfs_node_refnew(node);
        rfs_node = rfs_curnode;
	}

    memcpy(rfs_node->name, name, namelen);
	memset(rfs_node->name + namelen, 0, CONFIG_VFS_NAMELEN - namelen);
    ramfs_dir_nolock_push(&rfs_parent->children, rfs_node);

    ramfs_dir_unlock(&rfs_parent->children);

    memcpy(nnode->name, name, namelen);
	memset(nnode->name + namelen, 0, CONFIG_VFS_NAMELEN - namelen);

	*rnode = nnode;

	vfs_printk("ok>");

	if ( node_parent != NULL )
        vfs_node_refdrop(node_parent);

	return 0;
}

VFS_FS_UNLINK(ramfs_unlink)
{
	char tmpname[CONFIG_VFS_NAMELEN];
	memset(tmpname, 0, CONFIG_VFS_NAMELEN);
	memcpy(tmpname, name, namelen);

	vfs_printk("<%s %s ", __FUNCTION__, tmpname);

    struct ramfs_node_s *rfs_parent = parent->priv;

    ramfs_dir_wrlock(&rfs_parent->children);

    struct ramfs_node_s *rfs_node = ramfs_dir_nolock_lookup(&rfs_parent->children, tmpname);
	if ( rfs_node == NULL ) {
        ramfs_dir_unlock(&rfs_parent->children);
        vfs_printk("not found>");
		return -ENOENT;
    }

    if ( (rfs_node->type == VFS_NODE_DIR)
         && (ramfs_dir_nolock_count(&rfs_node->children)) ) {
        ramfs_node_refdrop(rfs_node);
        ramfs_dir_unlock(&rfs_parent->children);
        vfs_printk("nonempty>");
		return -EBUSY;
    }

    ramfs_dir_nolock_remove(&rfs_parent->children, rfs_node);
    ramfs_dir_unlock(&rfs_parent->children);

    ramfs_node_refdrop(rfs_node);

	vfs_printk("ok>");

	return 0;
}

VFS_FS_STAT(ramfs_stat)
{
	stat->type = node->type;
	stat->nlink = 1;

    struct ramfs_node_s *rfs_node = node->priv;

	switch ( node->type ) {
	case VFS_NODE_DIR:
		stat->size = ramfs_dir_count(&rfs_node->children);
		break;
	case VFS_NODE_FILE: {
		struct ramfs_data_s *data = rfs_node->data;
		stat->size = data->actual_size;
		stat->nlink = ramfs_data_refcount(data);
		break;
	}
	}

	return 0;
}



error_t ramfs_close(struct vfs_fs_s *fs)
{
	return -EBUSY;
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

    struct ramfs_node_s *root = ramfs_node_new(NULL, VFS_NODE_DIR);

	struct vfs_node_s *node = vfs_node_new(NULL, mnt, VFS_NODE_DIR, "", 0,
                                           root, ramfs_node_deletepriv);
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
        ramfs_node_refdrop(ent);
        ent = nent;
    }

    if ( ent == NULL ) {
        ramfs_dir_unlock(&node->children);
        return 0;
    }

	memcpy( dirent->name, ent->name, CONFIG_VFS_NAMELEN );
	dirent->type = ent->type;
	dirent->size = ent->type == VFS_NODE_FILE
		? ent->data->actual_size
		: ramfs_dir_count(&ent->children);

	ramfs_dir_unlock(&node->children);

    ramfs_node_refdrop(ent);

    return 1;
}


// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

