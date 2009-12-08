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

CONTAINER_KEY_TYPE(vfs_dir_hash, BLOB, name, CONFIG_VFS_NAMELEN);

CONTAINER_FUNC       (vfs_dir_hash, HASHLIST, static, vfs_dir, name);
CONTAINER_KEY_FUNC   (vfs_dir_hash, HASHLIST, static, vfs_dir, name);
CONTAINER_FUNC_NOLOCK(vfs_dir_hash, HASHLIST, static, vfs_dir_nolock, name);
CONTAINER_KEY_FUNC_NOLOCK(vfs_dir_hash, HASHLIST, static, vfs_dir_nolock, name);

static inline
bool_t vfs_node_is_dandling(struct vfs_node_s *node)
{
    return (node == node->parent) && (node->fs->root != node);
}

struct vfs_node_s *vfs_node_get_parent(struct vfs_node_s *node)
{
    struct vfs_node_s *parent = NULL;

    lock_spin(&node->parent_lock);
	if ( !vfs_node_is_dandling(node) )
        parent = vfs_node_refnew(node->parent);
    lock_release(&node->parent_lock);

    return parent;
}

static void
vfs_node_parent_nolock_unset(struct vfs_node_s *node)
{
    lock_spin(&node->parent_lock);
	if ( !vfs_node_is_dandling(node) ) {
        struct vfs_node_s *parent = node->parent;
        vfs_dir_nolock_remove(&parent->dir.children, node);
        node->parent = node;
        vfs_node_refdrop(parent);
    }
    lock_release(&node->parent_lock);
}

static void
vfs_node_parent_nolock_set(struct vfs_node_s *node, struct vfs_node_s *parent)
{
    lock_spin(&node->parent_lock);
	assert( vfs_node_is_dandling(node) );

    node->parent = vfs_node_refnew(parent);
    vfs_dir_nolock_push(&parent->dir.children, node);
    lock_release(&node->parent_lock);
}

static struct vfs_node_s *
vfs_dir_nolock_mangled_lookup(struct vfs_node_s *node,
                              const char *fullname, size_t fullnamelen)
{
	char tmpname[CONFIG_VFS_NAMELEN];
    vfs_name_mangle(fullname, fullnamelen, tmpname);

#if defined(CONFIG_VFS_STATS)
	atomic_inc(&node->lookup_count);
#endif

	return vfs_dir_nolock_lookup(&node->dir.children, tmpname);
}

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

	/* Keep a reference to the mountpoint node, it may be open */
	fs->old_node = vfs_node_refnew(mountpoint);

	/* Replace entry in parent's hash, with correct name */
	memcpy(fs_root->name, mountpoint->name, CONFIG_VFS_NAMELEN);

    struct vfs_node_s *parent = vfs_node_get_parent(mountpoint);

	vfs_dir_wrlock(&parent->dir.children);
    vfs_node_parent_nolock_unset(mountpoint);
    vfs_node_parent_nolock_set(fs_root, parent);
	vfs_dir_unlock(&parent->dir.children);

    vfs_node_refdrop(parent);

	vfs_printk("<mount ok>");

	return 0;
}

error_t vfs_umount(struct vfs_fs_s *fs)
{
	struct vfs_node_s *fs_root = fs->root;
    error_t err = -EBUSY;

	/* Cant umount the global root */
	if ( fs->old_node == NULL )
		return -EINVAL;

	/* This should not happen, but I want to be sure... */
	if ( fs_root->parent == NULL )
		return -EINVAL;

	struct vfs_node_s *parent = vfs_node_get_parent(fs_root);

	/* >>> Critical section */
	vfs_dir_wrlock(&parent->dir.children);

	if ( fs->can_unmount(fs) ) {
        err = 0;

        /* Reput the old node where it belongs */
        vfs_node_parent_nolock_unset(fs_root);
        vfs_node_parent_nolock_set(fs->old_node, parent);
        
        fs->old_node = NULL;

        fs_root->parent = fs_root;

        /* Reset the root name */
        memset(fs_root->name, 0, CONFIG_VFS_NAMELEN);
    }

	vfs_dir_unlock(&parent->dir.children);
	/* <<< Critical section */

    /* Do the refdrop without the parent locked */
    if ( err == 0 )
        vfs_node_refdrop(fs->old_node);

    vfs_node_refdrop(parent);
	return err;
}

/* Node operations */

error_t vfs_node_lookup(struct vfs_node_s *parent,
						const char *name,
						size_t namelen,
						struct vfs_node_s **node)
{
	error_t err = 0;

    if ( parent->type != VFS_NODE_DIR )
        return -EINVAL;

	vfs_printk("<lookup \"%s\"/%d parent: %p [%s]... ", name, namelen, parent, parent->name);

	/* Dandling nodes are valid, but no lookup is authorized on them... */
    if ( vfs_node_is_dandling(parent) )
		return -EINVAL;

	vfs_dir_wrlock(&parent->dir.children);

	/* Now lookup inside the hash */
	*node = vfs_dir_nolock_mangled_lookup(parent, name, namelen);
	if ( *node ) {
		vfs_node_refnew(*node);
		vfs_printk("ok %p [%s]>", (*node), (*node)->name);
		err = 0;
		goto fini;
	}

	/* Last call: ask the FS */
	err = parent->fs->lookup(parent, name, namelen, node);
#if defined(CONFIG_VFS_STATS)
	atomic_inc(&parent->fs->lookup_count);
#endif
	if ( err ) {
		vfs_printk("err %d>", err);
		goto fini;
	}

    assert((*node)->name[0]);   /* not an anonymous node */

	/* As FS got a node for this, we can register it in the hash */
    vfs_node_parent_nolock_set(*node, parent);

	vfs_printk("fs %p [%s]>", (*node), (*node)->name);

  fini:
	vfs_dir_unlock(&parent->dir.children);
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

#if defined(CONFIG_VFS_STATS)
	atomic_inc(&fs->create_count);
#endif
	return fs->create(fs, type, node);
}

error_t vfs_node_open(struct vfs_fs_s *fs,
                      struct vfs_node_s *node,
                      enum vfs_open_flags_e flags,
                      struct vfs_file_s **file)
{
	vfs_printk(" node_open[%p:%p](%p): ", fs, node->fs->node_open, node);

    assert( fs->node_open != NULL );

    /* check open mode en permission based on node type */
    switch ( node->type ) {
    case VFS_NODE_DIR:
        if ( flags != (VFS_OPEN_DIR | VFS_OPEN_READ) )
            return -EINVAL;
        break;

    case VFS_NODE_FILE:
        if ( flags & VFS_OPEN_DIR )
            return -EINVAL;

        if ( !(flags & (VFS_OPEN_READ | VFS_OPEN_WRITE) ) )
            return -EINVAL;

        if ( flags & (VFS_OPEN_WRITE | VFS_OPEN_CREATE | VFS_OPEN_APPEND) && fs->flag_ro )
            return -EPERM;
        break;
    }

#if defined(CONFIG_VFS_STATS)
	atomic_inc(&fs->node_open_count);
#endif
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

    if ( parent->fs != node->fs )
        return -ENOTSUP;

    if ( parent->fs->flag_ro )
        return -EPERM;

	vfs_dir_wrlock(&parent->dir.children);

	struct vfs_node_s *prev_node = vfs_dir_nolock_mangled_lookup(parent, name, namelen);

	if ( prev_node ) {
		if ( prev_node->type == VFS_NODE_DIR ) {
			err = -EBUSY;
			goto fini;
		} else {
            vfs_node_parent_nolock_unset(prev_node);
		}
	}

#if defined(CONFIG_VFS_STATS)
	atomic_inc(&parent->fs->link_count);
#endif
	err = parent->fs->link(parent, node, name, namelen, rnode);
	if ( err ) {
		vfs_printk("fail %d>\n", err);
		goto fini;
	}

    assert((*rnode)->name[0]);

    vfs_node_parent_nolock_set(*rnode, parent);

	vfs_printk("ok>\n");

	err = 0;
  fini:
	vfs_dir_unlock(&parent->dir.children);
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

	vfs_printk("<%s '%s'... ", __FUNCTION__, name);

	vfs_dir_wrlock(&parent->dir.children);

	struct vfs_node_s *node = vfs_dir_nolock_mangled_lookup(parent, name, namelen);

#if defined(CONFIG_VFS_STATS)
	atomic_inc(&parent->fs->unlink_count);
#endif
	error_t err = parent->fs->unlink(parent, name, namelen);
	if ( err )
		goto fini;

	if ( node )
        vfs_node_parent_nolock_unset(node);

	err = 0;
  fini:
	vfs_dir_unlock(&parent->dir.children);
	vfs_printk(" %s>", strerror(err));
	return err;
}

error_t vfs_node_stat(struct vfs_node_s *node,
					  struct vfs_stat_s *stat)
{
#if defined(CONFIG_VFS_STATS)
	atomic_inc(&node->fs->stat_count);
	atomic_inc(&node->stat_count);
#endif
	return node->fs->stat(node, stat);
}

size_t vfs_name_mangle(const char *fullname, size_t fulllen, char *vfsname)
{
    /* FIXME should gracefully handle shortened names colision */
    if (fulllen > CONFIG_VFS_NAMELEN)
        fulllen = CONFIG_VFS_NAMELEN;
    /* fulllen can be 0 for anonymous nodes only */
    memcpy(vfsname, fullname, fulllen);
	memset(vfsname + fulllen, 0, CONFIG_VFS_NAMELEN - fulllen);

    return fulllen;
}

bool_t vfs_name_compare(const char *fullname, size_t fulllen,
                        const char *vfsname, size_t vfsnamelen)
{
    /* should compare vfsname with both fullname and shortened name */

    assert(vfsnamelen && fulllen);

    if (fulllen == vfsnamelen && !memcmp(fullname, vfsname, fulllen))
        return 1;

    if (fulllen > CONFIG_VFS_NAMELEN) {
        /* FIXME should gracefully handle shortened names compare HERE */
    }

    return 0;
}

OBJECT_CONSTRUCTOR(vfs_node)
{
	struct vfs_fs_s *fs = va_arg(ap, struct vfs_fs_s *);
	obj->type = va_arg(ap, enum vfs_node_type_e);
	const char *fullname = va_arg(ap, const char *);
	size_t fullnamelen = va_arg(ap, size_t);
	obj->priv = va_arg(ap, void*);
	obj->priv_deleter = va_arg(ap, vfs_node_fs_priv_deleter_t*);

    vfs_name_mangle(fullname, fullnamelen, obj->name);
	obj->fs = fs;

    lock_init(&obj->parent_lock);

	atomic_inc(&fs->ref);
#if defined(CONFIG_VFS_STATS)
	atomic_inc(&fs->node_create_count);
    atomic_set(&obj->lookup_count, 0);
    atomic_set(&obj->open_count, 0);
    atomic_set(&obj->close_count, 0);
    atomic_set(&obj->stat_count, 0);
#endif

	obj->parent = obj;

	vfs_printk("<node create %p %p '%s'", fs, obj, obj->name);

	if ( obj->type == VFS_NODE_DIR )
		vfs_dir_init(&obj->dir.children);

	vfs_printk(">");

	return 0;
}

OBJECT_DESTRUCTOR(vfs_node)
{
	vfs_printk("<node delete %p '%s'", obj, obj->name);

    struct vfs_node_s *parent = vfs_node_get_parent(obj);
    if ( parent ) {
        vfs_dir_wrlock(&parent->dir.children);
        vfs_node_parent_nolock_unset(obj);
        vfs_dir_unlock(&parent->dir.children);
        vfs_node_refdrop(parent);
    }

    /* Now we are not accessible from anywhere, nobody can refnew us
     * any more */
    if ( vfs_node_refcount(obj) == 1 ) {
        if (obj->type == VFS_NODE_DIR) {
            vfs_dir_destroy(&obj->dir.children);
        }

        atomic_dec(&obj->fs->ref);

#if defined(CONFIG_VFS_STATS)
        atomic_inc(&obj->fs->node_destroy_count);
#endif

        if ( obj->priv && obj->priv_deleter )
            obj->priv_deleter(obj);
        vfs_printk(" done>");
    } else {
        vfs_printk(" cancel>");
    }
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

