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
    return (node == node->parent) && (node->fs->root != node->fs_node);
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
        vfs_dir_nolock_remove(&parent->children, node);
        node->parent = node;
        vfs_node_refdrop(parent);
    }
    lock_release(&node->parent_lock);
}

static void
vfs_node_parent_nolock_set_for_root(struct vfs_node_s *node, struct vfs_node_s *parent)
{
    lock_spin(&node->parent_lock);
    node->parent = vfs_node_refnew(parent);
    vfs_dir_nolock_push(&parent->children, node);
    lock_release(&node->parent_lock);
}

static void
vfs_node_parent_nolock_set(struct vfs_node_s *node, struct vfs_node_s *parent)
{
    lock_spin(&node->parent_lock);
	assert( vfs_node_is_dandling(node) );
    node->parent = vfs_node_refnew(parent);
    vfs_dir_nolock_push(&parent->children, node);
    lock_release(&node->parent_lock);
}

static struct vfs_node_s *
vfs_dir_nolock_mangled_lookup(struct vfs_node_s *node,
                              const char *fullname, size_t fullnamelen)
{
	char tmpname[CONFIG_VFS_NAMELEN];
    vfs_name_mangle(fullname, fullnamelen, tmpname);

	VFS_STATS_INC(node, lookup_count);

	return vfs_dir_nolock_lookup(&node->children, tmpname);
}

error_t vfs_mount(struct vfs_node_s *mountpoint,
				  struct vfs_fs_s *fs)
{
    {
        struct vfs_stat_s stat;
        vfs_node_stat(mountpoint, &stat);
        /* Must mount on a directory */
        if ( stat.type != VFS_NODE_DIR )
            return -EINVAL;
    }

	/* Cant mount a mounted fs */
	if ( fs->old_node != NULL )
		return -EBUSY;

	/* Cant mount at the root of the filesystem */
	if ( mountpoint->parent == mountpoint )
		return -EINVAL;

	/* Cant mount at the root of another mount */
	if ( mountpoint->fs->root == mountpoint->fs_node )
		return -EINVAL;

	/* Keep a reference to the mountpoint node, it may be open */
	fs->old_node = vfs_node_refnew(mountpoint);

    struct vfs_node_s *new_node = vfs_node_new(NULL, fs,
                                               mountpoint->name,
                                               strlen(mountpoint->name),
                                               fs->root);
    if ( new_node == NULL )
        return -ENOMEM;

    struct vfs_node_s *parent = vfs_node_get_parent(mountpoint);

	vfs_dir_wrlock(&parent->children);
    vfs_node_parent_nolock_unset(mountpoint);
    vfs_node_parent_nolock_set_for_root(new_node, parent);
	vfs_dir_unlock(&parent->children);

    vfs_node_refdrop(parent);

	vfs_printk("<mount ok>");

	return 0;
}

error_t vfs_umount(struct vfs_node_s *mountpoint)
{
    struct vfs_fs_s *fs = mountpoint->fs;

    /* Ensure mountpoint is a root */
    if ( fs->root != mountpoint->fs_node )
        return -EINVAL;

	/* Cant umount the global root */
	if ( fs->old_node == NULL )
		return -EINVAL;

	struct vfs_node_s *parent = vfs_node_get_parent(mountpoint);

	/* Is user playing with us ? */
	if ( parent == NULL )
		return -EINVAL;

	vfs_dir_wrlock(&parent->children);
	if ( !fs->can_unmount(fs) ) {
        vfs_dir_unlock(&parent->children);
        vfs_node_refdrop(parent);
        return -EBUSY;
    }
        
    /* Reput the old node where it belongs */
    vfs_node_parent_nolock_unset(mountpoint);
    vfs_node_parent_nolock_set(fs->old_node, parent);
    
    vfs_dir_unlock(&parent->children);

    /* Do the refdrop without the parent locked */
    vfs_node_refdrop(mountpoint);
    vfs_node_refdrop(fs->old_node);
    fs->old_node = NULL;

    vfs_node_refdrop(parent);
	return 0;
}

error_t vfs_create_root(struct vfs_fs_s *fs,
                        struct vfs_node_s **mountpoint)
{
	/* Cant mount a mounted fs */
	if ( fs->old_node != NULL )
		return -EBUSY;

    struct vfs_node_s *new_node =
        vfs_node_new(NULL, fs, "", 0, fs->root);

    if ( new_node == NULL )
        return -ENOMEM;

    fs->old_node = NULL;

	vfs_printk("<mount root ok>");

    *mountpoint = new_node;

	return 0;

}

/* Node operations */

error_t vfs_node_lookup(struct vfs_node_s *parent,
						const char *name,
						size_t namelen,
						struct vfs_node_s **node)
{
	error_t err = 0;

    struct fs_node_s *fs_node;

	VFS_STATS_INC(parent->fs, lookup_count);

	vfs_printk("<lookup \"%s\"/%d parent: %p [%s]... ", name, namelen, parent, parent->name);

	/* Dandling nodes are valid, but no lookup is authorized on them... */
    if ( vfs_node_is_dandling(parent) )
		return -EINVAL;

	vfs_dir_wrlock(&parent->children);

	/* Now lookup inside the hash */
	*node = vfs_dir_nolock_mangled_lookup(parent, name, namelen);
	if ( *node ) {
		vfs_node_refnew(*node);
		vfs_printk("ok %p [%s]>", (*node), (*node)->name);
		err = 0;
		goto fini;
	}

	/* Last call: ask the FS */
	err = parent->fs->lookup(parent->fs_node, name, namelen, &fs_node);

	if ( err ) {
		vfs_printk("err %d>", err);
		goto fini;
	}

    *node = vfs_node_new(NULL, parent->fs, name, namelen, fs_node);
    parent->fs->node_refdrop(fs_node);
    if ( *node == NULL ) {
        err = -ENOMEM;
        goto fini;
    }

	/* As FS got a node for this, we can register it in the hash */
    vfs_node_parent_nolock_set(*node, parent);

	vfs_printk("fs %p [%s]>", (*node), (*node)->name);

  fini:
	vfs_dir_unlock(&parent->children);
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

	VFS_STATS_INC(fs, create_count);

    struct fs_node_s *fs_node;

	error_t err = fs->create(fs, type, &fs_node);
    if ( err )
        return err;

    *node = vfs_node_new(NULL, fs, NULL, 0, fs_node);
    fs->node_refdrop(fs_node);

    if ( *node == NULL ) {
        return -ENOMEM;
    }

    return 0;
}

error_t vfs_node_open(struct vfs_node_s *node,
                      enum vfs_open_flags_e flags,
                      struct vfs_file_s **file)
{
	vfs_printk(" node_open(%p): ", node);

    assert( node->fs->node_open != NULL );

	VFS_STATS_INC(node->fs, node_open_count);

    return node->fs->node_open(node->fs_node, flags, file);
}

error_t vfs_node_link(struct vfs_node_s *parent,
					  struct vfs_node_s *node,
					  const char *name,
					  size_t namelen,
					  struct vfs_node_s **rnode)
{
	error_t err = 0;

	vfs_printk("<%s '%s' %p [%s] in %p [%s]... ", __FUNCTION__, name, node, node->name, parent, parent->name);

    if ( parent->fs->link == NULL )
        return -ENOTSUP;

    if ( parent->fs != node->fs )
        return -ENOTSUP;

    if ( parent->fs->flag_ro )
        return -EPERM;

	vfs_dir_wrlock(&parent->children);

	struct vfs_node_s *prev_node = vfs_dir_nolock_mangled_lookup(
        parent, name, namelen);

	VFS_STATS_INC(parent->fs, link_count);

    struct fs_node_s *rfs_node;

	err = parent->fs->link(parent->fs_node, node->fs_node,
                           name, namelen, &rfs_node);
	if ( err ) {
		vfs_printk("fail %d>\n", err);
		goto fini;
	}

    if ( prev_node != NULL ) {
        vfs_node_parent_nolock_unset(prev_node);
    }

    *rnode = vfs_node_new(NULL, parent->fs,
                          name, namelen,
                          rfs_node);
    parent->fs->node_refdrop(rfs_node);
    if ( *rnode == NULL ) {
        /*
          TODO
          Argh ! we did it on the FS, but we cant create the node
          what should we do ?

          We should not return an error, but if we dont, we must
          provide a valid node. There are options:

          * Change the prototype in order not to return the new node ?
          * Allocate the new node before, but we break the ctor/dtor
            which take valid fs_nodes
         */
        err = -ENOMEM;
        goto fini;
    }

    /* As FS got a node for this, we can register it in the hash */
    vfs_node_parent_nolock_set(*rnode, parent);

	vfs_printk("ok>\n");

	err = 0;
  fini:
	vfs_dir_unlock(&parent->children);
	return err;
}

/**
   Unlinks a node from its parent.
 */
error_t vfs_node_unlink(struct vfs_node_s *parent,
						const char *name,
						size_t namelen)
{
    if ( parent->fs->unlink == NULL )
        return -ENOTSUP;

    if ( parent->fs->flag_ro )
        return -EPERM;

	vfs_printk("<%s '%s'... ", __FUNCTION__, name);

	VFS_STATS_INC(parent->fs, unlink_count);

	vfs_dir_wrlock(&parent->children);

	struct vfs_node_s *node = vfs_dir_nolock_mangled_lookup(parent, name, namelen);

	error_t err = parent->fs->unlink(parent->fs_node, name, namelen);
	if ( err )
		goto fini;

	if ( node )
        vfs_node_parent_nolock_unset(node);

	err = 0;
  fini:
	vfs_dir_unlock(&parent->children);
	vfs_printk(" %s>", strerror(err));
	return err;
}

error_t vfs_node_stat(struct vfs_node_s *node,
					  struct vfs_stat_s *stat)
{
	VFS_STATS_INC(node->fs, stat_count);
	VFS_STATS_INC(node, stat_count);
	return node->fs->stat(node->fs_node, stat);
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
	const char *fullname = va_arg(ap, const char *);
	size_t fullnamelen = va_arg(ap, size_t);

    vfs_name_mangle(fullname, fullnamelen, obj->name);
	obj->fs = fs;

    obj->fs_node = obj->fs->node_refnew(va_arg(ap, struct fs_node_s *));

	obj->parent = obj;
    lock_init(&obj->parent_lock);

	atomic_inc(&fs->ref);
	VFS_STATS_INC(fs, node_create_count);

#if defined(CONFIG_VFS_STATS)
    atomic_set(&obj->lookup_count, 0);
    atomic_set(&obj->open_count, 0);
    atomic_set(&obj->close_count, 0);
    atomic_set(&obj->stat_count, 0);
#endif

    vfs_dir_init(&obj->children);

	vfs_printk("<node create %p %p '%s'>", fs, obj, obj->name);

	return 0;
}

OBJECT_DESTRUCTOR(vfs_node)
{
	vfs_printk("<node delete %p '%s'", obj, obj->name);

    struct vfs_node_s *parent = vfs_node_get_parent(obj);
    if ( parent ) {
        vfs_dir_wrlock(&parent->children);
        vfs_node_parent_nolock_unset(obj);
        vfs_dir_unlock(&parent->children);
        vfs_node_refdrop(parent);
    }

    /* Now we are not accessible from anywhere, nobody can refnew us
     * any more */
    if ( vfs_node_refcount(obj) == 1 ) {
        vfs_dir_destroy(&obj->children);

        atomic_dec(&obj->fs->ref);

        VFS_STATS_INC(obj->fs, node_destroy_count);

        obj->fs->node_refdrop(obj->fs_node);
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

