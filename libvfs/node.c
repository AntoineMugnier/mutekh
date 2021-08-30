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

#include <vfs/node.h>
#include <vfs/file.h>
#include <vfs/name.h>
#include <vfs/fs.h>
#include "vfs-private.h"
#include <mutek/printk.h>
#include <mutek/mem_alloc.h>

#include <string.h>

ssize_t vfs_node_get_name(struct vfs_node_s *node,
                          char *name,
                          size_t namelen)
{
    strncpy(name, node->name, CONFIG_VFS_NAMELEN);
    return (strlen(name) < CONFIG_VFS_NAMELEN) ? 0 : -ENOMEM;
}

void vfs_node_name_set(struct vfs_node_s *node, const char *name, size_t size)
{
    assert(vfs_node_is_dangling(node));

    if (name && size)
        vfs_name_mangle(name, size, node->name);
    else
        memset(node->name, 0, CONFIG_VFS_NAMELEN);
}

struct vfs_fs_s *vfs_node_get_fs(struct vfs_node_s *node)
{
    return node->fs;
}

error_t vfs_node_init(struct vfs_node_s *node,
                      struct vfs_fs_s *fs,
                      enum vfs_node_type_e type,
                      const char *name, size_t name_size)
{
    vfs_node_refinit(node);

	node->fs = vfs_fs_refinc(fs);
    node->type = type;
	node->parent = NULL;
    lock_init(&node->parent_lock);

	VFS_STATS_INC(fs, node_create_count);

    semaphore_init(&node->dir_semaphore, 1);

#if defined(CONFIG_VFS_STATS)
    atomic_set(&node->lookup_count, 0);
    atomic_set(&node->open_count, 0);
    atomic_set(&node->close_count, 0);
    atomic_set(&node->stat_count, 0);
#endif

    vfs_node_name_set(node, name, name_size);

    vfs_dir_init(&node->children);

	vfs_printk("<node init %p %p '%s'>",
               fs, node, node->name);

    return 0;
}

void vfs_node_cleanup(struct vfs_node_s *node)
{
	vfs_printk("<node cleanup %p '%s'", node, node->name);

    struct vfs_node_s *parent = vfs_node_get_parent(node);
    if (parent) {
        vfs_node_dirlock(parent);
        vfs_node_parent_nolock_unset(node);
        vfs_node_dirunlock(parent);
        vfs_node_refdec(parent);
    }
    
    vfs_dir_destroy(&node->children);

    semaphore_destroy(&node->dir_semaphore);

    VFS_STATS_INC(node->fs, node_destroy_count);

    lock_destroy(&node->parent_lock);

	vfs_fs_refdec(node->fs);
    vfs_node_refcleanup(node);

    vfs_printk(" done>");
}

void vfs_node_destroy(struct vfs_node_s *node)
{
    if (node->fs->ops->node_cleanup)
        node->fs->ops->node_cleanup(node);

    vfs_node_cleanup(node);

    mem_free(node);
}

struct vfs_node_s *vfs_node_get_parent(struct vfs_node_s *node)
{
    struct vfs_node_s *parent = NULL;

    CPU_INTERRUPT_SAVESTATE_DISABLE;
    lock_spin(&node->parent_lock);
	if (!vfs_node_is_dangling(node))
        parent = vfs_node_refinc(node->parent);
    lock_release(&node->parent_lock);
    CPU_INTERRUPT_RESTORESTATE;

    return parent;
}

void
vfs_node_parent_nolock_unset(struct vfs_node_s *node)
{
    vfs_printk("<node %p: unsetting parent>", node);

    CPU_INTERRUPT_SAVESTATE_DISABLE;
    lock_spin(&node->parent_lock);

	if (!vfs_node_is_dangling(node)) {
        struct vfs_node_s *parent = node->parent;
        vfs_dir_remove(&parent->children, node);
        node->parent = NULL;
        vfs_node_refdec(parent);
    }

    lock_release(&node->parent_lock);
    CPU_INTERRUPT_RESTORESTATE;
}


static
void vfs_dump_item(struct vfs_node_s *node,
				   size_t pfx)
{
	size_t i;
	for (i=0; i<pfx; ++i)
		printk(" ");
    printk(" + %d \"%s\" %p (%p)"
#if defined(CONFIG_VFS_STATS)
           ", lu: %d, open: %d, close: %d, stat: %d"
#endif
//           ", free: %p"
           "\n"
           , vfs_node_refcount(node)
           , node->name
           , node, node->parent
#if defined(CONFIG_VFS_STATS)
           , atomic_get(&node->lookup_count)
           , atomic_get(&node->open_count)
           , atomic_get(&node->close_count)
           , atomic_get(&node->stat_count)
#endif
//           , node->obj_entry.storage_free
        );

    GCT_FOREACH(vfs_dir_hash, &node->children, item, {
            vfs_dump_item(item, pfx+2);
        });
}

/*
  Here are two helpers when we need to take two locks at the same time
  in two directories. We MUST always take them in the same order to
  avoid deadlocks. So we take them in pointer order, and release them
  in the opposite order.
 */
void vfs_node_2dirlock(struct vfs_node_s *d1,
                       struct vfs_node_s *d2)
{
    if (d1 < d2) {
        vfs_node_dirlock(d1);
        vfs_node_dirlock(d2);
    } else {
        vfs_node_dirlock(d2);
        vfs_node_dirlock(d1);
    }
}

void vfs_node_2dirunlock(struct vfs_node_s *d1,
                         struct vfs_node_s *d2)
{
    if (d1 < d2) {
        vfs_node_dirunlock(d2);
        vfs_node_dirunlock(d1);
    } else {
        vfs_node_dirunlock(d1);
        vfs_node_dirunlock(d2);
    }
}

static void
vfs_node_parent_nolock_set_for_root(struct vfs_node_s *node, struct vfs_node_s *parent)
{
    CPU_INTERRUPT_SAVESTATE_DISABLE;
    lock_spin(&node->parent_lock);

    node->parent = vfs_node_refinc(parent);
    vfs_dir_push(&parent->children, node);

    lock_release(&node->parent_lock);
    CPU_INTERRUPT_RESTORESTATE;
}

static void
vfs_node_parent_nolock_set(struct vfs_node_s *node, struct vfs_node_s *parent)
{
    vfs_printk("<node %p: setting parent %p \"%s\">", node, parent, node->name);

    CPU_INTERRUPT_SAVESTATE_DISABLE;
    lock_spin(&node->parent_lock);

	assert(vfs_node_is_dangling(node));
    node->parent = vfs_node_refinc(parent);
    vfs_dir_push(&parent->children, node);

    lock_release(&node->parent_lock);
    CPU_INTERRUPT_RESTORESTATE;
}

static struct vfs_node_s *
vfs_dir_mangled_lookup(struct vfs_node_s *node,
                              const char *fullname, size_t fullnamelen)
{
	char tmpname[CONFIG_VFS_NAMELEN];
    vfs_name_mangle(fullname, fullnamelen, tmpname);

	VFS_STATS_INC(node, lookup_count);

    /* Take a ref on node, vfs_dir has no refcounting on its children */
	struct vfs_node_s *ret = vfs_dir_lookup(&node->children, tmpname);

    if (ret)
        return vfs_node_refinc(ret);

    return ret;
}

error_t vfs_mount(struct vfs_node_s *mountpoint,
				  struct vfs_fs_s *fs)
{
    struct vfs_stat_s stat;
    vfs_node_stat(mountpoint, &stat);
    /* Must mount on a directory */
    if (stat.type != VFS_NODE_DIR)
        return -EINVAL;

	/* Cant mount a mounted fs */
	if (fs->old_node != NULL)
		return -EBUSY;

	/* Cant mount at the root of the filesystem */
	if (mountpoint->parent == mountpoint)
		return -EINVAL;

	/* Cant mount at the root of another mount */
	if (mountpoint->fs->root == mountpoint)
		return -EINVAL;

	/* Keep a reference to the mountpoint node, it may be open */
	fs->old_node = vfs_node_refinc(mountpoint);

    fs->root->parent = NULL;

    struct vfs_node_s *parent = vfs_node_get_parent(mountpoint);

	vfs_node_dirlock(parent);
    vfs_node_parent_nolock_unset(mountpoint);
    memcpy(fs->root->name, mountpoint->name, CONFIG_VFS_NAMELEN);
    vfs_node_parent_nolock_set_for_root(fs->root, parent);
	vfs_node_dirunlock(parent);

    vfs_node_refdec(parent);

    /* Reference root had on itself */
    vfs_node_refdec(fs->root);

	vfs_printk("<mount ok>");

	return 0;
}

error_t vfs_umount(struct vfs_node_s *mountpoint)
{
    struct vfs_fs_s *fs = mountpoint->fs;

    /* Ensure mountpoint is a root */
    if (fs->root != mountpoint)
        return -EINVAL;

	/* Cant umount the global root */
	if (fs->old_node == NULL)
		return -EINVAL;

	struct vfs_node_s *parent = vfs_node_get_parent(mountpoint);

	/* Is user playing with us ? */
	if (parent == NULL)
		return -EINVAL;

	vfs_node_dirlock(parent);
	if (!fs->ops->can_unmount(fs)) {
        vfs_node_dirunlock(parent);
        vfs_node_refdec(parent);
        return -EBUSY;
    }

    /* Take reference for fs->root->parent now */
    fd_node_refinc(fs->root);

    /* Reput the old node where it belongs */
    vfs_node_parent_nolock_unset(mountpoint);
    memset(mountpoint->name, 0, CONFIG_VFS_NAMELEN);

    /* Old node may not have changed name */
    vfs_node_parent_nolock_set(fs->old_node, parent);

    vfs_node_dirunlock(parent);

    fs->root->parent = fs->root;

    vfs_node_refdec(fs->old_node);
    fs->old_node = NULL;

    vfs_node_refdec(parent);
	return 0;
}

/* Node operations */

error_t vfs_node_lookup(struct vfs_node_s *parent,
						const char *name,
						size_t namelen,
						struct vfs_node_s **node)
{
	error_t err = 0;

    struct vfs_node_s *fs_node;

	VFS_STATS_INC(parent->fs, lookup_count);

	vfs_printk("<lookup \"%s\"/%d parent: %p [%s]... ", name, namelen, parent, parent->name);

	/* Dandling nodes are valid, but no lookup is authorized on them... */
    if (vfs_node_is_dangling(parent))
		return -EINVAL;

	vfs_node_dirlock(parent);

	/* Now lookup inside the hash */
	*node = vfs_dir_mangled_lookup(parent, name, namelen);
	if (*node) {
		vfs_printk("ok %p [%s]>", (*node), (*node)->name);
		err = 0;
		goto fini;
	}

    char mangled_name[CONFIG_VFS_NAMELEN];

	/* Last call: ask the FS */
	err = parent->fs->ops->lookup(parent, name, namelen, &fs_node, mangled_name);

	if (err) {
		vfs_printk("err %d>", err);
		goto fini;
	}

    *node = fs_node;

	/* As FS got a node for this, we can register it in the hash */
    vfs_node_name_set(fs_node, name, namelen);
    vfs_node_parent_nolock_set(*node, parent);

	vfs_printk("fs %p [%s]>", (*node), (*node)->name);

  fini:
	vfs_node_dirunlock(parent);
	return err;
}

error_t vfs_node_anon_create(struct vfs_fs_s *fs,
						enum vfs_node_type_e type,
						struct vfs_node_s **node)
{
    if (fs->ops->create == NULL)
        return -ENOTSUP;

    if (fs->flag_ro)
        return -EPERM;

	VFS_STATS_INC(fs, create_count);

	error_t err = fs->ops->create(fs, type, node);
    if (err)
        return err;

    if (*node == NULL)
        return -ENOMEM;

    return 0;
}

error_t vfs_node_open(struct vfs_node_s *node,
                      enum vfs_open_flags_e flags,
                      struct vfs_file_s **file)
{
	vfs_printk(" node_open(%p): ", node);

    assert(node->fs->ops->node_open);

    if (flags & VFS_OPEN_WRITE && node->fs->flag_ro)
        return -EPERM;

	VFS_STATS_INC(node->fs, node_open_count);

    return node->fs->ops->node_open(node, flags, file);
}

error_t vfs_node_link(struct vfs_node_s *node,
					  struct vfs_node_s *target,
					  const char *name,
					  size_t namelen,
					  struct vfs_node_s **linked_node)
{
    struct vfs_node_s *prev_node;
	error_t err = 0;

	vfs_printk("<%s '%s' %p [%s] in %p [%s]... ", __FUNCTION__, name, node, node->name, target, target->name);

    if (target->fs->ops->link == NULL)
        return -ENOTSUP;

    if (target->fs != node->fs)
        return -ENOTSUP;

    if (vfs_node_is_dangling(target))
        return -ENOTSUP;

    if (target->fs->flag_ro)
        return -EPERM;

	VFS_STATS_INC(target->fs, link_count);

	vfs_node_dirlock(target);
	prev_node = vfs_dir_mangled_lookup(target, name, namelen);
	err = target->fs->ops->link(node, target, name, namelen, linked_node);

	if (err) {
		vfs_printk("fail %d>\n", err);
		goto out;
	}

    if (prev_node)
        vfs_node_parent_nolock_unset(prev_node);

    vfs_node_name_set(*linked_node, name, namelen);
    vfs_node_parent_nolock_set(*linked_node, target);

	vfs_printk("ok>\n");

	err = 0;

  out:
	vfs_node_dirunlock(target);

    if (prev_node != NULL)
        vfs_node_refdec(prev_node);

	return err;
}

error_t vfs_node_move(struct vfs_node_s *node,
					  struct vfs_node_s *parent,
					  const char *name,
					  size_t namelen)
{
	error_t err = 0;
    struct vfs_node_s *parent_src;
    struct vfs_node_s *prev_node;

	vfs_printk("<%s '%s' %p [%s] in %p [%s]... ", __FUNCTION__, name, node, node->name, parent, parent->name);

    if (parent->fs->ops->move == NULL)
        return -ENOTSUP;

    if (parent->fs != node->fs)
        return -ENOTSUP;

    if (vfs_node_is_dangling(parent))
        return -ENOTSUP;

    if (parent->fs->flag_ro)
        return -EPERM;

	VFS_STATS_INC(parent->fs, move_count);

    parent_src = vfs_node_get_parent(node);
    if (!parent_src)
        return -EINVAL;

	vfs_node_2dirlock(parent, parent_src);
	prev_node = vfs_dir_mangled_lookup(parent, name, namelen);

	err = parent->fs->ops->move(node, parent, name, namelen);
	if (err) {
		vfs_printk("fail %d>\n", err);
		goto out;
	}

    vfs_node_parent_nolock_unset(node);
    if (prev_node != NULL)
        vfs_node_parent_nolock_unset(prev_node);

    vfs_node_name_set(node, name, namelen);
    vfs_node_parent_nolock_set(node, parent);

	vfs_printk("ok>\n");

	err = 0;

  out:
	vfs_node_2dirunlock(parent, parent_src);

    if (prev_node)
        vfs_node_refdec(prev_node);

	return err;
}

error_t vfs_node_unlink(struct vfs_node_s *parent,
						const char *name,
						size_t namelen)
{
    if (parent->fs->ops->unlink == NULL)
        return -ENOTSUP;

    if (parent->fs->flag_ro)
        return -EPERM;

	vfs_printk("<%s '%s'... ", __FUNCTION__, name);

	VFS_STATS_INC(parent->fs, unlink_count);

	vfs_node_dirlock(parent);

	struct vfs_node_s *node = vfs_dir_mangled_lookup(parent, name, namelen);

	error_t err = parent->fs->ops->unlink(parent, name, namelen);
	if (err)
		goto out;

	if (node)
        vfs_node_parent_nolock_unset(node);

	err = 0;

  out:
	vfs_node_dirunlock(parent);

	if (node)
        vfs_node_refdec(node);

	vfs_printk(" %s>", strerror(err));

	return err;
}

error_t vfs_node_stat(struct vfs_node_s *node,
					  struct vfs_stat_s *stat)
{
	VFS_STATS_INC(node->fs, stat_count);
	VFS_STATS_INC(node, stat_count);

	return node->fs->ops->stat(node, stat);
}

void vfs_dump(struct vfs_node_s *root)
{
	printk("VFS dump for root %p, fsroot: %p, refcount: %d\n",
		   root, root->fs->root, vfs_fs_refcount(root->fs));
	vfs_dump_item(root, 0);
}

void vfs_fs_dump_stats(struct vfs_fs_s *fs)
{
#if defined(CONFIG_VFS_STATS)
    printk(" node_open:    %d\n", atomic_get(&fs->node_open_count));
    printk(" lookup:       %d\n", atomic_get(&fs->lookup_count));
    printk(" create:       %d\n", atomic_get(&fs->create_count));
    printk(" link:         %d\n", atomic_get(&fs->link_count));
    printk(" unlink:       %d\n", atomic_get(&fs->unlink_count));
    printk(" stat:         %d\n", atomic_get(&fs->stat_count));
    printk(" node_create:  %d\n", atomic_get(&fs->node_create_count));
    printk(" node_destroy: %d\n", atomic_get(&fs->node_destroy_count));
    printk(" file_open:    %d\n", atomic_get(&fs->file_open_count));
    printk(" file_close:   %d\n", atomic_get(&fs->file_close_count));
#endif
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

