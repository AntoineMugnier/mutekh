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

#include <vfs/path.h>
#include <vfs/node.h>
#include <vfs/file.h>
#include <vfs/fs.h>

#include "vfs-private.h"

#include <string.h>

static const char *next_nonslash(const char *str)
{
	while ( *str && *str == '/' )
		str++;
	return str;
}

static const char *next_slash(const char *str)
{
	while ( *str && *str != '/' )
		str++;
	return str;
}

static const char *prev_slash(const char *start, const char *str)
{
	const char *p = str;

	while ( p > start && *(p-1) != '/' )
        --p;
    if ( p > start )
        return p-1;
    return start;
}

#if 0
static const char *prev_token(const char *start, const char *str)
{
	const char *p = str;

	while ( p > start && *(p-1) != '/' )
        --p;
    return p;
}
#endif

static const char *last_slash_or_end(const char *str)
{
	const char *p = str + strlen(str);

	while ( p > str && *(p-1) == '/' )
		p--;
	return p;
}

/**
   @this goes as far as it can in an iterative lookup. It always
   returns a node, and pointer to the token where lookup failed.

   If this returns an error, this is just an informative error level
   about the failed lookup.

   @param root Process's root
   @param cwd Process's current working directory
   @param path Path to lookup
   @param end Where to end lookup. @tt end must be between @tt path
   and @tt{path + strlen(path)}
   @param next_place Next token to lookup (where lookup failed)
   @param last_found Node just before failed lookup

   @this transfers the ownership of @tt node to caller
 */
static
error_t vfs_lookup_part(
    struct vfs_node_s *root,
    struct vfs_node_s *cwd,
    const char *path,
    const char *end,
    const char **next_place,
    struct vfs_node_s **last_found)
{
    const char *to_lookup = path;
    error_t err = 0;

    vfs_printk("<%s '%s'->'%s' ", __FUNCTION__, path, end);

    /* Absolute lookup */
    if (to_lookup[0] == '/') {
        vfs_printk("absolute ");
        to_lookup = next_nonslash(to_lookup);
        cwd = root;
    }

    *last_found = vfs_node_refinc(cwd);

    vfs_printk("start %p ", *last_found);

    while (to_lookup < end) {
        const char *slash_or_end = next_slash(to_lookup);
        size_t to_lookup_len = slash_or_end - to_lookup;
        struct vfs_node_s *next_found = NULL;
        err = 0;

        vfs_printk("part \"%s\"/%d ", to_lookup, to_lookup_len);

        if (to_lookup_len == 2 && to_lookup[0] == '.' && to_lookup[1] == '.') {
            vfs_printk(".. ");
            // Trying to go above root, this is not permitted
            if (*last_found == root)
                next_found = vfs_node_refinc(root);
            else
                next_found = vfs_node_get_parent(*last_found);

        } else if (to_lookup_len == 1 && to_lookup[0] == '.') {
            vfs_printk(". ");
            next_found = vfs_node_refinc(*last_found);

        } else {
            vfs_printk("[vfs] ");
            err = vfs_node_lookup(*last_found, to_lookup, to_lookup_len, &next_found);
            if (err)
                next_found = NULL;
        }

        if (!next_found)
            break;

        vfs_node_refdec(*last_found);
        *last_found = next_found;

        to_lookup = next_nonslash(slash_or_end);
    }

    *next_place = to_lookup < end ? to_lookup : end;

    vfs_printk("stopped at '%s'->'%s' n: %p : %d>", *next_place, end, *last_found, err);
    return err;
}

error_t vfs_lookup(struct vfs_node_s *root,
				   struct vfs_node_s *cwd,
				   const char *path,
				   struct vfs_node_s **node)
{
	if (!path || !root || !cwd || !node)
		return -EINVAL;

	const char *end = last_slash_or_end(path);
	const char *where;

	error_t err = vfs_lookup_part(root, cwd, path, end, &where, node);
    vfs_printk("<%s %d got %p reffed %d %p/%p %s/%s>", __FUNCTION__, err, *node,
               vfs_node_refcount(*node), where, end, where, end);

    assert(where <= end);

    if (!err && where == end) {
        vfs_printk("<%s did valid lookup %p %d>", __FUNCTION__, *node,
                   vfs_node_refcount(*node));
        return 0;
    }

    vfs_node_refdec(*node);
    *node = NULL;

    if (err)
        return err;
    else
        return -ENOENT;
}

error_t vfs_create(struct vfs_node_s *root,
				   struct vfs_node_s *cwd,
				   const char *path,
				   enum vfs_node_type_e type,
				   struct vfs_node_s **linked_node)
{
	if (!path || !root || !cwd || !linked_node)
		return -EINVAL;

	const char *end = last_slash_or_end(path);
	const char *prev = prev_slash(path, end);
	const char *last_part = next_nonslash(prev);
    size_t last_part_len = end - last_part;
    struct vfs_node_s *parent;
	const char *stopped_at;
    error_t err;
    struct vfs_node_s *created_node;

    /* We wont create "." or ".." */
    if (last_part[0] == '.'
        && (last_part_len == 1 || (last_part[1] == '.' && last_part_len == 2)))
        return -EINVAL;

	err = vfs_lookup_part(root, cwd, path, end, &stopped_at, &parent);

    if (err != -ENOENT || stopped_at != last_part) {
        vfs_printk(" ! exists>\n");
        vfs_node_refdec(parent);

        if (err)
            return err;

        return -EEXISTS;
    }

    err = vfs_node_anon_create(parent->fs, type, &created_node);
    vfs_printk("create %d %p ", err, created_node);
    if (err) {
        vfs_node_refdec(parent);
        vfs_printk(" err>\n");
        return err;
    }

    err = vfs_node_link(created_node, parent, last_part, last_part_len, linked_node);
    vfs_node_refdec(parent);
    vfs_node_refdec(created_node);
    vfs_printk("link %d %p>", err, *linked_node);
    return err;
}

error_t vfs_open(
    struct vfs_node_s *root,
    struct vfs_node_s *cwd,
    const char *path,
    enum vfs_open_flags_e flags,
    struct vfs_file_s **file)
{
    if (!path || !root || !cwd || !file)
        return -EINVAL;

    const char *end = last_slash_or_end(path);
    const char *prev = prev_slash(path, end);
    const char *last_part = next_nonslash(prev);
    struct vfs_node_s *created_node;
    struct vfs_node_s *linked_node;
    struct vfs_node_s *node_or_parent;
    const char *stopped_at;

    error_t err = vfs_lookup_part(root, cwd, path, end, &stopped_at, &node_or_parent);
    if (err == -ENOENT && stopped_at >= last_part && flags & VFS_OPEN_CREATE)
        goto do_create;

    if (!err)
        goto do_open;

    vfs_printk(" parent %s>\n", strerror(err));
    vfs_node_refdec(node_or_parent);
    return err;

 do_create:
    // node_or_parent is parent

    vfs_printk("creating %s in %s", last_part, node_or_parent->name);

    err = vfs_node_anon_create(node_or_parent->fs, VFS_NODE_FILE, &created_node);
    if (err) {
        vfs_printk(" create error>\n");
        vfs_node_refdec(node_or_parent);
        return err;
    }

    err = vfs_node_link(created_node, node_or_parent, last_part, end-last_part, &linked_node);
    vfs_node_refdec(node_or_parent);
    vfs_node_refdec(created_node);

    if (err) {
        vfs_printk(" link error>\n");
        return err;
    }

    node_or_parent = linked_node;
    
 do_open:
    // node_or_parent is node to open

    vfs_printk("opening %s ", node_or_parent->name);

    err = vfs_node_open(node_or_parent, flags, file);
    vfs_node_refdec(node_or_parent);
    vfs_printk("%d %p>\n", err, *file);
    return err;
}

error_t vfs_stat(struct vfs_node_s *root,
				 struct vfs_node_s *cwd,
				 const char *path,
				 struct vfs_stat_s *stat)
{
	struct vfs_node_s *node;

	error_t err = vfs_lookup(root, cwd, path, &node);
	if (err)
		return err;

	err = vfs_node_stat(node, stat);
	vfs_node_refdec(node);

	return err;
}

error_t vfs_link(struct vfs_node_s *root,
                 struct vfs_node_s *cwd,
                 const char *src,
                 const char *dst)
{
	if ( !root || !cwd || !src || !dst )
		return -EINVAL;

	const char *dst_end = last_slash_or_end(dst);
	const char *dst_prev = prev_slash(dst, dst_end);

	const char *dst_last_part = next_nonslash(dst_prev);

    /* We wont create "." or ".." */
    if ( dst_last_part[0] == '.'
         && (((dst_end-dst_last_part) == 1)
             || ((dst_last_part[1] == '.')
                 && ((dst_end-dst_last_part) == 2))
             ))
        return -EINVAL;

    struct vfs_node_s *parent;
	const char *stopped_at;

	error_t err = vfs_lookup_part(root, cwd, dst, dst_end, &stopped_at, &parent);
    if ( (err == -ENOENT) && (stopped_at >= dst_last_part) )
        goto do_link;

    vfs_printk(" exists>\n");
    vfs_node_refdec(parent);
    return err;

  do_link:

    {
        struct vfs_node_s *rnode;

        err = vfs_lookup(root, cwd, src, &rnode);
        vfs_printk("src lookup %d %p ", err, rnode);
        if ( err ) {
            vfs_node_refdec(parent);
            vfs_printk(" err>\n");
            return err;
        }

        struct vfs_node_s *new_node;
        err = vfs_node_link(rnode, parent, dst_last_part, dst_end-dst_last_part, &new_node);
        if ( err == 0 )
            vfs_node_refdec(new_node);
        vfs_node_refdec(parent);
        vfs_node_refdec(rnode);
        vfs_printk("link %d %p>", err, new_node);
        return err;
    }
}

error_t vfs_unlink(struct vfs_node_s *root,
                   struct vfs_node_s *cwd,
                   const char *path)
{
	struct vfs_node_s *node;

	error_t err = vfs_lookup(root, cwd, path, &node);
	if (err)
		return err;

    // TODO also check we dont try to delete parent of any mountpoint
    // in the system.
	if (node->fs->root == node) {
		vfs_node_refdec(node);
		return -EBUSY;
	}

    struct vfs_node_s *parent = vfs_node_get_parent(node);
    if ( parent == NULL ) {
        vfs_node_refdec(node);
        return -EUNKNOWN;
    }

	err = vfs_node_unlink(parent, node->name, strlen(node->name));
	vfs_node_refdec(node);
    vfs_node_refdec(parent);    
	return err;
}
