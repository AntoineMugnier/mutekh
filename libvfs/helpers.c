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
#include <mutek/printk.h>

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

static const char *last_slash_or_end(const char *str)
{
	const char *p = str + strlen(str);

	while ( p > str && *(p-1) == '/' )
		p--;
	return p;
}

error_t vfs_lookup(struct vfs_node_s *root,
				   struct vfs_node_s *cwd,
				   const char *path,
				   struct vfs_node_s **node)
{
	struct vfs_node_s *rnode;
	const char *token = path;
	const char *slash_or_end;

	if ( !path || !root || !cwd || !node )
		return -EINVAL;

	if ( token[0] == '/' ) {
		token = next_nonslash(token);
		cwd = root;
	}

	rnode = vfs_node_refnew(cwd);

	while ( *token ) {
		slash_or_end = next_slash(token);

		if ( (struct vfs_node_s *)rnode == root
			 && ((slash_or_end-token) == 2)
			 && (token[0] == '.')
			 && (token[1] == '.') ) {
			// Trying to go above root, this is not permitted
		} else {
			error_t err = vfs_node_lookup(cwd, token, slash_or_end-token, &rnode);
			vfs_node_refdrop(cwd);
			if ( err )
				return err;
			cwd = rnode;
		}

		if ( *slash_or_end == '\0' )
			break;

		token = next_nonslash(slash_or_end);
	}
	*node = cwd;
	return 0;
}


error_t vfs_create(struct vfs_node_s *root,
				   struct vfs_node_s *cwd,
				   const char *path,
				   enum vfs_node_type_e type,
				   struct vfs_node_s **node)
{
	struct vfs_node_s *rnode;
	error_t err;
	const char *token = path;
	const char *slash_or_end;
	const char *end = last_slash_or_end(path);

	vfs_printk("<create %p %p '%s' %p ", root, cwd, path, node);

	if ( !path || !root || !cwd || !node ) {
		vfs_printk("inval>\n");
		return -EINVAL;
	}

	if ( token[0] == '/' ) {
		vfs_printk("absolute path ");
		token = next_nonslash(token);
		cwd = root;
	}

	rnode = vfs_node_refnew(cwd);

	while ( *token ) {
		slash_or_end = next_slash(token);

		if ( rnode == root
			 && ((slash_or_end-token) == 2)
			 && (token[0] == '.')
			 && (token[1] == '.') ) {
			// Trying to go above root, this is not permitted
		} else {
			err = vfs_node_lookup(cwd, token, slash_or_end-token, &rnode);
			if ( err ) {
				if ( end == slash_or_end )
					goto doit;
				vfs_node_refdrop(cwd);
				vfs_printk("error>\n");
				return err;
			}
			vfs_node_refdrop(cwd);
			cwd = rnode;
		}

		if ( *slash_or_end == '\0' )
			break;

		token = next_nonslash(slash_or_end);
	}
	vfs_printk("exists>\n");
	vfs_node_refdrop(cwd);
	return -EEXISTS;

  doit:
	err = vfs_node_create(cwd->fs, type, &rnode);
	vfs_printk("create %d %p ", err, rnode);
	if ( err ) {
		vfs_node_refdrop(cwd);
		return err;
	}

	err = vfs_node_link(cwd, rnode, token, slash_or_end-token, node);
	vfs_node_refdrop(cwd);
	vfs_printk("link %d %p ", err, *node);
	if ( err ) {
		vfs_node_refdrop(rnode);
		vfs_printk(" err>\n");
		return err;
	}
	vfs_printk(" ok>\n");
	return 0;
}

error_t vfs_open(struct vfs_node_s *root,
				 struct vfs_node_s *cwd,
				 const char *path,
				 enum vfs_open_flags_e flags,
				 struct vfs_file_s **file)
{
	struct vfs_node_s *node;

	vfs_printk("<open '%s' ", path);
	error_t err = vfs_lookup(root, cwd, path, &node);
	if ( err ) {
		if ( err != -ENOENT || !(flags & VFS_OPEN_CREATE) ) {
			vfs_printk("err>\n");
			return err;
		}

		vfs_printk("create ");
		err = vfs_create(root, cwd, path, VFS_NODE_FILE, &node);
		if ( err ) {
			vfs_printk("err>\n");
			return err;
		}
	}

	err = vfs_node_open(node->fs, node, flags, file);
	vfs_node_refdrop(node);
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
	if ( err )
		return err;

	err = vfs_node_stat(node, stat);
	vfs_node_refdrop(node);
	return err;
}

error_t vfs_unlink(struct vfs_node_s *root,
				   struct vfs_node_s *cwd,
				   const char *path)
{
	struct vfs_node_s *node;

	error_t err = vfs_lookup(root, cwd, path, &node);
	if ( err )
		return err;

	if ( !node->parent || node == root ) {
		vfs_node_refdrop(node);
		return -EBUSY;
	}

	err = vfs_node_unlink(node->parent, node->name, strlen(node->name));
	vfs_node_refdrop(node);
	return err;
}

static
void vfs_dump_item(struct vfs_node_s *node,
				   size_t pfx)
{
	size_t i;
	for (i=0; i<pfx; ++i)
		printk(" ");
	switch ( node->type ) {
	case VFS_NODE_FILE:
		printk(" - %d \"%s\" %p (%p)\n", vfs_node_refcount(node), node->name, node, node->parent);
		break;
	case VFS_NODE_DIR:
		printk(" > %d \"%s\" %p (%p)\n", vfs_node_refcount(node), node->name, node, node->parent);
		CONTAINER_FOREACH(vfs_dir_hash, HASHLIST, &node->dir.children, {
				vfs_dump_item(item, pfx+2);
			});
		break;
	}
}

void vfs_dump(struct vfs_node_s *root)
{
	printk("VFS dump for root %p, fsroot: %p, refcount: %d\n",
		   root, root->fs->root, atomic_get(&root->fs->ref));
	vfs_dump_item(root, 0);
}

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

