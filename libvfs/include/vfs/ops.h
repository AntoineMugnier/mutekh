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

/**
   @file
   @module {Virtual File System}
   @short Core operations on file system nodes
 */

#ifndef _VFS_OPS_H_
#define _VFS_OPS_H_

#include <vfs/types.h>
#include <vfs/fs.h>


/* VFS operations */

/**
   @this mounts a file system inside another, replacing the @tt
   mountpoint node in the VFS. @tt mountpoint must be a directory.

   @param mountpoint directory that must be replaced with fs's
   root.
   @param fs new filesystem to attach at mountpoint
   @return 0 if mounted correctly
 */
error_t vfs_mount(struct vfs_node_s *mountpoint,
				  struct vfs_fs_s *fs);

/**
   @this unmounts a file system. @tt fs must not have any files
   left open. Old directory node will be restored in VFS.

   @param fs filesystem to detach from its mountpoint
   @return 0 if unmounted correctly
 */
error_t vfs_umount(struct vfs_fs_s *fs);

/* Node operations */

/**
   @this looks for a node named @tt name as a child of @tt
   parent. First looks up in the hash table. If @tt name is not found,
   it calls the driver. @tt "." and @tt ".." are not supported.

   @param parent Node to look up from
   @param name Name of the node, must not contain any @tt '/'. It may
   not end with a @tt '\0'
   @param namelen Length of node name
   @param node Returned node
   @return 0 if found

   @this transfers the ownership of @tt node to caller.
*/
error_t vfs_node_lookup(struct vfs_node_s *parent,
						const char *name,
						size_t namelen,
						struct vfs_node_s **node);

/**
   @this creates a new anonymous node in a given FS.

   @param fs File system to create the node in
   @param type Type of the node
   @param node Returned node
   @return 0 if created

   @this transfers the ownership of @tt node to caller.
 */
error_t vfs_node_create(struct vfs_fs_s *fs,
						enum vfs_node_type_e type,
						struct vfs_node_s **node);

/**
   @this links a node in a given parent. As a node must be unique in
   the VFS, node may be cloned in order to be attached where wanted.
   Thus the actually attached node returned in @tt rnode may be
   different from @tt node.

   @param parent Where to attach a new child
   @param node Node to attach
   @param name Name of the new node
   @param namelen Length of name of the new node
   @param rnode Actually attached node
   @return 0 if created

   @this transfers the ownership of @tt rnode to caller, even if it is
   actually @tt node.
 */
error_t vfs_node_link(struct vfs_node_s *parent,
					  struct vfs_node_s *node,
					  const char *name,
					  size_t namelen,
					  struct vfs_node_s **rnode);

/**
   Unlinks a node from its parent.

   @param parent Where to unlink a child
   @param name Name of the node to unlink
   @param namelen Length of name
   @return 0 if unlinked correctly
 */
error_t vfs_node_unlink(struct vfs_node_s *parent,
						const char *name,
						size_t namelen);

/**
   @this retrieves information about a given node.

   @param node Node to retrieve information about
   @param stat User-provided buffer to hold node information
   @return 0 if node was found, or an error
*/
error_t vfs_node_stat(struct vfs_node_s *node,
					  struct vfs_stat_s *stat);

#endif

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
