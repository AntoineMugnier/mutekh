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
   @short File system driver operations interface


   File system
 */

#ifndef _VFS_FS_H_
#define _VFS_FS_H_

#include <hexo/types.h>
#include <hexo/atomic.h>
#include <hexo/error.h>

struct vfs_fs_s;

enum vfs_node_type_e;
enum vfs_open_flags_e;

struct vfs_node_s;
struct vfs_file_s;

struct vfs_stat_s;

/** @this defines the fs unmountable test prototype */
#define VFS_FS_CAN_UNMOUNT(x) bool_t (x)(struct vfs_fs_s *fs)

/**
   This function asks the file system whether its current internal state
   allows it to be unmounted.  This function should take opened files,
   directories and anonymous nodes into account.

   This function does not have to ensure the filesystem will stay unmountable.
   Ensuring nobody takes new references on the file system being
   unmounted is VFS's job.

   @param fs Fs state to probe for unmounting
   @return 1 if file system can be unmounted, else 0

   @csee #VFS_FS_CAN_UNMOUNT
 */
typedef VFS_FS_CAN_UNMOUNT(vfs_fs_can_unmount_t);

/** @this defines the fs node open operation prototype */
#define VFS_FS_NODE_OPEN(x) error_t (x)(struct vfs_node_s *node,	\
										enum vfs_open_flags_e flags,			\
										struct vfs_file_s **file)

/**
   This function opens a node for read/write operations.  @tt flags inform the
   file system about the actions intended on the file.  @ref VFS_OPEN_READ
   and @ref VFS_OPEN_WRITE may be ored together.  For directories, the only
   valid operation is @ref VFS_OPEN_READ | @ref VFS_OPEN_DIR.

   This function must fail if trying to open a file with @ref VFS_OPEN_DIR flag
   present.

   This function must only honor @ref VFS_OPEN_READ, @ref VFS_OPEN_WRITE and
   @ref VFS_OPEN_DIR flags.  Other flags must be ignored (even
   @ref VFS_OPEN_CREATE and @ref VFS_OPEN_APPEND).

   This function must not create new files implicitely.

   @param node Node to open. It may be an anonymous node (not present
   in any directory)
   @param flags Mode to open the file in
   @param file Returned file descriptor on success
   @return 0 on success, or an error code

   This function transfers the ownership of @tt *file to the caller

   @csee #VFS_FS_NODE_OPEN
 */
typedef VFS_FS_NODE_OPEN(vfs_fs_node_open_t);


/** @this defines the fs lookup operation prototype */
#define VFS_FS_LOOKUP(x) error_t (x)(struct vfs_node_s *ref,		\
									   const char *name,				\
									   size_t namelen,					\
									   struct vfs_node_s **node)

/**
   This function searches for a given name in a directory node. This function is only
   valid for directory nodes.

   This function must not create new nodes in the actual file system.

   @param ref Reference directory node
   @param name Name to lookup for, it may not end with @tt '\0'.
   @param namelen Length of name, excluding any @tt '\0'
   @param node Returned node if found.  File system must not insert
   the @tt node in the @tt ref's children hash
   @return 0 on success, or an error code

   This function transfers the ownership of @tt *node to the caller

   @csee #VFS_FS_LOOKUP
 */
typedef VFS_FS_LOOKUP(vfs_fs_lookup_t);


/** @this defines the fs create operation prototype */
#define VFS_FS_CREATE(x) error_t (x)(struct vfs_fs_s *fs,	\
									   enum vfs_node_type_e type,		\
									   struct vfs_node_s **node)

/**
   This function creates a new anonymous node in a given file system.

   @param fs The fs state
   @param type Node type
   @param node Returned created node
   @return 0 on success, or an error code

   This function transfers the ownership of @tt *node to the caller

   @csee #VFS_FS_CREATE
 */
typedef VFS_FS_CREATE(vfs_fs_create_t);

/** @this defines the fs link operation prototype */
#define VFS_FS_LINK(x) error_t (x)(struct vfs_node_s *parent,	   \
									 struct vfs_node_s *node,	   \
									 const char *name,				   \
									 size_t namelen,				   \
									 struct vfs_node_s **rnode)

/**
   This function links a node in a parent directory node.  Filesystem may not
   support linking operation on all node types, or may not support
   linking at all.  The only operation expected to succeed everywhere
   is linking an anonymous node created with @ref vfs_fs_create_t.

   As nodes must be unique in the VFS, the node present at destination
   point on link after the operation (@tt{rnode}) may be different from
   the passed node (@tt{node}).

   @param parent Parent directory node to attach @tt node in
   @param node Node to attach in @tt parent
   @param name Name to lookup for, it may not end with @tt '\0'.
   @param namelen Length of name, excluding any @tt '\0'
   @param rnode Actually attached node, may be @tt node or another new
   node.
   @return 0 on success, or an error code

   This function transfers the ownership of @tt *rnode to the caller, even if
   @tt *rnode is actually @tt node

   @csee #VFS_FS_LINK
 */
typedef VFS_FS_LINK(vfs_fs_link_t);


/** @this defines the fs unlink operation prototype */
#define VFS_FS_UNLINK(x) error_t (x)(struct vfs_node_s *parent,  \
									   const char *name,			   \
									   size_t namelen)

/**
   This function removes a node from the file system.  Node may still be
   referenced or open, and may become a dandling anonymous file.

   @param parent Directory node where to unlink a child
   @param name Name of child to unlink, it may not end with @tt '\0'.
   @param namelen Length of name, excluding any @tt '\0'
   @return 0 on success, or an error code

   @csee #VFS_FS_UNLINK
 */
typedef VFS_FS_UNLINK(vfs_fs_unlink_t);



/** @this defines the fs stat operation prototype */
#define VFS_FS_STAT(x) error_t (x)(struct vfs_node_s *node,  \
								   struct vfs_stat_s *stat)

/**
   This function retrieves informations about a given node.  This node may be
   a file or a directory.

   @param node Node to retrieve information about
   @param stat Stat buffer to store information into
   @return 0 on success, or an error code

   @csee #VFS_FS_STAT
 */
typedef VFS_FS_STAT(vfs_fs_stat_t);


/** VFS node file system private data deleter prototype macro */
#define VFS_NODE_FS_PRIV_DELETER(x) void (x)(struct vfs_node_s *node)

/**
   @this is a callback provided by the file system implementation
   called on node descruction.  It must delete any private data
   attached to the node.

   There is no way for this callback to cancel the deletion.

   File system must not perform any long-time operation in this call.

   @param node Deleted node

   @csee #VFS_NODE_FS_PRIV_DELETER
 */
typedef VFS_NODE_FS_PRIV_DELETER(vfs_node_fs_priv_deleter_t);

/**
   @this is an opened file system state.
 */
struct vfs_fs_s
{
	atomic_t ref;

	vfs_fs_node_open_t *node_open;
	vfs_fs_lookup_t *lookup;
	vfs_fs_create_t *create;
	vfs_fs_link_t *link;
	vfs_fs_unlink_t *unlink;
	vfs_fs_stat_t *stat;
	vfs_fs_can_unmount_t *can_unmount;

	struct vfs_node_s *old_node;
	struct vfs_node_s *root;
};

#endif

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4
