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

/**
   @file
   @module {Libraries::Virtual File System}
   @short Core file system node
 */

#ifndef _VFS_NODE_H_
#define _VFS_NODE_H_

#include <hexo/decls.h>

C_HEADER_BEGIN

#include <vfs/defs.h>

#include <mutek/semaphore.h>

#include <gct/container_avl_p.h>
#include <gct/refcount.h>

#define GCT_CONTAINER_ALGO_vfs_dir_hash AVL_P

enum vfs_node_type_e
{
    /** A directory node */
    VFS_NODE_DIR,
    /** A regular file node */
    VFS_NODE_FILE,
};

/** @This initializes common fields of a VFS node object.
 Name may be NULL or 0-sized. */
error_t vfs_node_init(struct vfs_node_s *node,
                      struct vfs_fs_s *fs,
                      enum vfs_node_type_e type,
                      const char *name, size_t name_size);

/** @This frees resources allocated by @ref vfs_node_init. */
void vfs_node_cleanup(struct vfs_node_s *node);

/** @This calls the @ref vfs_node_s::close and the @ref
    vfs_node_cleanup functions then free the node object. This is
    called when the node refcount reaches 0. */
void vfs_node_destroy(struct vfs_node_s *node);

//#define CONTAINER_LOCK_vfs_dir_hash MUTEK_SEMAPHORE

GCT_CONTAINER_TYPES    (vfs_dir_hash,
/**  @this is a node in the VFS. */
struct vfs_node_s
{
    /** File system the node is in */
    struct vfs_fs_s *fs;

    enum vfs_node_type_e type;

    /** Name of the node in its parent directory structure.  Anonymous
        (dangling) nodes should not have any name.  Any unused
        characters in the name should be filled with @tt '\0' */
    char name[CONFIG_VFS_NAMELEN];

    /** @internal
        Parent node.

        Root has its own pointer here, dangling nodes have NULL.

        Accesses to this value must be protected for atomicity with
        @tt parent_lock.

        Code external to VFS code MUST use @ref vfs_node_get_parent.
    */
    struct vfs_node_s *parent;
    /** @internal
        Lock protecting accesses to parent */
    lock_t parent_lock;

#if defined(CONFIG_VFS_STATS)
    /** @multiple
        Statistics counter
     */
    atomic_t lookup_count;
    atomic_t open_count;
    atomic_t close_count;
    atomic_t stat_count;
#endif

    /**
       @internal
       Children cache hash.
       
       Accesses to this value must be protected through @tt
       dir_semaphore.
    */
    vfs_dir_hash_root_t children;

    /** @internal
        Semaphore protecting @tt children */
    struct semaphore_s dir_semaphore;

    /** @internal */
    GCT_CONTAINER_ENTRY(vfs_dir_hash, hash_entry);
    /** @internal Object-management related */
    GCT_REFCOUNT_ENTRY(obj_entry);

} *, hash_entry);

GCT_REFCOUNT(vfs_node, struct vfs_node_s *, obj_entry);

static inline struct vfs_node_s *vfs_node__refinc(struct vfs_node_s *node, const char *func)
{
    vfs_printk("<%s %s %p %d>", __FUNCTION__, func, node, vfs_node_refcount(node));
    return vfs_node_refinc(node);
}

static inline bool_t vfs_node__refdec(struct vfs_node_s *node, const char *func)
{
    vfs_printk("<%s %s %p %d>", __FUNCTION__, func, node, vfs_node_refcount(node));
    return vfs_node_refdec(node);
}

#define vfs_node_refinc(x) vfs_node__refinc(x, __FUNCTION__)
#define vfs_node_refdec(x) vfs_node__refdec(x, __FUNCTION__)

/**
   @this is the vfs_node_stat() operation response buffer.
 */
struct vfs_stat_s
{
    /** File or directory */
    enum vfs_node_type_e type;

    /** File size in bytes, or directory entry count excluding "." and
        ".." */
    vfs_file_size_t size;

    /** Count of links to the data on disk */
    size_t nlink;

//  /** Creation timestamp */
//  time_t ctime;
//  /** Access timestamp */
//  time_t atime;
//  /** Modification timestamp */
//  time_t mtime;
//  /** Modes, ... */
//  vfs_node_attr_t attr;
//  /** User ID */
//  uid_t uid;
//  /** Group ID */
//  gid_t gid;
//  /** Device number */
//  dev_t dev;
};

struct vfs_dirent_s
{
    /** Name of the directory entry, asciiZ */
	char name[CONFIG_VFS_NAMELEN + 1];
    /** Type of node */
	enum vfs_node_type_e type;
    /** Size of file in bytes, or count of children nodes excluding
        "." and ".." */
	size_t size;
};


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

   @param mountpoint Mountpoint where to umount a file system
   @return 0 if unmounted correctly
 */
error_t vfs_umount(struct vfs_node_s *mountpoint);

/* Node operations */

/** @this increases the node reference count and return the node itself. */
//struct vfs_node_s * vfs_node_refinc(struct vfs_node_s * node);

/** @this decreases the node reference count and may delete the node if no more reference exist. */
//bool_t vfs_node_refdec(struct vfs_node_s * node);

/**
   @this looks for a node named @tt name as a child of @tt
   parent. First looks up in the hash table. If @tt name is not found,
   it calls the driver. @tt '.' and @tt '..' are not supported.

   @param parent Node to look up from
   @param name Name of the node, must not contain any @tt '/'. It may
          not end with a @tt '\0' . May be eiter a full length file system
          name or a vfs shortened node name.
   @param namelen Length of node name
   @param node Returned node
   @return 0 if found

   @this transfers the ownership of @tt node to caller.
   @see vfs_fs_lookup_t @see vfs_lookup
   @see vfs_name_mangle
*/
error_t vfs_node_lookup(struct vfs_node_s *parent,
						const char *fullname,
						size_t fullnamelen,
						struct vfs_node_s **node);

/**
   @this opens an existing node in a given FS.

   @tt flags inform the file system about the actions intended on the
   file.  @ref VFS_OPEN_READ and @ref VFS_OPEN_WRITE may be ored
   together.  For directories, the only valid operation is @ref
   VFS_OPEN_READ | @ref VFS_OPEN_DIR.

   @this fail if trying to open a file with @ref VFS_OPEN_DIR flag present.

   This function must only honor @ref VFS_OPEN_READ, @ref VFS_OPEN_WRITE and
   @ref VFS_OPEN_DIR flags.  Other flags must be ignored (even
   @ref VFS_OPEN_CREATE and @ref VFS_OPEN_APPEND).

   This function must not create new files implicitely.

   It relies on the @ref vfs_fs_node_open_t fs drivers operation, refer for details.
   @this transfers the ownership of @tt node to caller.
   @see vfs_open @see vfs_fs_node_open_t
*/
error_t vfs_node_open(struct vfs_node_s *node,
                      enum vfs_open_flags_e flags,
                      struct vfs_file_s **file);

/**
   @this creates a new anonymous node in a given FS.

   It relies on the @ref vfs_fs_create_t fs drivers operation, refer for details.
   @this transfers the ownership of @tt node to caller.
   @see vfs_create
 */
error_t vfs_node_anon_create(struct vfs_fs_s *fs,
						     enum vfs_node_type_e type,
						     struct vfs_node_s **node);

/**
   @this links a node in a given parent. As a node must be unique in
   the VFS, node may be cloned in order to be attached where wanted.
   Thus the actually attached node returned in @tt rnode may be
   different from @tt node.

   @param node Node to attach
   @param parent Where to attach a new child
   @param fullname Name of the new node, may be a long file system
          entry name but will be shortened for use as vfs node name.
   @param fullnamelen Length of name of the new node
   @param rnode Actually attached node
   @return 0 if created

   @this transfers the ownership of @tt rnode to caller, even if it is
   actually @tt node.
   @see vfs_fs_link_t
   @see vfs_name_mangle
 */
error_t vfs_node_link(struct vfs_node_s *node,
					  struct vfs_node_s *parent,
					  const char *fullname,
					  size_t fullnamelen,
					  struct vfs_node_s **rnode);

/**
   @this moves a node in a given parent.

   @param node Node to attach to a new parent
   @param parent Where to attach node
   @param fullname Name of the new node, may be a long file system
          entry name but will be shortened for use as vfs node name.
   @param fullnamelen Length of name of the new node
   @return 0 if created

   @see vfs_fs_move_t
   @see vfs_name_mangle
 */
error_t vfs_node_move(struct vfs_node_s *node,
					  struct vfs_node_s *parent,
					  const char *fullname,
					  size_t fullnamelen);

/**
   Unlinks a node from its parent.

   @param parent Where to unlink a child
   @param fullname Name of the node, must not contain any @tt '/'. It may
          not end with a @tt '\0' . May be eiter a full length file system
          name or a vfs shortened node name.
   @param fullnamelen Length of name
   @return 0 if unlinked correctly
   @see vfs_fs_unlink_t @see vfs_unlink
 */
error_t vfs_node_unlink(struct vfs_node_s *parent,
						const char *fullname,
						size_t fullnamelen);

/**
   @this retrieves information about a given node.

   @param node Node to retrieve information about
   @param stat User-provided buffer to hold node information
   @return 0 if node was found, or an error
   @see vfs_fs_stat_t @see vfs_stat
*/
error_t vfs_node_stat(struct vfs_node_s *node,
					  struct vfs_stat_s *stat);

struct vfs_node_s *vfs_node_get_parent(struct vfs_node_s *node);

ssize_t vfs_node_get_name(struct vfs_node_s *node,
                          char *name,
                          size_t namelen);

struct vfs_fs_s *vfs_node_get_fs(struct vfs_node_s *node);

C_HEADER_END

#endif

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

