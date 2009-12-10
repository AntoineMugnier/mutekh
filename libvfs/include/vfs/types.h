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
   @short Core file system nodes and mounts types
 */

#ifndef _VFS_TYPES_H_
#define _VFS_TYPES_H_

#include <vfs/fs.h>
#include <pthread.h>
#include <mutek/printk.h>

#undef GPCT_OBJ_REF_INFO
#define GPCT_OBJ_REF_INFO(p) vfs_printk("<%s %p %d />", __FUNCTION__, p, gpct_atomic_get(&p->obj_entry.refcnt))

#ifdef CONFIG_VFS_VERBOSE
# define vfs_printk(fmt, x...) do{printk("%p: "fmt"\n", pthread_self(), ##x);}while(0)
#else
# define vfs_printk(...) do{}while(0)
#endif

#include <mutek/gpct_mutek_semaphore.h>
#include <hexo/gpct_platform_hexo.h>
#include <hexo/gpct_lock_hexo.h>
#include <gpct/cont_hashlist.h>
#include <gpct/object_refcount.h>
#include <mutek/semaphore.h>


typedef size_t vfs_file_size_t;
typedef uint16_t vfs_node_attr_t;

/**
   @this is a node structure for a given File System. @this is
   FS-dependant.
*/
struct fs_node_s;

enum vfs_node_type_e
{
    /** A directory node */
	VFS_NODE_DIR,
    /** A regular file node */
	VFS_NODE_FILE,
};

struct vfs_node_s;

OBJECT_TYPE     (vfs_node, REFCOUNT, struct vfs_node_s);
OBJECT_PROTOTYPE(vfs_node, static inline, vfs_node);

#define CONTAINER_LOCK_vfs_dir_hash MUTEK_SEMAPHORE

CONTAINER_TYPE    (vfs_dir_hash, HASHLIST,
/**
   @this is a node in the VFS.
 */
struct vfs_node_s
{
	CONTAINER_ENTRY_TYPE(HASHLIST) hash_entry;

    /** File system the node is in */
    struct vfs_fs_s *fs;

    /** Name of the node in its parent directory structure.  Anonymous
        (dandling) nodes should not have any name.  Any unused
        characters in the name should be filled with @tt '\0' */
    char name[CONFIG_VFS_NAMELEN];

    /** Parent node. Root and dandling node have their own pointer here */
	struct vfs_node_s *parent;

    lock_t parent_lock;

    /** Private file system data attached to this node */
    struct fs_node_s *fs_node;

	vfs_node_entry_t obj_entry;

#if defined(CONFIG_VFS_STATS)
    atomic_t lookup_count;
    atomic_t open_count;
    atomic_t close_count;
    atomic_t stat_count;
#endif

    /** Children cache hash */
	vfs_dir_hash_root_t children;
}
, hash_entry, 5);

struct vfs_node_s *vfs_node_get_parent(struct vfs_node_s *node);

/** @see vfs_node_new */
OBJECT_CONSTRUCTOR(vfs_node);
OBJECT_DESTRUCTOR(vfs_node);

OBJECT_FUNC   (vfs_node, REFCOUNT, static inline, vfs_node, obj_entry);
// CONTAINER_FUNC(vfs_dir_hash, HASHLIST, static inline, vfs_dir);


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
//	time_t ctime;
//  /** Access timestamp */
//	time_t atime;
//  /** Modification timestamp */
//	time_t mtime;
//  /** Modes, ... */
//	vfs_node_attr_t attr;
//  /** User ID */
//	uid_t uid;
//  /** Group ID */
//	gid_t gid;
//  /** Device number */
//	dev_t dev;
};

#include <pthread.h>

#if defined(CONFIG_VFS_STATS)
# define VFS_STATS_INC(obj, field) atomic_inc(&(obj)->field)
#else
# define VFS_STATS_INC(x, y) do{}while(0)
#endif

#endif

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

