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

#ifndef _RAMFS_PRIVATE_H_
#define _RAMFS_PRIVATE_H_

#include <hexo/types.h>

#include <vfs/node.h>
#include <vfs/file.h>

#include <gct/container_avl_p.h>

struct ramfs_fs_s {
    struct vfs_fs_s fs;
};

VFS_FS_NODE_OPEN(ramfs_node_open);

#define GCT_CONTAINER_LOCK_ramfs_dir_hash HEXO_LOCK_IRQ
#define GCT_CONTAINER_ALGO_ramfs_dir_hash AVL_P
#define GCT_CONTAINER_COUNTER_ramfs_dir_hash

#define GCT_CONTAINER_REFCOUNT_ramfs_dir_hash ramfs_node

GCT_CONTAINER_TYPES    (ramfs_dir_hash,
struct ramfs_node_s
{
    struct vfs_node_s node;       /* keep first field */

    struct ramfs_node_s *parent;
    union {
        struct ramfs_data_s *data;
        ramfs_dir_hash_root_t children;
    };

    char name[CONFIG_VFS_NAMELEN];
	GCT_CONTAINER_ENTRY(ramfs_dir_hash, hash_entry);

} *, hash_entry);

static inline struct ramfs_node_s *ramfs_node_refinc(struct ramfs_node_s *node)
{
    return (struct ramfs_node_s *) vfs_node_refinc(&node->node);
}

static inline bool_t ramfs_node_refdec(struct ramfs_node_s *node)
{
    return vfs_node_refdec(&node->node);
}

static const bool_t _gct_refcount_ramfs_dir_hash_ramfs_node_enabled = 1;

GCT_CONTAINER_KEY_TYPES(ramfs_dir_hash, PTR, BLOB, name, CONFIG_VFS_NAMELEN);
//GCT_CONTAINER_PROTOTYPES(ramfs_dir_hash, HASHLIST, static inline);

struct ramfs_node_s *ramfs_node_create(
    enum vfs_node_type_e type,
    struct vfs_fs_s *fs,
    struct ramfs_data_s *data,
    const char *name, size_t namelen);

struct ramfs_node_s;

bool_t ramfs_dir_get_nth(struct ramfs_node_s *node, struct vfs_dirent_s *dirent, size_t n);

#endif

// Local Variables:
// tab-width: 4
// c-basic-offset: 4
// c-file-offsets:((innamespace . 0)(inline-open . 0))
// indent-tabs-mode: nil
// End:

// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=4:softtabstop=4

