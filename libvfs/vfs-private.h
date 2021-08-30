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
   @module {Virtual File System}
   @short Core file system nodes and mounts types
 */

#ifndef _VFS_PRIVATE_H_
#define _VFS_PRIVATE_H_

#include <vfs/node.h>

static inline
bool_t vfs_node_is_dangling(struct vfs_node_s *node)
{
    return node->parent == NULL;
}

GCT_CONTAINER_KEY_TYPES(vfs_dir_hash, PTR, BLOB, name, CONFIG_VFS_NAMELEN);

GCT_CONTAINER_KEY_FCNS(vfs_dir_hash, ASC, static inline, vfs_dir, name,
                       init, destroy, push, lookup, remove);

static inline void vfs_node_dirlock(struct vfs_node_s *node)
{
    semaphore_take(&node->dir_semaphore, 1);
}

static inline void vfs_node_dirunlock(struct vfs_node_s *node)
{
    semaphore_give(&node->dir_semaphore, 1);
}

static inline bool_t vfs_node_dirtrylock(struct vfs_node_s *node)
{
    return semaphore_try_take(&node->dir_semaphore, 1);
}

void vfs_node_parent_nolock_unset(struct vfs_node_s *node);

void vfs_node_lru_rehash(struct vfs_node_s *node);

#endif
